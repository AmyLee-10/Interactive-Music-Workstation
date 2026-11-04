module sd_card_bmp #(
parameter integer CLK_FREQ_HZ       = 100_000_000,
parameter integer LONG_PRESS_MS     = 1000,
parameter [31:0]  SCAN_START_SECTOR = 32'd0,
parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
parameter         FAT32_MODE = 1'b1 // 1=normal FAT32; 0=original raw IMG
)(
input                       clk,
input                       rst,
input                       key_next,
input                       key_auto,
output reg [3:0]            state_code,
input  [15:0]               bmp_width,
input  [15:0]               bmp_height,
output reg                  display_valid,
// E9.3.7 D read-only loader diagnostic bus.
// bit10 = BMP producer emitted >= 307200 pixel words this load.
// bits9:0 retain C-version meanings, including 9=write_done_seen.
// No changes to BMP parser, memory writes, or frame-commit handshakes.
output wire [10:0] e933_img_status,
// P3.7 read-only loader diagnostics. These are NOT used to control SD/FAT.
output wire dbg_sd_init_done,
output wire [3:0] dbg_fat_error,
output wire [3:0] dbg_fat_stage,

output                      piano_bank_found,

// V16: dynamic C4 sample-cache write stream from TF scan.
output                      piano_c4_pcm_wr_en,
output [11:0]               piano_c4_pcm_wr_addr,
output [15:0]               piano_c4_pcm_wr_data,
output                      piano_c4_pcm_ready,
output                      e93_bank_start,
output [20:0]               e93_bank_words,
output                      e93_pcm_word_en,
output [31:0]               e93_pcm_word,
output                      e93_scan_complete,
// P4.3 independent PCM bank completion status from the SDRAM writer.
input                       piano_dma_ready_sd,
input                       violin_dma_ready_sd,

input                       write_finish_toggle,
output reg [1:0]            write_buf_idx,
output reg [1:0]            disp_buf_idx,

// V15F: actual frame-latched buffer currently used by video_clk,
// already synchronized back into this sd_card_clk domain by top.
input      [1:0]            video_disp_buf_idx,

output                      write_req,
input                       write_req_ack,
output                      write_en,
output [31:0]               write_data,
output                      SD_nCS,
output                      SD_DCLK,
output                      SD_MOSI,
input                       SD_MISO
);
localparam integer LONG_PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
wire key_next_press;
wire key_auto_press;
wire key_auto_stable;
wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;
wire             bank_found;
wire             image_source_ok;
wire             image_load_failed;
wire             image_fatal_error;
wire [3:0]      fat_error_code;
reg              image_fault_seen; // sticky; do not retry a poisoned video writer
assign piano_bank_found = bank_found;
assign dbg_sd_init_done = sd_init_done;
assign dbg_fat_error = fat_error_code;
assign dbg_fat_stage = bmp_state_code;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [28:0]       auto_cnt;       // 300M<2^29: save 3 FF
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              prev_req_pending;
reg              load_busy;
reg [26:0]       long_press_cnt; // 100MHz*1s=100M <2^27; save 5 FF
reg              long_press_marked;
// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
// E9.3.4: sticky frame-writer completion prevents a one-cycle pulse
// from being lost when the SDRAM writer finishes before BMP reading.
reg              write_done_seen;
// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;
wire [3:0]       bmp_state_code;
wire auto_tick;
wire [1:0] next_from_current;

// The logical commit (disp_buf_idx) may lead the actual video display by up to
// one frame.  Never reuse the previous buffer until video has acknowledged
// the commit and is really reading the committed buffer.
wire video_buf_caught_up;
// Count only the pixels emitted by BMP parser while THIS frame is active.
// Saturate at one 640x480 frame. Observation only; does not gate writes.
reg [18:0] e937_src_pixel_count;
wire e937_src_pixels_full = (e937_src_pixel_count == 19'd307200);
always @(posedge clk or posedge rst) begin
    if (rst)
        e937_src_pixel_count <= 19'd0;
    else if (!sd_init_done || !load_busy)
        e937_src_pixel_count <= 19'd0;
    else if (bmp_data_wr_en && e937_src_pixel_count < 19'd307200)
        e937_src_pixel_count <= e937_src_pixel_count + 19'd1;
end
assign e933_img_status = {e937_src_pixels_full,
                          write_done_seen, display_valid, scan_done,
                          img_found_count, load_busy, source_done_seen,
                          first_image_committed, bmp_ready};

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt == 32'd299_999_999);//100_000_000 = 1s
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

// First image has no ownership conflict.  After display_valid becomes true,
// a new load may start only after the video side is confirmed to be using
// the last committed display buffer.
assign video_buf_caught_up =
    (!display_valid) || (video_disp_buf_idx == disp_buf_idx);
key_press_debounce #(
.CLK_FREQ_HZ (CLK_FREQ_HZ),
.DEBOUNCE_MS (20)
) u_key_next (
.clk        (clk),
.rst        (rst),
.button_in  (key_next),
.press_pulse(key_next_press)
);
key_press_debounce #(
.CLK_FREQ_HZ (CLK_FREQ_HZ),
.DEBOUNCE_MS (20)
) u_key_auto (
.clk          (clk),
.rst          (rst),
.button_in    (key_auto),
.press_pulse  (key_auto_press),
.button_state (key_auto_stable)
);
function [1:0] next_index_limited;
input [1:0] cur;
input [2:0] count;
begin
case (count)
3'd0: next_index_limited = 2'd0;
3'd1: next_index_limited = 2'd0;
3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
endcase
end
endfunction
function [1:0] prev_index_limited;
input [1:0] cur;
input [2:0] count;
begin
case(count)
3'd0: prev_index_limited = 2'd0;
3'd1: prev_index_limited = 2'd0;
3'd2: prev_index_limited = (cur == 2'd0) ? 2'd1 : 2'd0;
3'd3: prev_index_limited = (cur == 2'd0) ? 2'd2 : (cur - 2'd1);
default: prev_index_limited = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);
endcase
end
endfunction
function [31:0] sector_lut;
input [1:0] idx;
begin
case (idx)
2'd0: sector_lut = img_sector0;
2'd1: sector_lut = img_sector1;
2'd2: sector_lut = img_sector2;
2'd3: sector_lut = img_sector3;
default: sector_lut = img_sector0;
endcase
end
endfunction
function [1:0] next_buf_lut;
input [1:0] cur_disp_buf;
input       valid_now;
begin
if (!valid_now)
next_buf_lut = 2'd0;                 // 首图固定写 buffer0
else if (cur_disp_buf == 2'd0)
next_buf_lut = 2'd1;
else
next_buf_lut = 2'd0;
end
endfunction
always @(posedge clk or posedge rst) begin
if (rst) begin
wrfin_tgl_sync        <= 3'b000;
scan_start_pulse      <= 1'b0;
load_start_pulse      <= 1'b0;
load_sector           <= 32'd0;
scan_kicked           <= 1'b0;
first_image_committed <= 1'b0;
image_fault_seen       <= 1'b0;
auto_play_en          <= 1'b0;
auto_cnt              <= 29'd0;
img_found_count       <= 3'd0;
img_idx               <= 2'd0;
load_idx              <= 2'd0;
pending_buf_idx       <= 2'd0;
write_buf_idx         <= 2'd0;
disp_buf_idx          <= 2'd0;
img_sector0           <= 32'd0;
img_sector1           <= 32'd0;
img_sector2           <= 32'd0;
img_sector3           <= 32'd0;
next_req_pending      <= 1'b0;
prev_req_pending      <= 1'b0;
long_press_cnt        <= 27'd0;
long_press_marked     <= 1'b0;
load_busy             <= 1'b0;
source_done_seen      <= 1'b0;
write_done_seen       <= 1'b0;
display_valid         <= 1'b0;
state_code            <= 4'd0;
end else begin
wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
// 数码管显示当前图片编号：img_idx=0~3，对用户显示为1~4
state_code <= image_fault_seen ? 4'hE : ({2'b00, img_idx} + 4'd1);
scan_start_pulse <= 1'b0;
load_start_pulse <= 1'b0;
    if (!sd_init_done) begin
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        image_fault_seen       <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 29'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        prev_req_pending      <= 1'b0;
        long_press_cnt        <= 27'd0;
        long_press_marked     <= 1'b0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
    end else begin
        // 扫描阶段缓存前 4 张图的起始 sector
        if (scan_found_valid) begin
            case (img_found_count)
                3'd0: img_sector0 <= scan_found_sector;
                3'd1: img_sector1 <= scan_found_sector;
                3'd2: img_sector2 <= scan_found_sector;
                3'd3: img_sector3 <= scan_found_sector;
                default: ;
            endcase
            if (img_found_count < 3'd4)
                img_found_count <= img_found_count + 3'd1;
        end
        // 记住 bmp_read 已经把源图送完 FIFO，但还不能切显示，得等整帧写完
        if (load_busy && image_source_ok && !image_fault_seen && !image_load_failed)
            source_done_seen <= 1'b1;
        // Remember both completion events independently: either one may
        // arrive first. The old one-cycle write_finish pulse could be lost
        // if TF sector reading reported bmp_ready a few clocks later.
        if (load_busy && write_finish_pulse)
            write_done_seen <= 1'b1;
        if (load_busy && source_done_seen && write_done_seen &&
            !image_load_failed && !image_fault_seen) begin
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
write_done_seen       <= 1'b0;
            disp_buf_idx          <= pending_buf_idx;
            img_idx               <= load_idx;
            display_valid         <= 1'b1;
            first_image_committed <= 1'b1;
        end
        // A failed FAT32 image read must never commit a partial new buffer.
        // If the frame writer was already armed, it may be waiting for missing
        // pixels. Freeze further frame loading until an explicit system reset.
        if (image_load_failed && load_busy) begin
            image_fault_seen  <= 1'b1;
            load_busy         <= 1'b0;
            source_done_seen  <= 1'b0;
            write_done_seen   <= 1'b0;
            next_req_pending  <= 1'b0;
            prev_req_pending  <= 1'b0;
            auto_play_en      <= 1'b0;
        end
        // 上电后自动发起一次“扫描前 4 张 BMP”
        if (!scan_kicked && bmp_ready) begin
            scan_start_pulse      <= 1'b1;
            scan_kicked           <= 1'b1;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 29'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            next_req_pending      <= 1'b0;
            display_valid         <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
write_done_seen       <= 1'b0;
        end else begin
            // P5.0: KEY2 short RELEASE -> previous BMP;
            //        KEY2 HOLD >=1s -> toggle AUTO slideshow once.
            // This reuses the ORIGINAL 100MHz hold counter and
            // debounced active-low key state, no second timer/FF.
            if (key_auto_stable == 1'b0) begin
                if (long_press_cnt < LONG_PRESS_CYCLES)
                    long_press_cnt <= long_press_cnt + 27'd1;
                if ((long_press_cnt >= LONG_PRESS_CYCLES-1) &&
                    !long_press_marked) begin
                    long_press_marked <= 1'b1;
                    if (scan_done && img_found_count>3'd1) begin
                        auto_play_en <= ~auto_play_en;
                        auto_cnt <= 29'd0;
                    end
                end
            end else begin
                if ((long_press_cnt!=27'd0) && !long_press_marked &&
                    scan_done && (img_found_count>3'd0))
                    prev_req_pending <= 1'b1;
                long_press_cnt <= 27'd0;
                long_press_marked <= 1'b0;
            end
            // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
            if (key_next_press && scan_done && (img_found_count > 3'd0))
                next_req_pending <= 1'b1;
            // 自动播放 1s 计数：只有当前没有写图任务时才计时
            if (scan_done && auto_play_en && display_valid && !load_busy && video_buf_caught_up &&
                first_image_committed && (img_found_count > 3'd1)) begin
                if (auto_tick)
                    auto_cnt <= 29'd0;
                else
                    auto_cnt <= auto_cnt + 29'd1;
            end else begin
                auto_cnt <= 29'd0;
            end
            // 首图自动加载到 buffer0
            if (scan_done && !image_fault_seen && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                load_idx         <= 2'd0;
                load_sector      <= img_sector0;
                pending_buf_idx  <= 2'd0;
                write_buf_idx    <= 2'd0;
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                next_req_pending <= 1'b0;
                auto_cnt         <= 32'd0;
            end
            // 手动下一张优先：写到“非当前显示”的另一块 buffer
            else if (scan_done && !image_fault_seen && bmp_ready && display_valid && !load_busy && video_buf_caught_up && next_req_pending && (img_found_count != 3'd0)) begin
                load_idx         <= next_from_current;
                load_sector      <= sector_lut(next_from_current);
                pending_buf_idx  <= next_buf_lut(video_disp_buf_idx, display_valid);
                write_buf_idx    <= next_buf_lut(video_disp_buf_idx, display_valid);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                next_req_pending <= 1'b0;
                auto_cnt         <= 32'd0;
            end
            // 手动上一张：KEY2长按触发
            else if (scan_done && !image_fault_seen && bmp_ready && display_valid && !load_busy && video_buf_caught_up && prev_req_pending && (img_found_count != 3'd0)) begin
                load_idx         <= prev_index_limited(img_idx, img_found_count);
                load_sector      <= sector_lut(prev_index_limited(img_idx, img_found_count));
                pending_buf_idx  <= next_buf_lut(video_disp_buf_idx, display_valid);
                write_buf_idx    <= next_buf_lut(video_disp_buf_idx, display_valid);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                prev_req_pending <= 1'b0;
                auto_cnt         <= 32'd0;
            end
            // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
            else if (scan_done && !image_fault_seen && bmp_ready && display_valid && !load_busy && video_buf_caught_up && auto_play_en && auto_tick && (img_found_count > 3'd1)) begin
                load_idx         <= next_from_current;
                load_sector      <= sector_lut(next_from_current);
                pending_buf_idx  <= next_buf_lut(video_disp_buf_idx, display_valid);
                write_buf_idx    <= next_buf_lut(video_disp_buf_idx, display_valid);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                auto_cnt         <= 32'd0;
            end
        end
    end
end
end
generate
if (FAT32_MODE) begin : g_fat32_loader
    fat32_bmp_read #(.IMAGE_USE_FOLDER(1'b0)) bmp_read_m0(
.clk                    (clk),
.rst                    (rst),
.ready                  (bmp_ready),
.scan_start             (scan_start_pulse),
.scan_start_sector      (SCAN_START_SECTOR),
.scan_max_sector        (SCAN_MAX_SECTOR),
.scan_target_count      (SCAN_TARGET_COUNT),
.scan_done              (scan_done),
.scan_found_valid       (scan_found_valid),
.scan_found_sector      (scan_found_sector),
.scan_found_total       (scan_found_total),
.bank_found             (bank_found),
.piano_c4_pcm_wr_en     (piano_c4_pcm_wr_en),
.piano_c4_pcm_wr_addr   (piano_c4_pcm_wr_addr),
.piano_c4_pcm_wr_data   (piano_c4_pcm_wr_data),
.piano_c4_pcm_ready     (piano_c4_pcm_ready),
.e93_bank_start (e93_bank_start),
.e93_bank_words (e93_bank_words),
.e93_pcm_word_en (e93_pcm_word_en),
.e93_pcm_word (e93_pcm_word),
.e93_scan_complete (e93_scan_complete),
.piano_dma_ready_sd     (piano_dma_ready_sd),
.violin_dma_ready_sd    (violin_dma_ready_sd),
.load_start             (load_start_pulse),
.load_sector            (load_sector),
.sd_init_done           (sd_init_done),
.state_code             (bmp_state_code),
.bmp_width              (bmp_width),
.bmp_height             (bmp_height),
.write_req              (write_req),
.write_req_ack          (write_req_ack),
.sd_sec_read            (sd_sec_read),
.sd_sec_read_addr       (sd_sec_read_addr),
.sd_sec_read_data       (sd_sec_read_data),
.sd_sec_read_data_valid (sd_sec_read_data_valid),
.sd_sec_read_end        (sd_sec_read_end),
.bmp_data_wr_en         (bmp_data_wr_en),
.bmp_data               (bmp_data),
    .image_source_ok(image_source_ok),
    .image_load_failed(image_load_failed),
    .image_fatal_error(image_fatal_error),
    .fat_error_code(fat_error_code)
    );
end else begin : g_raw_loader
    bmp_read bmp_read_m0(
.clk                    (clk),
.rst                    (rst),
.ready                  (bmp_ready),
.scan_start             (scan_start_pulse),
.scan_start_sector      (SCAN_START_SECTOR),
.scan_max_sector        (SCAN_MAX_SECTOR),
.scan_target_count      (SCAN_TARGET_COUNT),
.scan_done              (scan_done),
.scan_found_valid       (scan_found_valid),
.scan_found_sector      (scan_found_sector),
.scan_found_total       (scan_found_total),
.bank_found             (bank_found),
.piano_c4_pcm_wr_en     (piano_c4_pcm_wr_en),
.piano_c4_pcm_wr_addr   (piano_c4_pcm_wr_addr),
.piano_c4_pcm_wr_data   (piano_c4_pcm_wr_data),
.piano_c4_pcm_ready     (piano_c4_pcm_ready),
.e93_bank_start (e93_bank_start),
.e93_bank_words (e93_bank_words),
.e93_pcm_word_en (e93_pcm_word_en),
.e93_pcm_word (e93_pcm_word),
.e93_scan_complete (e93_scan_complete),
.load_start             (load_start_pulse),
.load_sector            (load_sector),
.sd_init_done           (sd_init_done),
.state_code             (bmp_state_code),
.bmp_width              (bmp_width),
.bmp_height             (bmp_height),
.write_req              (write_req),
.write_req_ack          (write_req_ack),
.sd_sec_read            (sd_sec_read),
.sd_sec_read_addr       (sd_sec_read_addr),
.sd_sec_read_data       (sd_sec_read_data),
.sd_sec_read_data_valid (sd_sec_read_data_valid),
.sd_sec_read_end        (sd_sec_read_end),
.bmp_data_wr_en         (bmp_data_wr_en),
.bmp_data               (bmp_data)
);
    assign image_source_ok = bmp_ready;
    assign image_load_failed = 1'b0;
    assign image_fatal_error = 1'b0;
    assign fat_error_code = 4'd0;
end
endgenerate
sd_card_top sd_card_top_m0(
.clk                    (clk),
.rst                    (rst),
.SD_nCS                 (SD_nCS),
.SD_DCLK                (SD_DCLK),
.SD_MOSI                (SD_MOSI),
.SD_MISO                (SD_MISO),
.sd_init_done           (sd_init_done),
.sd_sec_read            (sd_sec_read),
.sd_sec_read_addr       (sd_sec_read_addr),
.sd_sec_read_data       (sd_sec_read_data),
.sd_sec_read_data_valid (sd_sec_read_data_valid),
.sd_sec_read_end        (sd_sec_read_end),
.sd_sec_write           (1'b0),
.sd_sec_write_addr      (32'd0),
.sd_sec_write_data      (),
.sd_sec_write_data_req  (),
.sd_sec_write_end       ()
);
endmodule
module key_press_debounce #(
parameter integer CLK_FREQ_HZ = 100_000_000,
parameter integer DEBOUNCE_MS = 20
)(
input  wire clk,
input  wire rst,
input  wire button_in,    // 默认：松开=1，按下=0
output reg  press_pulse,
output reg  button_state
);
localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;
always @(posedge clk or posedge rst) begin
if (rst) begin
button_sync0  <= 1'b1;
button_sync1  <= 1'b1;
button_stable <= 1'b1;
button_state  <= 1'b1;
cnt           <= 32'd0;
press_pulse   <= 1'b0;
end else begin
button_sync0 <= button_in;
button_sync1 <= button_sync0;
press_pulse  <= 1'b0;
    if (button_sync1 == button_stable) begin
        cnt <= 32'd0;
    end else begin
        if (cnt >= DEBOUNCE_CYCLES - 1) begin
            if (button_stable && !button_sync1)
                press_pulse <= 1'b1;
            button_stable <= button_sync1;
            button_state  <= button_sync1;
            cnt           <= 32'd0;
        end else begin
            cnt <= cnt + 32'd1;
        end
    end
end
end
endmodule
