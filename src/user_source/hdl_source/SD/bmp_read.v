module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // V15B: minimal SBNK discovery.
    output reg                  bank_found,

    // V16: while the normal TF scan walks past PIANO.BIN, capture the first
    // 4096 signed PCM16 samples immediately after the 512-byte SBNK header.
    // The current bank tool always stores C4 as entry0 at offset 512.
    output reg                  piano_c4_pcm_wr_en,
    output reg [11:0]           piano_c4_pcm_wr_addr,
    output reg [15:0]           piano_c4_pcm_wr_data,
    output reg                  piano_c4_pcm_ready,

    // E9.3 raw PCM16 bank stream; SD scan and image control unchanged.
    output reg                  e93_bank_start,
    output reg [20:0]           e93_bank_words,
    output reg                  e93_pcm_word_en,
    output reg [31:0]           e93_pcm_word,
    output reg                  e93_scan_complete,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;
// E8.3: split BMP scan header checking, sector arithmetic and scan FSM
// update into separate 100 MHz stages.  The SD controller sees an idle gap
// while these stages execute, preserving its S_READ_END handshake.
localparam ST_SCAN_CALC = 3'd5;
localparam ST_SCAN_APPLY = 3'd6;
localparam ST_LOAD_CHECK = 3'd7;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [7:0]  bank_magic_2;
reg [7:0]  bank_magic_3;
reg [31:0] e93_tag;
reg [31:0] e93_header_words;
reg [31:0] e93_expected_crc;
reg [31:0] e93_rate_tag;
reg        e93_capture_active;
reg [20:0] e93_words_received;
reg [23:0] e93_pack_bytes;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
// Pipeline registers for the scan-completion timing path.
reg        scan_is_bmp;
reg        scan_is_bank;
reg [23:0] scan_skip_sectors;
reg [31:0] scan_next_bmp_sector;
reg        load_is_bmp;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

// V16 C4 capture state.
// 4096 samples * 2 bytes = 8192 bytes = exactly 16 SD sectors.
reg        c4_capture_active;
reg [11:0] c4_capture_index;
reg [7:0]  c4_pcm_low_byte;

wire header_match;
wire bank_header_match;
wire bmp_data_valid;
wire [23:0] file_sector_count_ceil;
wire [31:0] next_scan_sector_if_miss;

assign ready = (state == ST_IDLE);
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width[15:0]  == bmp_width) &&
                      (height[15:0] == bmp_height) &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);

// Keep the bank detector deliberately tiny for timing closure.
assign bank_header_match = (header_0 == "S") &&
                           (header_1 == "B") &&
                           (bank_magic_2 == "N") &&
                           (bank_magic_3 == "K");
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
// ceil(file_len / 512), with zero-length files advanced by one sector.
// Avoid the original 32-bit (file_len+511) carry chain and its overflow.
assign file_sector_count_ceil = (file_len == 32'd0) ? 24'd1 :
                                ({1'b0,file_len[31:9]} +
                                 {23'd0,(|file_len[8:0])});
assign next_scan_sector_if_miss  = scan_sector + 32'd1;

// This pulse occurs at the END of the SBNK header sector.
// Capture therefore starts from the very next sector, which is byte offset512
// in the PIANO.BIN format generated by our bank tool.
wire e93_header_match = bank_header_match &&
                        (e93_tag == 32'h46333945) &&
                        (e93_header_words == 32'd844672) &&
                        (e93_expected_crc == 32'h0302c286) &&
                        (e93_rate_tag == 32'h314b3834);
wire e93_capture_start = (state == ST_SCAN) && sd_sec_read_end &&
                         e93_header_match && !e93_capture_active &&
                         !e93_scan_complete;
wire c4_capture_start =
    (state == ST_SCAN) && sd_sec_read_end && bank_header_match &&
    !e93_header_match;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        bank_magic_2 <= 8'd0;
        bank_magic_3 <= 8'd0;
        e93_tag <= 32'd0;
        e93_header_words <= 32'd0;
        e93_expected_crc <= 32'd0;
        e93_rate_tag <= 32'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : begin
                bank_magic_2 <= sd_sec_read_data;
                file_len[7:0] <= sd_sec_read_data;
            end
            10'd3 : begin
                bank_magic_3 <= sd_sec_read_data;
                file_len[15:8] <= sd_sec_read_data;
            end
            10'd4 : begin file_len[23:16] <= sd_sec_read_data; e93_tag[7:0] <= sd_sec_read_data; end
            10'd5 : begin file_len[31:24] <= sd_sec_read_data; e93_tag[15:8] <= sd_sec_read_data; end

            10'd6 : e93_tag[23:16] <= sd_sec_read_data;
            10'd7 : e93_tag[31:24] <= sd_sec_read_data;
            10'd8 : e93_header_words[7:0] <= sd_sec_read_data;
            10'd9 : e93_header_words[15:8] <= sd_sec_read_data;
            10'd10: begin pixel_offset[7:0] <= sd_sec_read_data; e93_header_words[23:16] <= sd_sec_read_data; end
            10'd11: begin pixel_offset[15:8] <= sd_sec_read_data; e93_header_words[31:24] <= sd_sec_read_data; end
            10'd12: begin pixel_offset[23:16] <= sd_sec_read_data; e93_expected_crc[7:0] <= sd_sec_read_data; end
            10'd13: begin pixel_offset[31:24] <= sd_sec_read_data; e93_expected_crc[15:8] <= sd_sec_read_data; end

            10'd14: e93_expected_crc[23:16] <= sd_sec_read_data;
            10'd15: e93_expected_crc[31:24] <= sd_sec_read_data;
            10'd16: e93_rate_tag[7:0] <= sd_sec_read_data;
            10'd17: e93_rate_tag[15:8] <= sd_sec_read_data;
            10'd18: begin width[7:0] <= sd_sec_read_data; e93_rate_tag[23:16] <= sd_sec_read_data; end
            10'd19: begin width[15:8] <= sd_sec_read_data; e93_rate_tag[31:24] <= sd_sec_read_data; end
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

// E9.3 PCM16 stream: exact 844672 x 32-bit payload words, following the
// 512-byte SBNK/E93F header.  The four data bytes are little-endian.
// Never read beyond the declared count into an adjacent BMP file.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        e93_bank_start <= 1'b0;
        e93_bank_words <= 21'd0;
        e93_pcm_word_en <= 1'b0;
        e93_pcm_word <= 32'd0;
        e93_scan_complete <= 1'b0;
        e93_capture_active <= 1'b0;
        e93_words_received <= 21'd0;
        e93_pack_bytes <= 24'd0;
    end else begin
        e93_bank_start <= 1'b0;
        e93_pcm_word_en <= 1'b0;
        if (!sd_init_done || scan_start) begin
            e93_scan_complete <= 1'b0;
            e93_capture_active <= 1'b0;
            e93_words_received <= 21'd0;
            e93_bank_words <= 21'd0;
        end else if (e93_capture_start) begin
            e93_capture_active <= 1'b1;
            e93_bank_start <= 1'b1;
            e93_bank_words <= 21'd844672;
            e93_words_received <= 21'd0;
        end else if (e93_capture_active && state == ST_SCAN &&
                     sd_sec_read_data_valid) begin
            case (rd_cnt[1:0])
                2'd0: e93_pack_bytes[7:0] <= sd_sec_read_data;
                2'd1: e93_pack_bytes[15:8] <= sd_sec_read_data;
                2'd2: e93_pack_bytes[23:16] <= sd_sec_read_data;
                2'd3: begin
                    e93_pcm_word <= {sd_sec_read_data,e93_pack_bytes};
                    e93_pcm_word_en <= 1'b1;
                    e93_words_received <= e93_words_received + 21'd1;
                    if (e93_words_received == 21'd844671) begin
                        e93_capture_active <= 1'b0;
                        e93_scan_complete <= 1'b1;
                    end
                end
            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// V16: C4 PCM capture during the existing sector scan.
// No extra TF reader is created, so BMP scanning and PIANO.BIN discovery still
// share the one already-stable SD controller.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        c4_capture_active  <= 1'b0;
        c4_capture_index   <= 12'd0;
        c4_pcm_low_byte    <= 8'd0;
        piano_c4_pcm_wr_en   <= 1'b0;
        piano_c4_pcm_wr_addr <= 12'd0;
        piano_c4_pcm_wr_data <= 16'd0;
        piano_c4_pcm_ready   <= 1'b0;
    end
    else if (!sd_init_done) begin
        c4_capture_active  <= 1'b0;
        c4_capture_index   <= 12'd0;
        c4_pcm_low_byte    <= 8'd0;
        piano_c4_pcm_wr_en   <= 1'b0;
        piano_c4_pcm_wr_addr <= 12'd0;
        piano_c4_pcm_wr_data <= 16'd0;
        piano_c4_pcm_ready   <= 1'b0;
    end
    else begin
        piano_c4_pcm_wr_en <= 1'b0;

        // A new full scan invalidates the previous dynamic sample cache.
        if (scan_start) begin
            c4_capture_active <= 1'b0;
            c4_capture_index  <= 12'd0;
            piano_c4_pcm_ready <= 1'b0;
        end
        else if (c4_capture_start) begin
            c4_capture_active <= 1'b1;
            c4_capture_index  <= 12'd0;
            piano_c4_pcm_ready <= 1'b0;
        end
        else if (c4_capture_active &&
                 (state == ST_SCAN) &&
                 sd_sec_read_data_valid) begin

            // PCM16 in PIANO.BIN is little-endian: low byte first.
            if (rd_cnt[0] == 1'b0) begin
                c4_pcm_low_byte <= sd_sec_read_data;
            end
            else begin
                // Hold the write address/data stable for the following
                // sd_card_clk edge, where the dual-clock BRAM write port
                // consumes this one-cycle write-enable.
                piano_c4_pcm_wr_addr <= c4_capture_index;
                piano_c4_pcm_wr_data <= {sd_sec_read_data, c4_pcm_low_byte};
                piano_c4_pcm_wr_en   <= 1'b1;

                if (c4_capture_index == 12'd4095) begin
                    c4_capture_active <= 1'b0;
                    piano_c4_pcm_ready <= 1'b1;
                end
                else begin
                    c4_capture_index <= c4_capture_index + 12'd1;
                end
            end
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        bank_found        <= 1'b0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        scan_is_bmp       <= 1'b0;
        scan_is_bank      <= 1'b0;
        scan_skip_sectors <= 24'd0;
        scan_next_bmp_sector <= 32'd0;
        load_is_bmp       <= 1'b0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        bank_found        <= 1'b0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        scan_is_bmp       <= 1'b0;
        scan_is_bank      <= 1'b0;
        scan_skip_sectors <= 24'd0;
        scan_next_bmp_sector <= 32'd0;
        load_is_bmp       <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    bank_found       <= 1'b0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read      <= 1'b0;
                    // Stage 1: latch the header decision and sector count.
                    // No address update / scan-limit compare in this cycle.
                    scan_is_bmp      <= header_match;
                    scan_is_bank     <= bank_header_match;
                    scan_skip_sectors <= file_sector_count_ceil;
                    state            <= ST_SCAN_CALC;
                end
            end

            ST_SCAN_CALC: begin
                state_code <= 4'd2;
                sd_sec_read <= 1'b0;
                // Stage 2: one sector-address adder, with registered input.
                scan_next_bmp_sector <= scan_sector + {8'd0,scan_skip_sectors};
                state <= ST_SCAN_APPLY;
            end

            ST_SCAN_APPLY: begin
                state_code <= 4'd2;
                sd_sec_read <= 1'b0;
                // Stage 3: apply previously registered classification/address.
                if (scan_is_bmp) begin
                    scan_found_valid  <= 1'b1;
                    scan_found_sector <= scan_sector;
                    scan_found_total  <= scan_found_total + 3'd1;

                    if ((scan_found_total + 3'd1 >= scan_target_count) ||
                        (scan_next_bmp_sector > scan_max_sector)) begin
                        scan_done        <= 1'b1;
                        state            <= ST_IDLE;
                    end else begin
                        state            <= ST_SCAN;
                    end
                    sd_sec_read_addr <= scan_next_bmp_sector;
                    scan_sector      <= scan_next_bmp_sector;
                end else if (scan_is_bank) begin
                    bank_found <= 1'b1;
                    if (scan_sector >= scan_max_sector) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= next_scan_sector_if_miss;
                        scan_sector      <= next_scan_sector_if_miss;
                        state            <= ST_SCAN;
                    end
                end else begin
                    if (scan_sector >= scan_max_sector) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= next_scan_sector_if_miss;
                        scan_sector      <= next_scan_sector_if_miss;
                        state            <= ST_SCAN;
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // Load validation also stays behind a register.
                    load_is_bmp <= header_match;
                    state <= ST_LOAD_CHECK;
                end
            end

            ST_LOAD_CHECK: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b0;
                if (load_is_bmp) begin
                    write_req        <= 1'b1;
                    sd_sec_read_addr <= load_sector_latched;
                    state            <= ST_LOAD_WAIT;
                end else begin
                    state <= ST_IDLE;
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
