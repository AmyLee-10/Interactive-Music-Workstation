
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 图片:下一张;编辑:上一音轨; SW3=1:C4
    input                       key2,           // 图片:短按上一张/长按轮播;编辑:下一音轨;SW3=1:D4
    input                       key4,           // 图片:迷你编辑器;编辑:MIC录音;SW3=1:E4
    input                       sw1,            // P5.0: 全局音乐播放/暂停(1/0)
    input                       sw2,            // P5.0: 节拍器声音开/关(1/0)
    input                       sw3,            // 琴键测试模式 + OSD：1=KEY1/KEY2/KEY4 -> C4/D4/E4，并显示底部OSD
    input                       sw4,            // 编辑器总模式开关
    input                       pcb_k1,         // PCB J1 K1~K5: active LOW
    input                       pcb_k2,
    input                       pcb_k3,
    input                       pcb_k4,
    input                       pcb_k5,
    input                       enc_sw,         // PCB J1 EC11 switch: active LOW
    input                       enc_a,
    input                       enc_b,
    input  [4:0]                kbd_row,        // J2 keyboard rows: pull-up, LOW=pressed
    inout  [4:0]                kbd_col,        // J2 keyboard columns: LOW or Hi-Z

    output [3:0]                led,            // 4个板载独立LED，同时闪烁表示一拍
    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,
    // P2.3 INPUT-DIAG: mic pins on HX4S20 J1 expansion.
    output                      mic_i2s_lrcl, // GPIOA_31 / P15
    input                       mic_i2s_dout, // GPIOA_32 / R16, weak pulldown
    output                      mic_i2s_bclk  // GPIOA_35 / R14
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
// E935 LUT audit: independent compile-time switches.
// Keep at 1 for baseline; set each to 0 in a SEPARATE experimental build.
localparam E935_KEEP_EDITOR_DEBUG_UI = 1'b0;
localparam E935_KEEP_LEGACY_C4_DIAG = 1'b0;
// P1 retires the fixed 440 Hz test override; the REAL square instrument remains.
localparam E936_KEEP_FIXED_440_DIAG = 1'b0;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data_base;
wire [23:0] vout_data_score;
wire [23:0] vout_data;
wire        display_valid;
wire        piano_bank_found_sd;
// E9.3 48-kHz/PCM16 original 25-key bank stream.
wire e93_bank_start_sd, e93_word_en_sd, e93_scan_complete_sd;
wire [20:0] e93_bank_words_sd;
wire [31:0] e93_pcm_word_sd;
wire e93_bank_loaded_mem;
wire e964_violin_loaded_mem;
reg [2:0] e964_violin_ready_video_ff;
reg [2:0] e964_piano_ready_sd_ff, e964_violin_ready_sd_ff;
wire e964_violin_ready_video = e964_violin_ready_video_ff[2];
wire e964_piano_ready_sd = e964_piano_ready_sd_ff[2];
wire e964_violin_ready_sd = e964_violin_ready_sd_ff[2];
always @(posedge sd_card_clk or posedge rst_tf) begin
    if(rst_tf) begin
        e964_piano_ready_sd_ff <= 3'b000;
        e964_violin_ready_sd_ff <= 3'b000;
    end else begin
        e964_piano_ready_sd_ff <= {e964_piano_ready_sd_ff[1:0],
                                  e93_bank_loaded_mem};
        e964_violin_ready_sd_ff <= {e964_violin_ready_sd_ff[1:0],
                                   e964_violin_loaded_mem};
    end
end
// E9.3.12 I: latch the first *actual source* PCM32 payload word.
// The value remains stable after capture; a synchronized valid level
// makes the multi-bit diagnostic safe to observe in the video clock domain.
reg [31:0] e941_tf_first_word_sd;
reg e941_tf_first_valid_sd;
always @(posedge sd_card_clk or posedge rst_tf) begin
    if (rst_tf) begin
        e941_tf_first_word_sd <= 32'd0;
        e941_tf_first_valid_sd <= 1'b0;
    end else begin
        if (e93_bank_start_sd)
            e941_tf_first_valid_sd <= 1'b0;
        if (e93_word_en_sd && (!e941_tf_first_valid_sd || e93_bank_start_sd)) begin
            e941_tf_first_word_sd <= e93_pcm_word_sd;
            e941_tf_first_valid_sd <= 1'b1;
        end
    end
end
// E9.3.26 LOCAL-PCM-A/B: non-SDRAM PCM control path.
// Reuse the EXISTING I2S C4 dual-port BRAM (4096x16, no extra RAM block).
// E93F delivers one 32-bit word for every TWO little-endian PCM16 frames.
// The SD stream produces a word every four data bytes, providing room
// to emit its low and high halves on successive sd_card_clk cycles.
// This is a passive tap: it NEVER changes TF parsing, SDRAM commands,
// SDRAM arbitration, framebuffers, or the E9.3.25 piano renderer.
reg [11:0] e926_c4_copy_addr_sd;
reg [15:0] e926_c4_upper_sd;
reg        e926_c4_upper_pending_sd;
reg        e926_c4_ready_sd;
reg        e926_c4_wr_en_sd;
reg [11:0] e926_c4_wr_addr_sd;
reg [15:0] e926_c4_wr_data_sd;
always @(posedge sd_card_clk or posedge rst_tf) begin
    if (rst_tf) begin
        e926_c4_copy_addr_sd <= 12'd0;
        e926_c4_upper_sd <= 16'd0;
        e926_c4_upper_pending_sd <= 1'b0;
        e926_c4_ready_sd <= 1'b0;
        e926_c4_wr_en_sd <= 1'b0;
        e926_c4_wr_addr_sd <= 12'd0;
        e926_c4_wr_data_sd <= 16'd0;
    end else begin
        e926_c4_wr_en_sd <= 1'b0;
        if (e93_bank_start_sd) begin
            e926_c4_copy_addr_sd <= 12'd0;
            e926_c4_upper_pending_sd <= 1'b0;
            e926_c4_ready_sd <= 1'b0;
        end else if (!e926_c4_ready_sd) begin
            if (e926_c4_upper_pending_sd) begin
                e926_c4_wr_en_sd <= 1'b1;
                e926_c4_wr_addr_sd <= e926_c4_copy_addr_sd;
                e926_c4_wr_data_sd <= e926_c4_upper_sd;
                e926_c4_copy_addr_sd <= e926_c4_copy_addr_sd + 12'd1;
                e926_c4_upper_pending_sd <= 1'b0;
                if (e926_c4_copy_addr_sd == 12'd4095)
                    e926_c4_ready_sd <= 1'b1;
            end else if (e93_word_en_sd) begin
                e926_c4_wr_en_sd <= 1'b1;
                e926_c4_wr_addr_sd <= e926_c4_copy_addr_sd;
                e926_c4_wr_data_sd <= e93_pcm_word_sd[15:0];
                e926_c4_copy_addr_sd <= e926_c4_copy_addr_sd + 12'd1;
                e926_c4_upper_sd <= e93_pcm_word_sd[31:16];
                e926_c4_upper_pending_sd <= 1'b1;
            end
        end
    end
end

// E9.3.11 H: frame read priority and response ownership; F writer unchanged.
// BMP lower 10 status bits are unchanged. Bit10 is producer pixel target.
wire [10:0] e933_img_status_sd;
reg [10:0] e933_img_ff1, e933_img_ff2;
reg e933_mem_nonzero_seen, e933_c4_header_ok, e933_c4_header_bad;
// D read-only pixel-domain copies of the SDRAM writer's progress.
reg [1:0] e937_mem_full_video_sync;
reg [1:0] e937_mem_any_video_sync;
// D display (green=event true, red at 8=source finished but
// SDRAM video writer never issued any write for this frame).
wire [8:0] e933_view_status = {
    (video_fifo_underflow_v ||
     (e933_img_ff2[3] && e933_img_ff2[2] && !e937_mem_any_video_sync[1])), // 8: video FIFO empty / no image writes
    e933_img_ff2[9],              // 7: writer completion captured
    e933_img_ff2[2],              // 6: source reports idle/complete
    e933_img_ff2[8],              // 5: display_valid
    e933_img_ff2[1],              // 4: first image committed
    e933_img_ff2[3],              // 3: load_busy
    (e933_img_ff2[3] ? e937_mem_full_video_sync[1] : e933_c4_header_bad), // 2: loading=count / idle=C4 mismatch
    (e933_img_ff2[3] ? e933_img_ff2[10] : e933_c4_header_ok),   // 1: loading=count / idle=C4 exact match
    e933_img_ff2[7]               // 0: BMP scan_done
};
reg [2:0] e93_loaded_sync_video;
wire e93_loaded_video = e93_loaded_sync_video[2];
wire e93_bank_fifo_overflow_sd;
// P1 event protocol, all signals synchronous to video_clk (25 MHz).
// {on[29], instrument[28:27], track[26:25], note_id[24:16],
//  pitch[15:11], velocity[10:7], gain[6:0]}.
// instrument: 0=piano PCM, 1=square oscillator, 2=sustained violin PCM.
wire [174:0] e93_pitch_gain; // existing square-level diagnostic bus
wire         e936_note_evt_valid;
wire         e936_note_evt_ready;
wire [29:0]  e936_note_evt_data;
wire [3:0]   e936_track_mute_mask;

wire [23:0] e93_pcm_audio;
// P4.8 MIC spectrum frame bus; declared BEFORE editor instance.
wire [39:0] e982_mic_spectrum_audio;
wire e982_mic_spectrum_toggle;
wire e93_mem_req_toggle;
wire [20:0] e93_mem_req_addr;
wire e93_mem_rsp_toggle;
wire [31:0] e93_mem_rsp_word;
wire e93_pcm_underrun;
// E9.3.17 N-DIAG: reader observability only; no SDRAM signal mux change.
wire [31:0] e944_diag_status_mem, e944_diag_expected_mem;
wire [31:0] e944_diag_observed_mem;
wire [20:0] e944_diag_bad_addr_mem;
always @(posedge video_clk or posedge rst_all) begin
    if(rst_all) begin
        e93_loaded_sync_video <= 3'b0;
        e964_violin_ready_video_ff <= 3'b0;
    end else begin
        e93_loaded_sync_video <= {e93_loaded_sync_video[1:0],
                                  e93_bank_loaded_mem};
        e964_violin_ready_video_ff <= {e964_violin_ready_video_ff[1:0],
                                       e964_violin_loaded_mem};
    end
end


// V16 dynamic C4 PCM cache, written in sd_card_clk and read in audio_mclk.
wire        piano_c4_pcm_wr_en_sd;
wire [11:0] piano_c4_pcm_wr_addr_sd;
wire [15:0] piano_c4_pcm_wr_data_sd;
wire        piano_c4_pcm_ready_sd;

wire [3:0]  state_code;
wire [6:0]  seg_image;
wire [6:0]  seg_bpm_hundreds;
wire [6:0]  seg_bpm_tens;
wire [6:0]  seg_bpm_ones;
wire [6:0]  seg_beat_tens;
wire [6:0]  seg_beat_ones;

// SW1=0 时使用的节拍器设置（本阶段只做按键设置和数码管显示，后续再接音频节拍逻辑）
reg  [9:0]  metronome_bpm;      // 1~999，默认120；预留给后续音频节拍逻辑
reg  [4:0]  beats_per_bar;      // 1~16，默认4
reg  [3:0]  bpm_hundreds;       // BPM百位，BCD保存，避免显示路径做除法/取模
reg  [3:0]  bpm_tens;           // BPM十位
reg  [3:0]  bpm_ones;           // BPM个位
wire [3:0]  beat_tens;
wire [3:0]  beat_ones;

wire        key1_setting_press;
wire        key2_setting_press;
wire        key4_setting_press;
wire        key1_button_state;
wire        key2_button_state;
wire        key4_button_state;

// SW3=1 时进入临时琴键测试模式。当前仍然是单音优先级模式：C4 > D4 > E4。
reg  [1:0]  sw3_sync_clk;
reg  [1:0]  sw3_sync_sd;
wire        piano_mode_clk;
wire        piano_mode_sd;

// E1 editor mode. SW4 has the highest functional priority.
reg  [1:0]  sw4_sync_clk;
reg  [1:0]  sw4_sync_sd;
wire        editor_mode_clk;
wire        editor_mode_sd;

// Editor E6 runs exclusively in the pixel clock domain, separate from
// existing 50 MHz metronome / I2S / TF memory pipelines.
wire        piano_key_active;
wire [6:0]  piano_selected_note;

// V10：当前板载三个琴键直接提供三个独立持续 gate 给音频域。
// Note Event 暂时继续保留给 OSD/历史音符显示，不在本步重构视频事件链。
wire        piano_c4_gate;
wire        piano_d4_gate;
wire        piano_e4_gate;
// 25-key matrix is sampled in the video domain, then synchronized into the
// original 50 MHz control domain for C4 PCM / D4,E4 DDS gate sharing.
wire [24:0] e6_keys_down, e6_keys_pressed;
reg [24:0] keyboard_50_ff1, keyboard_50_ff2;
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        keyboard_50_ff1 <= 25'd0;
        keyboard_50_ff2 <= 25'd0;
    end else begin
        keyboard_50_ff1 <= e6_keys_down;
        keyboard_50_ff2 <= keyboard_50_ff1;
    end
end

// 标准 Note Event：后续 25 键矩阵只需要替换事件产生端，声音链不用改。
reg         note_evt_toggle;
reg         note_evt_on;
reg  [6:0]  note_evt_note;
reg  [6:0]  note_evt_velocity;
reg         piano_active_prev;
reg  [6:0]  piano_note_prev;

// 动态节拍器：当前只用LED验证节拍速度
wire        metronome_beat_tick;
wire        metronome_strong_tick;
wire        metronome_weak_tick;
wire [4:0]  metronome_current_beat;
wire        metronome_flash;

reg         metronome_beat_toggle;    // 保留通用节拍事件，后续OSD可继续用
reg         metronome_strong_toggle;  // 第1拍事件
reg         metronome_weak_toggle;    // 非第1拍事件
reg         key_click_toggle;         // KEY1/KEY2/KEY4 任意一次有效按下事件

reg  [1:0]  sw1_sync_clk;
reg  [1:0]  sw1_sync_sd;
wire        image_mode_clk;
wire        image_mode_sd;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

// V15D: lock display-buffer switching to video frame boundaries.
// Only buffer0/1 are used, so synchronize the one actual select bit.
reg         disp_buf_bit_sync1_v;
reg         disp_buf_bit_sync2_v;
reg [1:0]   disp_buf_idx_frame;
reg         video_read_req_d;

// V15F: return the ACTUAL video-visible buffer ownership back to sd_card_clk.
// This closes the double-buffer handshake: SD is not allowed to reuse a buffer
// until the video domain has really moved away from it.
reg         video_buf_bit_sync1_sd;
reg         video_buf_bit_sync2_sd;
wire [1:0]  video_disp_buf_idx_sd = {1'b0, video_buf_bit_sync2_sd};

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;
// E9.3.24 OWNER-GUARD: use the same video outstanding-return status
// at both write-client admission gates. Bursts already in progress run
// uninterrupted. A video return never changes to a PCM owner halfway.
// The existing OR/mux command path is otherwise intentionally unchanged.
// The original framebuffer controller owns *video* requests; E9.3 uses
// its own FIFO and a strictly exclusive SDRAM port for audio reads/writes.
wire video_App_rd_en;
wire [ADDR_BITS-1:0] video_App_rd_addr;
wire [20:0] e962_zoom_video_App_rd_addr;
wire video_Sdr_rd_en;
wire video_App_wr_en;
wire [ADDR_BITS-1:0] video_App_wr_addr;
wire [MEM_DATA_BITS-1:0] video_App_wr_din;
wire [3:0] video_App_wr_dm;
wire pcm_App_wr_en;
wire [20:0] pcm_App_wr_addr;
wire [31:0] pcm_App_wr_din;
wire pcm_writer_busy;
wire video_memory_priority;
// E9.3.24: returned by the PCM reader's EXISTING command/return scoreboard.
// Prevent a new write burst while older video reads have not all returned.
wire video_returns_pending;
wire [9:0] video_fifo_words_mem; // G: mem_clk-domain framebuffer FIFO occupancy
wire video_fifo_underflow_v;     // H: video_clk-domain live frame underflow
wire pcm_read_lock;
wire pcm_App_rd_en;
wire [20:0] pcm_App_rd_addr;
assign App_rd_en = pcm_App_rd_en | video_App_rd_en;
// E962: ONE existing framebuffer reader.  When 2x/4x is selected,
// map the FIFO's sequential read commands onto cropped SDRAM rows.
// PCM still has the original, highest-priority address mux.
assign App_rd_addr = pcm_App_rd_en ? pcm_App_rd_addr :
                     e962_zoom_video_App_rd_addr;
// ===============================================================
// E962: full-screen video zoom SDRAM address mapping (MEM clock).
// Original frame_fifo_read stays untouched: 128-word read bursts,
// refresh/video priority, PCM ownership and SD writer ALL unchanged.
// Zoom mode simply changes the video addresses; the video read FIFO
// consumes only 320/160 source words per every 2/4 destination rows.
// ===============================================================
reg [1:0] e962_mem_level_ff1, e962_mem_level_ff2;
reg [18:0] e962_mem_offset_ff1, e962_mem_offset_ff2;
reg [20:0] e962_mem_buf1_first_addr;
reg [20:0] e962_mem_next_addr;
reg [9:0] e962_mem_source_col;
reg [8:0] e962_mem_source_row;
wire e962_mem_last_source_row =
    (e962_mem_level_ff2==2'd2) ?
        (e962_mem_source_row==9'd119) :
        (e962_mem_source_row==9'd239);
wire [9:0] e962_mem_source_width =
    (e962_mem_level_ff2==2'd2) ? 10'd160 : 10'd320;
wire [20:0] e962_mem_row_jump =
    (e962_mem_level_ff2==2'd2) ? 21'd481 : 21'd321;
wire e962_mem_first_buf0 = (video_App_rd_addr==21'd0);
wire e962_mem_first_buf1 = (video_App_rd_addr==21'd307200);
wire e962_mem_first = e962_mem_first_buf0 || e962_mem_first_buf1;
wire [20:0] e962_mem_first_source_addr =
    e962_mem_first_buf1 ? e962_mem_buf1_first_addr :
                           {2'd0,e962_mem_offset_ff2};
assign e962_zoom_video_App_rd_addr =
    (e962_mem_level_ff2==2'd0) ? video_App_rd_addr :
    e962_mem_first ? e962_mem_first_source_addr : e962_mem_next_addr;
always @(posedge ext_mem_clk or posedge rst_tf) begin
    if (rst_tf) begin
        e962_mem_level_ff1<=2'd0;
        e962_mem_level_ff2<=2'd0;
        e962_mem_offset_ff1<=19'd0;
        e962_mem_offset_ff2<=19'd0;
        e962_mem_buf1_first_addr<=21'd307200;
        e962_mem_next_addr<=21'd0;
        e962_mem_source_col<=10'd0;
        e962_mem_source_row<=9'd0;
    end else begin
        e962_mem_level_ff1<=e962_zoom_level_frame;
        e962_mem_level_ff2<=e962_mem_level_ff1;
        e962_mem_offset_ff1<=e962_zoom_frame_offset;
        e962_mem_offset_ff2<=e962_mem_offset_ff1;
        e962_mem_buf1_first_addr<=21'd307200 +
                                   {2'd0,e962_mem_offset_ff2};
        if(video_App_rd_en) begin
            if(e962_mem_first) begin
                // First word of each 1x/2x/4x frame starts at the
                // exact original SDRAM display-buffer base + ROI.
                e962_mem_next_addr <= e962_mem_first_source_addr + 21'd1;
                e962_mem_source_col <= 10'd1;
                e962_mem_source_row <= 9'd0;
            end else if(e962_mem_source_col ==
                          e962_mem_source_width-10'd1) begin
                // Last source pixel on this displayed row.  Skip the
                // unneeded remainder of the 640-word SDRAM scanline.
                // Current last +321 (2x) or +481 (4x).
                if(!e962_mem_last_source_row) begin
                    e962_mem_next_addr <=
                        e962_mem_next_addr + e962_mem_row_jump;
                    e962_mem_source_col <= 10'd0;
                    e962_mem_source_row <=
                        e962_mem_source_row + 9'd1;
                end
                // The legacy FIFO may prefetch at most ~one burst beyond
                // the visible crop; repeat the final legal pixel instead
                // of reading into the OTHER display buffer / audio area.
            end else begin
                e962_mem_next_addr <= e962_mem_next_addr + 21'd1;
                e962_mem_source_col <= e962_mem_source_col + 10'd1;
            end
        end
    end
end

assign App_wr_en = pcm_App_wr_en | video_App_wr_en;
assign App_wr_addr = pcm_App_wr_en ? pcm_App_wr_addr : video_App_wr_addr;
assign App_wr_din = pcm_App_wr_en ? pcm_App_wr_din : video_App_wr_din;
assign App_wr_dm = pcm_App_wr_en ? 4'b0000 : video_App_wr_dm;

// E9.3.13 J: SAMPLE THE ACTUAL SDRAM APPLICATION-PORT INPUTS.
// All counters and snapshots are passive. They never drive the SDRAM interface.
reg        e943_first_pcm_wr_seen_mem;
reg [31:0] e943_first_pcm_wr_data_mem;
reg [20:0] e943_first_pcm_wr_addr_mem;
reg [20:0] e943_pcm_wr_count_mem;
reg        e943_pcm_video_collision_mem;
wire e943_conflicting_commands =
    (pcm_App_wr_en && (video_App_wr_en || video_App_rd_en)) ||
    (pcm_App_rd_en && (video_App_wr_en || video_App_rd_en));
always @(posedge ext_mem_clk or posedge rst_tf) begin
    if (rst_tf) begin
        e943_first_pcm_wr_seen_mem <= 1'b0;
        e943_first_pcm_wr_data_mem <= 32'd0;
        e943_first_pcm_wr_addr_mem <= 21'd0;
        e943_pcm_wr_count_mem <= 21'd0;
        e943_pcm_video_collision_mem <= 1'b0;
    end else begin
        if (e943_conflicting_commands)
            e943_pcm_video_collision_mem <= 1'b1;
        if (pcm_App_wr_en) begin
            if (e943_pcm_wr_count_mem != 21'h1fffff)
                e943_pcm_wr_count_mem <= e943_pcm_wr_count_mem + 21'd1;
            if (!e943_first_pcm_wr_seen_mem) begin
                // Captured at the same rising edge that the SDRAM IP sees.
                e943_first_pcm_wr_seen_mem <= 1'b1;
                e943_first_pcm_wr_data_mem <= App_wr_din;
                e943_first_pcm_wr_addr_mem <= App_wr_addr;
            end
        end
    end
end

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n | ~audio_pll_lock;

// ===================== P5.0 音乐播放状态和模式 =====================
// SW1只负责音乐播放/暂停；SW4独占图片/编辑器模式切换。
// SW1同步到50MHz音频节拍/数码管域
always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        sw1_sync_clk <= 2'b11;
    else
        sw1_sync_clk <= {sw1_sync_clk[0], sw1};
end
assign image_mode_clk = !editor_mode_clk; // SW4=0 图片，SW1不再决定画面

// SW3 同步到 50MHz 控制域：1=临时琴键测试模式，同时开启底部 OSD。
always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        sw3_sync_clk <= 2'b00;
    else
        sw3_sync_clk <= {sw3_sync_clk[0], sw3};
end
assign piano_mode_clk = sw3_sync_clk[1];

// SW4 同步到50MHz控制域：1=编辑器模式。
// 编辑器优先级高于图片控制、节拍器参数设置和临时电子琴。
always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        sw4_sync_clk <= 2'b00;
    else
        sw4_sync_clk <= {sw4_sync_clk[0], sw4};
end
assign editor_mode_clk = sw4_sync_clk[1];

// Editor keyboard controls are handled by editor_e6_integrated below.

// E8.1: send EC11 tempo increments safely from 25MHz video to 50MHz control.
wire e81_tempo_up, e81_tempo_down;
wire e81_strong_toggle, e81_weak_toggle;
reg [2:0] tempo_up_cdc, tempo_down_cdc;
reg [2:0] e81_strong_cdc, e81_weak_cdc;
wire bpm_encoder_up = tempo_up_cdc[1] && !tempo_up_cdc[2];
wire bpm_encoder_down = tempo_down_cdc[1] && !tempo_down_cdc[2];
wire e81_strong_event = e81_strong_cdc[2] ^ e81_strong_cdc[1];
wire e81_weak_event = e81_weak_cdc[2] ^ e81_weak_cdc[1];
always @(posedge clk or posedge rst_all) begin
    if(rst_all) begin
        tempo_up_cdc <= 3'd0;
        tempo_down_cdc <= 3'd0;
        e81_strong_cdc <= 3'd0;
        e81_weak_cdc <= 3'd0;
    end else begin
        tempo_up_cdc <= {tempo_up_cdc[1:0],e81_tempo_up};
        tempo_down_cdc <= {tempo_down_cdc[1:0],e81_tempo_down};
        e81_strong_cdc <= {e81_strong_cdc[1:0],e81_strong_toggle};
        e81_weak_cdc <= {e81_weak_cdc[1:0],e81_weak_toggle};
    end
end

// P5.0 BPM在全屏编辑器通过EC11直接调节，40..240。
 // 不再让板载KEY1/KEY2在SW1=0时误改变BPM，节拍数保持4/4。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        metronome_bpm <= 10'd120;
        beats_per_bar <= 5'd4;
        bpm_hundreds  <= 4'd1;
        bpm_tens      <= 4'd2;
        bpm_ones      <= 4'd0;
    end
    else if (editor_mode_clk) begin
        if (bpm_encoder_up && metronome_bpm < 10'd240) begin
            metronome_bpm <= metronome_bpm + 10'd1;
            if (bpm_ones == 4'd9) begin
                bpm_ones <= 4'd0;
                if (bpm_tens == 4'd9) begin
                    bpm_tens     <= 4'd0;
                    bpm_hundreds <= bpm_hundreds + 4'd1;
                end
                else begin
                    bpm_tens <= bpm_tens + 4'd1;
                end
            end
            else begin
                bpm_ones <= bpm_ones + 4'd1;
            end
        end
        else if (bpm_encoder_down && metronome_bpm > 10'd40) begin
            metronome_bpm <= metronome_bpm - 10'd1;
            if (bpm_ones == 4'd0) begin
                bpm_ones <= 4'd9;
                if (bpm_tens == 4'd0) begin
                    bpm_tens     <= 4'd9;
                    bpm_hundreds <= bpm_hundreds - 4'd1;
                end
                else begin
                    bpm_tens <= bpm_tens - 4'd1;
                end
            end
            else begin
                bpm_ones <= bpm_ones - 4'd1;
            end
        end

    end
end

// 三个设置按键在 50MHz 域独立消抖；按键均为按下低电平
key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) u_key1_setting (
    .clk          (clk),
    .rst          (rst_all),
    .button_in    (key1),
    .press_pulse  (key1_setting_press),
    .button_state (key1_button_state)
);

key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) u_key2_setting (
    .clk          (clk),
    .rst          (rst_all),
    .button_in    (key2),
    .press_pulse  (key2_setting_press),
    .button_state (key2_button_state)
);

key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) u_key4_setting (
    .clk          (clk),
    .rst          (rst_all),
    .button_in    (key4),
    .press_pulse  (key4_setting_press),
    .button_state (key4_button_state)
);

// ===================== SW3 琴键测试模式 -> Note Event =====================
// 按键为低有效，使用消抖后的持续状态。当前保持单音优先级：KEY1 > KEY2 > KEY4。
// MIDI note number: C4=60, D4=62, E4=64。
// 三个独立音频 gate：SW3=1 时才允许发声；按键状态为低有效。
// SW3=1: physical FPGA KEYs and PCB matrix C4/D4/E4 share the established
// sample/DDS path.  Other 22 pitches continue through the lightweight mixer.
assign piano_c4_gate = piano_mode_clk && !editor_mode_clk &&
                       ((!key1_button_state) || keyboard_50_ff2[0]);
assign piano_d4_gate = piano_mode_clk && !editor_mode_clk &&
                       ((!key2_button_state) || keyboard_50_ff2[2]);
assign piano_e4_gate = piano_mode_clk && !editor_mode_clk &&
                       ((!key4_button_state) || keyboard_50_ff2[4]);

// 下面这一套单通道 Note Event 暂时只供 OSD/历史轨迹使用。
// 因此同时按多个键时，画面事件仍按 C4 > D4 > E4 优先级；
// 本版本只验证“声音端”真正的3音复音。
assign piano_key_active =
    piano_c4_gate || piano_d4_gate || piano_e4_gate;

assign piano_selected_note =
    piano_c4_gate ? 7'd60 :
    piano_d4_gate ? 7'd62 :
    piano_e4_gate ? 7'd64 :
                    7'd60;

// 只有“是否有键按下”或“当前选中的音高”发生变化时才产生一次事件。
// SW3 从 1 切回 0 时，piano_key_active 会变 0，因此会自动产生 NOTE_OFF，避免卡音。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        note_evt_toggle     <= 1'b0;
        note_evt_on         <= 1'b0;
        note_evt_note       <= 7'd60;
        note_evt_velocity   <= 7'd0;
        piano_active_prev   <= 1'b0;
        piano_note_prev     <= 7'd60;
    end
    else begin
        if ((piano_key_active != piano_active_prev) ||
            (piano_key_active && (piano_selected_note != piano_note_prev))) begin

            note_evt_toggle <= ~note_evt_toggle;

            if (piano_key_active) begin
                note_evt_on       <= 1'b1;
                note_evt_note     <= piano_selected_note;
                note_evt_velocity <= 7'd100;
                piano_note_prev   <= piano_selected_note;
            end
            else begin
                note_evt_on       <= 1'b0;
                note_evt_note     <= piano_note_prev;
                note_evt_velocity <= 7'd0;
            end

            piano_active_prev <= piano_key_active;
        end
    end
end

// P5.0: SW1=音乐时钟播放, SW2=节拍器发声开关；LED和发声一致。
 // 全屏编辑器的节拍事件仍由同一个音轨时钟生成，音画同步。
metronome_bpm_led #(
    .CLK_FREQ_HZ (50_000_000),
    .FLASH_MS    (50)
) u_metronome_bpm_led (
    .clk          (clk),
    .rst          (rst_all),
    .enable       (sw1_sync_clk[1] && sw2 && !editor_mode_clk),
    .bpm          (metronome_bpm),
    .beats_per_bar(beats_per_bar),
    .beat_tick    (metronome_beat_tick),
    .strong_tick  (metronome_strong_tick),
    .weak_tick    (metronome_weak_tick),
    .current_beat (metronome_current_beat),
    .flash        (metronome_flash)
);

// E8.1 editor: metronome flash uses the very same tick as recording.
reg [21:0] e81_led_flash_count;
always @(posedge clk or posedge rst_all) begin
    if(rst_all) e81_led_flash_count <= 22'd0;
    else if(editor_mode_clk && (e81_strong_event || e81_weak_event))
        e81_led_flash_count <= 22'd2500000; // 50 ms at 50 MHz
    else if(e81_led_flash_count!=22'd0)
        e81_led_flash_count <= e81_led_flash_count-22'd1;
end
// Global metronome audio indicator follows SW2 in BOTH screens.
wire e952_metro_audible_50 =
    sw1_sync_clk[1] && e950_metro_on50;
assign led = (e952_metro_audible_50 &&
              (editor_mode_clk ? (e81_led_flash_count!=22'd0) :
               metronome_flash)) ? 4'b1111 : 4'b0000;

// 将50MHz域的节拍事件编码成toggle，供12.288MHz音频域可靠同步。
// strong/weak分别使用独立toggle，避免跨域时“事件”和“类型标志”不同步。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        metronome_beat_toggle    <= 1'b0;
        metronome_strong_toggle  <= 1'b0;
        metronome_weak_toggle    <= 1'b0;
        key_click_toggle         <= 1'b0;
    end
    else begin
        if (metronome_beat_tick)
            metronome_beat_toggle <= ~metronome_beat_toggle;

        if (e952_metro_audible_50 &&
            ((editor_mode_clk && e81_strong_event) ||
             (!editor_mode_clk && metronome_strong_tick)))
            metronome_strong_toggle <= ~metronome_strong_toggle;

        if (e952_metro_audible_50 &&
            ((editor_mode_clk && e81_weak_event) ||
             (!editor_mode_clk && metronome_weak_tick)))
            metronome_weak_toggle <= ~metronome_weak_toggle;

        // SW3=0 时保持原来的 UI click；SW3=1 时三个按键作为琴键，不再叠加 click。
        if (!piano_mode_clk && !editor_mode_clk &&
            (key1_setting_press || key2_setting_press || key4_setting_press))
            key_click_toggle <= ~key_click_toggle;
    end
end

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// SW1 同步到 sd_card_clk 域，用于屏蔽/恢复原图片按键功能
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        sw1_sync_sd <= 2'b11;
        sw3_sync_sd <= 2'b00;
        sw4_sync_sd <= 2'b00;
    end
    else begin
        sw1_sync_sd <= {sw1_sync_sd[0], sw1};
        sw3_sync_sd <= {sw3_sync_sd[0], sw3};
        sw4_sync_sd <= {sw4_sync_sd[0], sw4};
    end
end
assign image_mode_sd  = !editor_mode_sd; // 图片TF换页与SW1音乐播放解耦
assign piano_mode_sd  = sw3_sync_sd[1];
assign editor_mode_sd = sw4_sync_sd[1];

// E8: TF/SDRAM initializes from power-on and runs even with SW4=1.
// Do NOT pulse TF reset on the first transition out of the editor:
// that breaks the producer/consumer framebuffer handshake on cold boots.
wire rst_tf = rst_all;

// D read-only counters. Count requests emitted by VIDEO write FSM only.
// App_wr_en is a REQUEST to SDRAM, not a guarantee of actual data acceptance.
// A 3-stage sync brings the TF 'load_busy' window to ext_mem_clk.
reg [2:0]  e937_load_mem_sync;
reg [18:0] e937_mem_write_count;
reg        e937_mem_write_any;
wire       e937_mem_write_full = (e937_mem_write_count == 19'd307200);

always @(posedge ext_mem_clk or posedge rst_tf) begin
    if (rst_tf) begin
        e937_load_mem_sync   <= 3'b000;
        e937_mem_write_count <= 19'd0;
        e937_mem_write_any   <= 1'b0;
    end else begin
        e937_load_mem_sync <= {e937_load_mem_sync[1:0], e933_img_status_sd[3]};
        if (!e937_load_mem_sync[2]) begin
            e937_mem_write_count <= 19'd0;
            e937_mem_write_any   <= 1'b0;
        end else if (video_App_wr_en) begin
            e937_mem_write_any <= 1'b1;
            if (e937_mem_write_count < 19'd307200)
                e937_mem_write_count <= e937_mem_write_count + 19'd1;
        end
    end
end

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        e937_mem_full_video_sync <= 2'b00;
        e937_mem_any_video_sync  <= 2'b00;
    end else begin
        e937_mem_full_video_sync <= {e937_mem_full_video_sync[0], e937_mem_write_full};
        e937_mem_any_video_sync  <= {e937_mem_any_video_sync[0], e937_mem_write_any};
    end
end

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_tf) begin
    if (rst_tf)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
// P3.7 only: expose SD status to the existing image-digit LED.
wire e949_sd_ready;
wire [3:0] e949_fat_error, e949_fat_stage;
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    .FAT32_MODE        (1'b1) // set 0 only to restore raw-IMG loader
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_tf),
    // Board K1/K2 control the mini cursor/track ONLY when its panel is
    // open. When hidden they retain normal picture next/auto behavior.
    .key_next          ((image_mode_sd && !piano_mode_sd && !editor_mode_sd &&
                         !e952_mini_keys_active_sd[1]) ? key1 : 1'b1),
    .key_auto          ((image_mode_sd && !piano_mode_sd && !editor_mode_sd &&
                         !e952_mini_keys_active_sd[1]) ? key2 : 1'b1),
    .state_code        (state_code),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),
    .e933_img_status (e933_img_status_sd),
    .dbg_sd_init_done  (e949_sd_ready),
    .dbg_fat_error     (e949_fat_error),
    .dbg_fat_stage     (e949_fat_stage),
    .piano_bank_found  (piano_bank_found_sd),
    .piano_c4_pcm_wr_en   (piano_c4_pcm_wr_en_sd),
    .piano_c4_pcm_wr_addr (piano_c4_pcm_wr_addr_sd),
    .piano_c4_pcm_wr_data (piano_c4_pcm_wr_data_sd),
    .piano_c4_pcm_ready   (piano_c4_pcm_ready_sd),
    .e93_bank_start (e93_bank_start_sd),
    .e93_bank_words (e93_bank_words_sd),
    .e93_pcm_word_en (e93_word_en_sd),
    .e93_pcm_word (e93_pcm_word_sd),
    .e93_scan_complete (e93_scan_complete_sd),
    .piano_dma_ready_sd (e964_piano_ready_sd),
    .violin_dma_ready_sd (e964_violin_ready_sd),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),
    .video_disp_buf_idx(video_disp_buf_idx_sd),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

// 节拍数只有1~16，十位/个位用简单比较与减法生成；BPM三位直接由BCD寄存器保存
assign beat_tens    = (beats_per_bar >= 5'd10) ? 4'd1 : 4'd0;
assign beat_ones    = (beats_per_bar >= 5'd10) ? (beats_per_bar - 5'd10) : beats_per_bar[3:0];

// E949 P3.7 TEMPORARY TF fault locator: no new ROM, BRAM, PLL or debug UART.
// Instead of the original image number, the EXISTING image 7-seg digit
// displays one hex status nibble selected with SW3 and SW1.
// SW3=0 SW1=0: {SD_INIT, SCAN_DONE, IMAGE_FOUND, IMAGE_DISPLAYED}
// SW3=0 SW1=1: {PIANO_FOUND, FIRST_PCM_WORD, C4_4096_READY, PCM_BANK_LOADED}
// SW3=1 SW1=0: last FAT32/BMP error nibble (0=none)
// SW3=1 SW1=1: FAT32 loader phase (0 init,1 bank,2 scan,3 probe,4 image)
// NOTE: sw1/sw3 retain their NORMAL mode behavior. Avoid playing while
// changing them; use only to inspect LEDs after initial load attempt.
// TF/PCM are now confirmed stable on the user's board.
// Restore the original picture number on the 7-segment display and
// let synthesis prune the old four-page debug-only CDC registers.
// Change back to 1'b1 only for future hardware diagnostics.
localparam E949_SHOW_TF_DIAG_ON_IMAGE_DIGIT = 1'b0;

wire [3:0] e949_image_flags_sd =
    {e949_sd_ready, e933_img_status_sd[7],
     (|e933_img_status_sd[6:4]), e933_img_status_sd[8]};
wire [3:0] e949_pcm_flags_sd =
    {piano_bank_found_sd, e941_tf_first_valid_sd,
     e926_c4_ready_sd, e93_bank_loaded_mem};
reg [3:0] e949_img_meta,e949_img_sync;
reg [3:0] e949_pcm_meta,e949_pcm_sync;
reg [3:0] e949_err_meta,e949_err_sync;
reg [3:0] e949_stage_meta,e949_stage_sync;
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        e949_img_meta <= 4'd0; e949_img_sync <= 4'd0;
        e949_pcm_meta <= 4'd0; e949_pcm_sync <= 4'd0;
        e949_err_meta <= 4'd0; e949_err_sync <= 4'd0;
        e949_stage_meta <= 4'd0; e949_stage_sync <= 4'd0;
    end else begin
        e949_img_meta <= e949_image_flags_sd;
        e949_img_sync <= e949_img_meta;
        e949_pcm_meta <= e949_pcm_flags_sd;
        e949_pcm_sync <= e949_pcm_meta;
        e949_err_meta <= e949_fat_error;
        e949_err_sync <= e949_err_meta;
        e949_stage_meta <= e949_fat_stage;
        e949_stage_sync <= e949_stage_meta;
    end
end
wire [3:0] e949_diag_hex =
      (!sw3 && !sw1) ? e949_img_sync :
      (!sw3 &&  sw1) ? e949_pcm_sync :
      ( sw3 && !sw1) ? e949_err_sync :
                        e949_stage_sync;

seg_decoder seg_decoder_image(
    .bin_data          (E949_SHOW_TF_DIAG_ON_IMAGE_DIGIT ?
                        e949_diag_hex : state_code),
    .seg_data          (seg_image)
);

seg_decoder seg_decoder_bpm_hundreds(
    .bin_data          (bpm_hundreds),
    .seg_data          (seg_bpm_hundreds)
);

seg_decoder seg_decoder_bpm_tens(
    .bin_data          (bpm_tens),
    .seg_data          (seg_bpm_tens)
);

seg_decoder seg_decoder_bpm_ones(
    .bin_data          (bpm_ones),
    .seg_data          (seg_bpm_ones)
);

seg_decoder seg_decoder_beat_tens(
    .bin_data          (beat_tens),
    .seg_data          (seg_beat_tens)
);

seg_decoder seg_decoder_beat_ones(
    .bin_data          (beat_ones),
    .seg_data          (seg_beat_ones)
);

// 本工程使用右侧6位数码管：
// SW1=1：保持原显示，只在最右一位显示当前图片编号1~4
// SW1=0：从右到左为 BPM个位/十位/百位、节拍个位/十位，最左一位留空
seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        (8'hff),
    .seg_data_1        (editor_mode_clk ? 8'hff :
                        (image_mode_clk ? 8'hff : {1'b1,seg_beat_tens})),
    .seg_data_2        (editor_mode_clk ? 8'hff :
                        (image_mode_clk ? 8'hff : {1'b1,seg_beat_ones})),
    .seg_data_3        (editor_mode_clk ? 8'hff :
                        (image_mode_clk ? 8'hff : {1'b1,seg_bpm_hundreds})),
    .seg_data_4        (editor_mode_clk ? 8'hff :
                        (image_mode_clk ? 8'hff : {1'b1,seg_bpm_tens})),
    .seg_data_5        (editor_mode_clk ? 8'hff :
                        (image_mode_clk ? {1'b1,seg_image} : {1'b1,seg_bpm_ones}))
);

// ===================== 原图像时序与帧缓存 =====================
// sd_card_clk may commit a completed back buffer at any moment.
// The video side only accepts that change at the beginning of a new frame.
//
// If a new image becomes ready too close to a frame boundary, it is safer to
// wait one complete frame than to let the last/first scan line use another
// buffer. This eliminates one-line-late / mixed-frame artifacts on fast flips.
always @(posedge video_clk or posedge rst_tf) begin
    if (rst_tf) begin
        disp_buf_bit_sync1_v <= 1'b0;
        disp_buf_bit_sync2_v <= 1'b0;
        disp_buf_idx_frame   <= 2'd0;
        video_read_req_d     <= 1'b0;
    end
    else begin
        disp_buf_bit_sync1_v <= disp_buf_idx[0];
        disp_buf_bit_sync2_v <= disp_buf_bit_sync1_v;
        video_read_req_d     <= video_read_req;

        // video_read_req rises once for each new frame.
        if (video_read_req && !video_read_req_d)
            disp_buf_idx_frame <= {1'b0, disp_buf_bit_sync2_v};
    end
end

// Return the frame-latched (actually displayed) buffer to the SD-control domain.
// Only bit0 is needed because the project currently uses exactly two buffers.
always @(posedge sd_card_clk or posedge rst_tf) begin
    if (rst_tf) begin
        video_buf_bit_sync1_sd <= 1'b0;
        video_buf_bit_sync2_sd <= 1'b0;
    end
    else begin
        video_buf_bit_sync1_sd <= disp_buf_idx_frame[0];
        video_buf_bit_sync2_sd <= video_buf_bit_sync1_sd;
    end
end

video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_tf),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

// E962 full-screen 1x/2x/4x replacement for the original video_delay.
// 1x is the same timing/read pattern as the previously stable B bitstream.
e962_full_screen_zoom_video video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_tf),
    .zoom_level        (e962_zoom_level_frame),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data_base = display_valid ? vout_data_raw : 24'd0;

// E951 / P3.9: picture mode now uses ONE image-linked, real-track
// mini Piano Roll, rendered downstream by editor_loop_e8.
// Disable the obsolete secondary 256-frame live-keyboard score OSD
// AND its 24px ASCII BPM/TF strip to recover physical MSlices.
// The original image pipeline/TF PCM loader is unchanged.
localparam E951_KEEP_LEGACY_IMAGE_STRIP = 1'b0;
generate if (E951_KEEP_LEGACY_IMAGE_STRIP) begin : G_E951_LEGACY_IMAGE_OSD
    music_score_osd #(.E950_KEEP_LIVE_KEY_HISTORY(1'b0))
    u_music_score_osd (
        .video_clk(video_clk), .rst(rst_all),
        .enable(!sw4), .running(sw2 && !sw4),
        .show_status(sw3),
        .vs(vs), .de(de), .base_rgb(vout_data_base),
        .keys_25(e6_keys_down), .image_ready(display_valid),
        .bpm_hundreds(bpm_hundreds), .bpm_tens(bpm_tens), .bpm_ones(bpm_ones),
        .beats_per_bar(beats_per_bar),
        .current_beat(metronome_current_beat),
        .c4_gate(piano_c4_gate),
        .d4_gate(piano_d4_gate), .e4_gate(piano_e4_gate),
        .piano_bank_found(piano_bank_found_sd),
        .out_rgb(vout_data_score)
    );
end else begin : G_E951_DIRECT_IMAGE
    assign vout_data_score = vout_data_base;
end endgenerate

// E9.3.1 diagnostic only: observe real pipeline stages, NOT the old SBNK 'P'.
// Capturing the one-sd-clock E93F match into a sticky bit is required because
// video_clk cannot reliably sample a 10 ns pulse from sd_card_clk.
reg e931_e93_header_seen_sd;
always @(posedge sd_card_clk or posedge rst_tf) begin
    if (rst_tf) e931_e93_header_seen_sd <= 1'b0;
    else if (e93_bank_start_sd) e931_e93_header_seen_sd <= 1'b1;
end
reg [2:0] e931_p_ff, e931_h_ff, e931_d_ff, e931_f_ff;
reg [2:0] e931_rsp_ff;
reg e931_rsp_previous;
reg e931_req_previous;
reg e931_any_key_seen, e931_any_gate_seen;
reg e931_request_seen, e931_response_seen, e931_nonzero_seen;
reg e931_underrun_seen;
// Static visual snapshots: LEFT=TF source first word, RIGHT=SDRAM C4 response.
// A failed read takes precedence over a previous good snapshot.
reg [2:0] e941_tf_first_valid_sync_v;
reg [31:0] e941_tf_first_word_v;
reg e941_tf_first_ready_v;
reg [31:0] e941_c4_read_word_v;
reg e941_c4_read_seen_v;
reg e941_c4_bad_snapshot_v;
// J read-only video domain snapshots.
reg [2:0] e943_first_wr_sync_v;
reg [2:0] e943_collision_sync_v;
reg [31:0] e943_first_wr_data_v;
reg [31:0] e943_first_wr_addr_hex_v;
reg [31:0] e943_wr_count_hex_v;
reg e943_first_wr_ready_v;
reg e943_wr_count_ready_v;
reg [31:0] e944_diag_status_sync1, e944_diag_status_v;
reg [31:0] e944_diag_expected_sync1, e944_diag_expected_v;
reg [31:0] e944_diag_observed_sync1, e944_diag_observed_v;
reg [20:0] e944_diag_addr_sync1, e944_diag_addr_v;
wire [10:0] e931_debug_flags = {
    e931_underrun_seen,   // U: player missed a PCM deadline
    e931_f_ff[2],         // F: SD->SDRAM FIFO overflow
    e931_nonzero_seen,    // A: nonzero PCM reached the audio mixer
    e931_response_seen,   // R: SDRAM sent an audio response
    e931_request_seen,    // Q: voice requested a memory word
    e931_any_gate_seen,   // G: editor gain for a pitch was nonzero
    e931_any_key_seen,    // K: 25-key physical matrix detected a key
    e93_loaded_video,     // L: DMA says the entire bank is loaded
    e931_d_ff[2],         // D: SD parser captured all payload bytes
    e931_h_ff[2],         // H: exact E93F header matched
    e931_p_ff[2]          // P: legacy generic SBNK header detected
};
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        e931_p_ff <= 3'b0;
        e931_h_ff <= 3'b0;
        e931_d_ff <= 3'b0;
        e931_f_ff <= 3'b0;
        e931_rsp_ff <= 3'b0;
        e931_rsp_previous <= 1'b0;
        e931_req_previous <= 1'b0;
        e931_any_key_seen <= 1'b0;
        e931_any_gate_seen <= 1'b0;
        e931_request_seen <= 1'b0;
        e931_response_seen <= 1'b0;
        e931_nonzero_seen <= 1'b0;
        e931_underrun_seen <= 1'b0;
        e941_tf_first_valid_sync_v <= 3'b000;
        e941_tf_first_word_v <= 32'd0;
        e941_tf_first_ready_v <= 1'b0;
        e941_c4_read_word_v <= 32'd0;
        e941_c4_read_seen_v <= 1'b0;
        e941_c4_bad_snapshot_v <= 1'b0;
        e943_first_wr_sync_v <= 3'd0;
        e943_collision_sync_v <= 3'd0;
        e943_first_wr_data_v <= 32'd0;
        e943_first_wr_addr_hex_v <= 32'd0;
        e943_wr_count_hex_v <= 32'd0;
        e943_first_wr_ready_v <= 1'b0;
        e943_wr_count_ready_v <= 1'b0;
        e933_img_ff1 <= 11'd0;
        e933_img_ff2 <= 11'd0;
        e933_mem_nonzero_seen <= 1'b0;
        e933_c4_header_ok <= 1'b0;
        e933_c4_header_bad <= 1'b0;
    end else begin
        e943_first_wr_sync_v <=
            {e943_first_wr_sync_v[1:0], e943_first_pcm_wr_seen_mem};
        e943_collision_sync_v <=
            {e943_collision_sync_v[1:0], e943_pcm_video_collision_mem};
        if (e943_first_wr_sync_v[2]) begin
            e943_first_wr_data_v <= e943_first_pcm_wr_data_mem;
            e943_first_wr_addr_hex_v <=
                {11'd0, e943_first_pcm_wr_addr_mem};
            e943_first_wr_ready_v <= 1'b1;
        end
        if (e93_loaded_video) begin
            // The write total is stable when L is high; no tearing.
            e943_wr_count_hex_v <= {11'd0, e943_pcm_wr_count_mem};
            e943_wr_count_ready_v <= 1'b1;
        end
        e941_tf_first_valid_sync_v <=
            {e941_tf_first_valid_sync_v[1:0], e941_tf_first_valid_sd};
        if (e941_tf_first_valid_sync_v[2]) begin
            e941_tf_first_word_v <= e941_tf_first_word_sd;
            e941_tf_first_ready_v <= 1'b1;
        end
        e933_img_ff1 <= e933_img_status_sd;
        e933_img_ff2 <= e933_img_ff1;
        e931_p_ff <= {e931_p_ff[1:0], piano_bank_found_sd};
        e931_h_ff <= {e931_h_ff[1:0], e931_e93_header_seen_sd};
        e931_d_ff <= {e931_d_ff[1:0], e93_scan_complete_sd};
        e931_f_ff <= {e931_f_ff[1:0], e93_bank_fifo_overflow_sd};
        e931_rsp_ff <= {e931_rsp_ff[1:0],e93_mem_rsp_toggle};
        if (e931_rsp_ff[2] != e931_rsp_previous) begin
            e931_rsp_previous <= e931_rsp_ff[2];
            e931_response_seen <= 1'b1;
            if (e93_mem_rsp_word != 32'd0)
                e933_mem_nonzero_seen <= 1'b1;
            // Compare C4's very first 32-bit PCM payload word read from SDRAM.
            // The expected value comes from the supplied E93F source bank at
            // byte offset 512, not from simulated music/gain output.
            if (e93_mem_req_addr == 21'd655360) begin
                if (e93_mem_rsp_word == 32'hFFF6FFF3)
                    e933_c4_header_ok <= 1'b1;
                else
                    e933_c4_header_bad <= 1'b1;
                // Capture the first response, but replace it with the FIRST
                // failing response if later reads disagree with the bank.
                // Freeze that failure for a readable on-screen photograph.
                if (!e941_c4_read_seen_v ||
                    (!e941_c4_bad_snapshot_v &&
                     e93_mem_rsp_word != 32'hFFF6FFF3)) begin
                    e941_c4_read_word_v <= e93_mem_rsp_word;
                    e941_c4_read_seen_v <= 1'b1;
                    e941_c4_bad_snapshot_v <=
                        (e93_mem_rsp_word != 32'hFFF6FFF3);
                end
            end
        end
        if (e93_mem_req_toggle != e931_req_previous) begin
            e931_req_previous <= e93_mem_req_toggle;
            e931_request_seen <= 1'b1;
        end
        if (|e6_keys_down) e931_any_key_seen <= 1'b1;
        if (|e93_pitch_gain) e931_any_gate_seen <= 1'b1;
        if (e93_pcm_audio[23:8] != 16'd0) e931_nonzero_seen <= 1'b1;
        if (e93_pcm_underrun) e931_underrun_seen <= 1'b1;
    end
end

// Slow-changing diagnostics: two registers per bit into video_clk.
// Address/hash are latched on the FIRST failure and then remain stable;
// byte counters may briefly straddle an update and settle next frame.
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        e944_diag_status_sync1 <= 32'd0;
        e944_diag_status_v <= 32'd0;
        e944_diag_expected_sync1 <= 32'd0;
        e944_diag_expected_v <= 32'd0;
        e944_diag_observed_sync1 <= 32'd0;
        e944_diag_observed_v <= 32'd0;
        e944_diag_addr_sync1 <= 21'd0;
        e944_diag_addr_v <= 21'd0;
    end else begin
        e944_diag_status_sync1 <= e944_diag_status_mem;
        e944_diag_status_v <= e944_diag_status_sync1;
        e944_diag_expected_sync1 <= e944_diag_expected_mem;
        e944_diag_expected_v <= e944_diag_expected_sync1;
        e944_diag_observed_sync1 <= e944_diag_observed_mem;
        e944_diag_observed_v <= e944_diag_observed_sync1;
        e944_diag_addr_sync1 <= e944_diag_bad_addr_mem;
        e944_diag_addr_v <= e944_diag_addr_sync1;
    end
end

// PCB input sampler + step recorder, all in the 25 MHz video clock domain.
// Matrix stays scanning in LIVE mode (SW4=0), so it can sound immediately.
// Export high-impedance column pins from the TOP LEVEL for TD IO inference.
wire [4:0] e6_col_drive_low;
wire [4:0] e6_btn_pressed;
wire [4:0] e6_btn_down;
// E963 hardware correction: PCB matrix labels F5 and F#5 were
// electrically interchanged. Scanner retains original timing and
// hardware row/column order; swap only MIDI note indices 17 and 18.
wire [24:0] e963_raw_keys_down, e963_raw_keys_press;
assign e6_keys_down =
    {e963_raw_keys_down[24:19], e963_raw_keys_down[17],
     e963_raw_keys_down[18], e963_raw_keys_down[16:0]};
assign e6_keys_pressed =
    {e963_raw_keys_press[24:19], e963_raw_keys_press[17],
     e963_raw_keys_press[18], e963_raw_keys_press[16:0]};
wire e952_mini_keys_active_video;
wire [1:0] e962_zoom_level_ctrl;
wire [9:0] e962_zoom_x_ctrl;
wire [8:0] e962_zoom_y_ctrl;
reg [1:0] e962_zoom_level_frame;
reg [9:0] e962_zoom_x_frame;
reg [8:0] e962_zoom_y_frame;
reg e962_vs0_prev;
wire [18:0] e962_zoom_frame_offset =
    ({10'd0,e962_zoom_y_frame} << 9) +
    ({10'd0,e962_zoom_y_frame} << 7) +
    {9'd0,e962_zoom_x_frame};
always @(posedge video_clk or posedge rst_tf) begin
    if(rst_tf) begin
        e962_vs0_prev <= 1'b1;
        e962_zoom_level_frame <= 2'd0;
        e962_zoom_x_frame <= 10'd0;
        e962_zoom_y_frame <= 9'd0;
    end else begin
        e962_vs0_prev <= vs_0;
        // Same raw VS event starts the original 307200-word video read.
        // Video-side scaler + SDRAM address mapper keep an entire frame
        // coherent even while EC11 changes the requested view mid-frame.
        if(e962_vs0_prev && !vs_0) begin
            e962_zoom_level_frame <= e962_zoom_level_ctrl;
            e962_zoom_x_frame <= e962_zoom_x_ctrl;
            e962_zoom_y_frame <= e962_zoom_y_ctrl;
        end
    end
end
reg [1:0] e952_mini_keys_active_sd;
always @(posedge sd_card_clk or posedge rst_tf) begin
    if (rst_tf) e952_mini_keys_active_sd <= 2'b00;
    else e952_mini_keys_active_sd <=
         {e952_mini_keys_active_sd[0],e952_mini_keys_active_video};
end
// P5.0: SW2 is the single metronome SOUND switch, not a toggling
// state. Reuse the former video→50MHz synchronizer, no extra timer.
reg [1:0] e950_sw2_sync_video;
wire e950_metro_on_video = e950_sw2_sync_video[1];
reg [1:0] e950_metro_sync50;
wire e950_metro_on50 = e950_metro_sync50[1];
always @(posedge video_clk or posedge rst_all) begin
    if(rst_all) e950_sw2_sync_video <= 2'b00;
    else e950_sw2_sync_video <= {e950_sw2_sync_video[0],sw2};
end
always @(posedge clk or posedge rst_all) begin
    if(rst_all) e950_metro_sync50 <= 2'b00;
    else e950_metro_sync50 <=
         {e950_metro_sync50[0],e950_metro_on_video};
end
wire e6_enc_pressed, e6_enc_cw, e6_enc_ccw;
wire [23:0] audio_left_e6,audio_right_e6;
// P4.8: keep original HDMI output mix separate from the actual
// SPH0645 microphone PCM used by the new spectrum/waveform display.
wire [23:0] audio_left_to_hdmi;
assign kbd_col[0]=e6_col_drive_low[0] ? 1'b0 : 1'bz;
assign kbd_col[1]=e6_col_drive_low[1] ? 1'b0 : 1'bz;
assign kbd_col[2]=e6_col_drive_low[2] ? 1'b0 : 1'bz;
assign kbd_col[3]=e6_col_drive_low[3] ? 1'b0 : 1'bz;
assign kbd_col[4]=e6_col_drive_low[4] ? 1'b0 : 1'bz;
pcb_e6_inputs u_pcb_e6_inputs (
    .clk(video_clk), .rst(rst_all), .enable(1'b1),
    .btn_n({pcb_k5,pcb_k4,pcb_k3,pcb_k2,pcb_k1}),
    // E963: board-tested EC11 A/B order is reversed vs the old RTL.
    // Swap ONLY the two phase wires; CW now means physical clockwise
    // in zoom, BPM, track volume and all editor menus.
    .enc_sw_n(enc_sw), .enc_a_n(enc_b), .enc_b_n(enc_a),
    .row_n(kbd_row),.col_drive_low(e6_col_drive_low),
    .btn_pulse(e6_btn_pressed),.btn_down(e6_btn_down),.enc_press(e6_enc_pressed),
    .enc_cw(e6_enc_cw),.enc_ccw(e6_enc_ccw),
    .keys_down(e963_raw_keys_down),.key_press(e963_raw_keys_press)
);
editor_loop_e8 #(
    .KEEP_E93_DEBUG_UI(E935_KEEP_EDITOR_DEBUG_UI)
) u_editor_loop_e8 (
    .clk(video_clk), .rst(rst_all),
    .sw4(sw4), .sw1(sw1), .sw2(sw2), .sw3(sw3),
    .board_keys_down({!key4_button_state,
                       !key2_button_state,!key1_button_state}),
    .mini_keys_active(e952_mini_keys_active_video),
    .full_zoom_level(e962_zoom_level_ctrl),
    .full_zoom_origin_x(e962_zoom_x_ctrl),
    .full_zoom_origin_y(e962_zoom_y_ctrl),
    .bpm(metronome_bpm),
    .bpm_hundreds(bpm_hundreds),
    .bpm_tens(bpm_tens), .bpm_ones(bpm_ones),
    .buttons(e6_btn_pressed), .button_down(e6_btn_down),
    .metro_sound_enabled(e950_metro_on_video),
    .tempo_up(e81_tempo_up), .tempo_down(e81_tempo_down),
    .metro_strong_toggle(e81_strong_toggle),
    .metro_weak_toggle(e81_weak_toggle),
    .enc_button(e6_enc_pressed),
    .enc_cw(e6_enc_cw), .enc_ccw(e6_enc_ccw),
    .key_down(e6_keys_down),
    // Original FPGA keys C4/D4/E4 stay playable in SW3 mode.
    .old_keys({(piano_mode_clk && !key4_button_state),
               (piano_mode_clk && !key2_button_state),
               (piano_mode_clk && !key1_button_state)}),
    .vs(vs), .de(de), .video_in(vout_data_score), .video_out(vout_data),
    .audio_valid(audio_valid), .audio_left_in(audio_left_data),
    .audio_right_in(audio_right_data),
    .audio_left_out(audio_left_e6), .audio_right_out(audio_right_e6),
    .e93_pcm_loaded(e93_loaded_video),
    .e964_violin_loaded(e964_violin_ready_video),
    .e93_pcm_audio(e93_pcm_audio),
    .e982_mic_bands(e982_mic_spectrum_audio),
    .e982_mic_bands_toggle(e982_mic_spectrum_toggle),
    .e93_debug_flags(e931_debug_flags),
    .e932_pcm_live(e932_pcm_audible),
    .e932_hdmi_live(e932_hdmi_audible),
    .e933_view_status(e933_view_status),
    .e941_tf_word(e941_tf_first_word_v),
    .e941_tf_ready(e941_tf_first_ready_v),
    .e941_c4_read_word(e941_c4_read_word_v),
    .e941_c4_read_ready(e941_c4_read_seen_v),
    .e943_first_wr_word(e943_first_wr_data_v),
    .e943_first_wr_addr(e943_first_wr_addr_hex_v),
    .e943_first_wr_ready(e943_first_wr_ready_v),
    .e943_wr_count(e943_wr_count_hex_v),
    .e943_wr_count_ready(e943_wr_count_ready_v),
    .e943_collision(e943_collision_sync_v[2]),
    .e944_diag_status(e944_diag_status_v),
    .e944_diag_bad_addr({11'd0,e944_diag_addr_v}),
    .e944_diag_expected(e944_diag_expected_v),
    .e944_diag_observed(e944_diag_observed_v),
    .e93_pitch_gain(e93_pitch_gain),
    .note_evt_valid(e936_note_evt_valid),
    .note_evt_ready(e936_note_evt_ready),
    .note_evt_data(e936_note_evt_data),
    .track_mute_mask(e936_track_mute_mask),
    .mic_left_pin_state(e939_left_display),
    .mic_right_pin_state(e939_right_display),
    .mic_frame_blink(e940_blink_sync[1]),
    .mic_word_changed(e940_changed_sync[1]),
    .mic_voice_level(e940_level_display),
    .mic_pitch_code(e942_pitch_display),
    .mic_pitch_valid(e942_pitch_valid_sync[1])
);

// ========================================================================
// P2.3 MINIMUM I2S GPIO diagnosis. This is deliberately NOT a PCM receiver.
// BCLK/LRCL use existing audio_mclk; DOUT samples only the actual 24 bits.
// It consumes no 9K BRAM, no 32K BRAM and no DSP by design.
// ========================================================================
wire [1:0] e939_left_bits_audio, e939_right_bits_audio;
reg [1:0] e939_left_meta, e939_left_display;
reg [1:0] e939_right_meta, e939_right_display;
wire e940_frame_blink_audio;
wire e940_word_changed_audio;
wire [2:0] e940_voice_level_audio;
reg [1:0] e940_blink_sync, e940_changed_sync;
reg [2:0] e940_level_meta, e940_level_display;

// P3.0: stable, monophonic pitch candidate (C3..C6).
// Audio-side outputs update rarely. This stage is DISPLAY ONLY;
// editor Note RAM, Note Event, and track timing are untouched.
wire signed [11:0] e942_pcm12_audio;
wire e942_pcm_strobe_audio;
wire signed [11:0] e942_center_audio;
wire e942_gate_audio;
wire [5:0] e942_pitch_code_audio;
wire e942_pitch_valid_audio;
reg [5:0] e942_pitch_meta, e942_pitch_display;
reg [1:0] e942_pitch_valid_sync;

// P2.5: left-channel 21ms PCM peak-to-peak amplitude meter.
e939_mic_bit_probe #(
    .ENABLE_METER(1'b1),
    .LEVEL1_P2P(16'd120), // change to 240 if background noise still lights bars
    .LEVEL2_P2P(16'd300),
    .LEVEL3_P2P(16'd750),
    .LEVEL4_P2P(16'd1800)
) u_e939_mic_bit_probe (
    .audio_mclk(audio_mclk), .rst(rst_all),
    .dout(mic_i2s_dout),
    .mic_bclk(mic_i2s_bclk), .mic_lrcl(mic_i2s_lrcl),
    .left_state(e939_left_bits_audio),
    .right_state(e939_right_bits_audio),
    .frame_blink(e940_frame_blink_audio),
    .word_changed(e940_word_changed_audio),
    .voice_level(e940_voice_level_audio),
    .pcm12(e942_pcm12_audio),
    .pcm_strobe(e942_pcm_strobe_audio),
    .dc_center(e942_center_audio),
    .pitch_gate(e942_gate_audio)
);

// Independent P3.0 low-area monophonic pitch stage.
// Uses P2.5's already decoded signed 12-bit LEFT PCM; no sample BRAM.
// P3.2: replace zero-crossings with BRAM-buffered two-pass AMDF.
// Single shared absolute-difference engine; no FFT or DSP multiplier.
e942_mic_period_pitch u_e942_pitch (
    .clk(audio_mclk), .rst(rst_all),
    .pcm12(e942_pcm12_audio),
    .sample_valid(e942_pcm_strobe_audio),
    .center12(e942_center_audio),
    .energy_ok(e942_gate_audio),
    .pitch_code(e942_pitch_code_audio),
    .pitch_valid(e942_pitch_valid_audio)
);

// E982: single-multiplier eight-frequency Goertzel DFT from SAME
// real SPH0645 audio samples already used by AMDF.  No additional
// I2S decoder or clock. The 40-bit magnitude frame is held stable
// until a toggle, then snapshot in video_clk after 3 flop sync.
e982_mic_goertzel8 u_e982_mic_goertzel8 (
    .clk(audio_mclk), .rst(rst_all),
    .pcm12(e942_pcm12_audio),
    .center12(e942_center_audio),
    .pcm_strobe(e942_pcm_strobe_audio),
    .energy_gate(e942_gate_audio),
    .band_levels(e982_mic_spectrum_audio),
    .frame_toggle(e982_mic_spectrum_toggle)
);


// Snapshot states only change about every 85ms. Two flop synchronizers
// keep them out of the editor's high-rate combinational audio/video logic.
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        e939_left_meta <= 2'b00;
        e939_left_display <= 2'b00;
        e939_right_meta <= 2'b00;
        e939_right_display <= 2'b00;
        e940_blink_sync <= 2'b00;
        e940_changed_sync <= 2'b00;
        e940_level_meta <= 2'b00;
        e940_level_display <= 3'b000;
        e942_pitch_meta <= 6'd0;
        e942_pitch_display <= 6'd0;
        e942_pitch_valid_sync <= 2'b00;
    end else begin
        e939_left_meta <= e939_left_bits_audio;
        e939_left_display <= e939_left_meta;
        e939_right_meta <= e939_right_bits_audio;
        e939_right_display <= e939_right_meta;
        e940_blink_sync <= {e940_blink_sync[0], e940_frame_blink_audio};
        e940_changed_sync <= {e940_changed_sync[0], e940_word_changed_audio};
        e940_level_meta <= e940_voice_level_audio;
        e940_level_display <= e940_level_meta;
        e942_pitch_meta <= e942_pitch_code_audio;
        e942_pitch_display <= e942_pitch_meta;
        e942_pitch_valid_sync <=
            {e942_pitch_valid_sync[0], e942_pitch_valid_audio};
    end
end

// E9.3.2: HDMI output / PCM amplitude diagnostic only.  In SW4=1 mode,
// SW3=1 overrides the editor audio with a fixed, directly generated 440-Hz
// square wave. This test passes through HDMI transmitter audio pins but
// deliberately BYPASSES the SD card, PCM mixer, and SDRAM reader.
// SW3=0 keeps E9.3.1 A/B behavior: SW1=0 square synth; SW1=1 sample PCM.
// SW4=1 editor, SW3=1, SW1=1: C4 from E93F -> existing on-chip
// PCM BRAM -> I2S -> HDMI, bypassing external SDRAM reader and piano FSM.
// Other switch combinations retain exactly their previous audio selection.
reg [2:0] e926_c4_ready_sync_video;
reg [1:0] e926_sw1_sync_video;
wire e926_local_c4_ready = e926_c4_ready_sync_video[2];
wire e926_local_c4_test = E935_KEEP_LEGACY_C4_DIAG &&
                             e932_force_tone && e926_sw1_sync_video[1];
wire e926_local_c4_gate = e926_local_c4_test && e926_local_c4_ready &&
                          (e6_keys_down[0] || !key1_button_state);
reg [1:0] e932_editor_sync, e932_sw3_sync;
reg [31:0] e932_tone_phase;
reg [11:0] e932_pcm_activity, e932_out_activity;
wire e932_force_tone = E936_KEEP_FIXED_440_DIAG &&
                 e932_editor_sync[1] && e932_sw3_sync[1];
wire [23:0] e932_test_sample = e932_tone_phase[31] ?
                              24'he00000 : 24'h200000; // signed +/- 0.25FS
assign audio_left_to_hdmi = e926_local_c4_test ? audio_left_data :
                                   (e932_force_tone ? e932_test_sample : audio_left_e6);
wire [23:0] audio_right_to_hdmi = e926_local_c4_test ? audio_right_data :
                                    (e932_force_tone ? e932_test_sample : audio_right_e6);
wire [23:0] e932_pcm_magnitude = e93_pcm_audio[23] ?
                               (~e93_pcm_audio + 24'd1) : e93_pcm_audio;
wire [23:0] e932_hdmi_magnitude = audio_left_to_hdmi[23] ?
                                (~audio_left_to_hdmi + 24'd1) : audio_left_to_hdmi;
wire e932_pcm_audible = (e932_pcm_activity != 12'd0);
wire e932_hdmi_audible = (e932_out_activity != 12'd0);
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        e932_editor_sync <= 2'b00;
        e932_sw3_sync <= 2'b00;
        e926_sw1_sync_video <= 2'b00;
        e926_c4_ready_sync_video <= 3'b000;
        e932_tone_phase <= 32'd0;
        e932_pcm_activity <= 12'd0;
        e932_out_activity <= 12'd0;
    end else begin
        e932_editor_sync <= {e932_editor_sync[0], sw4};
        e932_sw3_sync <= {e932_sw3_sync[0], sw3};
        e926_sw1_sync_video <= {e926_sw1_sync_video[0], sw1};
        e926_c4_ready_sync_video <= {e926_c4_ready_sync_video[1:0],e926_c4_ready_sd};
        if (audio_valid) begin
            if (e932_force_tone)
                e932_tone_phase <= e932_tone_phase + 32'd39370534; // 440.000005 Hz at 48k
            else
                e932_tone_phase <= 32'd0;
            // These are LIVE held peak-activity meters rather than sticky
            // events: a single ancient nonzero sample cannot keep them lit.
            // 0x008000 = -48.16 dBFS threshold for a signed 24-bit value.
            if (e932_pcm_magnitude >= 24'h008000)
                e932_pcm_activity <= 12'd4095;
            else if (e932_pcm_activity != 0)
                e932_pcm_activity <= e932_pcm_activity - 12'd1;
            if (e932_hdmi_magnitude >= 24'h008000)
                e932_out_activity <= 12'd4095;
            else if (e932_out_activity != 0)
                e932_out_activity <= e932_out_activity - 12'd1;
        end
    end
end

// E9.3.21 R-FAIR-YIELD: PCM requests are allowed during image loading
// only between video transactions and when all of the EXISTING reader's
// FIFO/headroom, SDRAM busy, return-ownership and guard conditions pass.
// The image writer now provides a short 64-clock gap after each full burst.
// This change does not alter the SDRAM controller, PHY, or PLL.
// E9.3 SDRAM: low-priority audio bank DMA + one-word synchronous reader.
// The old image double-buffer addresses and buffer handshakes are unchanged.
// E9.3.9 F: 1=attempt real PCM readback; 0=load-only safe fallback.
localparam E939_ENABLE_PCM_READ = 1'b1;
// H: mono first proves audible data/SDRAM ownership before eight voices.
localparam H11_MONO_TEST = 1'b0;
localparam E935_DUO_TEST = 1'b1;  // E9.3.35: only voices 0/1, cached
e93_sdram_bank_writer #(
    .AUDIO_BASE(21'd655360),
    .AUDIO_WORDS(21'd844672),
    .VIOLIN_BASE(21'd1500032),
    .VIOLIN_WORDS(21'd480000)
) e93_audio_bank_dma (
    .rst(rst_tf), .sd_clk(sd_card_clk), .mem_clk(ext_mem_clk),
    .sd_bank_start(e93_bank_start_sd),  // E9.3.9 F: enabled PCM bank LOAD
    .sd_bank_words(e93_bank_words_sd),
    .sd_pcm_en(e93_word_en_sd),
    .sd_pcm_word(e93_pcm_word_sd),
    .sd_scan_complete(e93_scan_complete_sd),
    .sdram_init_done(Sdr_init_done),
    .sdram_ref_vld(Sdr_init_ref_vld),
    .sdram_busy(Sdr_busy),
    // During an image load, defer NEW PCM DMA bursts until the image
    // frame commits. An already active 128-word burst finishes normally.
    // This avoids starving VIDEO writes without discarding PCM FIFO data.
    .higher_priority_busy(
        video_memory_priority | e937_load_mem_sync[2] |
        video_returns_pending),
    .audio_read_lock(pcm_read_lock),
    .bank_write_busy(pcm_writer_busy),
    .bank_loaded_mem(e93_bank_loaded_mem),
    .violin_loaded_mem(e964_violin_loaded_mem),
    .fifo_overflow_sd(e93_bank_fifo_overflow_sd),
    .App_wr_en(pcm_App_wr_en),
    .App_wr_addr(pcm_App_wr_addr),
    .App_wr_data(pcm_App_wr_din)
);

e93_sdram_reader e93_audio_sdram_reader (
    .rst(rst_tf), .video_clk(video_clk), .mem_clk(ext_mem_clk),
    // Full-PCM experimental restore. Gate in the piano engine by
    // bank_loaded. To diagnose readback separately, switch 1'b1 to 1'b0.
    .req_toggle_video(E939_ENABLE_PCM_READ ? e93_mem_req_toggle : 1'b0),
    .req_addr_video(e93_mem_req_addr),
    .rsp_toggle_mem(e93_mem_rsp_toggle),
    .rsp_word_mem(e93_mem_rsp_word),
    .video_App_rd_en(video_App_rd_en),
    .video_fifo_words_mem(video_fifo_words_mem),
    // Audio may start only when the video FIFO has safe headroom.
    .image_priority_busy(video_memory_priority), // R: image writer now provides safe PCM yield windows
    .pcm_writer_busy(pcm_writer_busy),
    .bank_loaded_mem(e93_bank_loaded_mem),
    .sdram_init_done(Sdr_init_done),
    .sdram_ref_vld(Sdr_init_ref_vld),
    .sdram_busy(Sdr_busy),
    .Sdr_rd_en(Sdr_rd_en),
    .Sdr_rd_dout(Sdr_rd_dout),
    .audio_read_lock(pcm_read_lock),
    .video_returns_pending(video_returns_pending),
    .video_Sdr_rd_en(video_Sdr_rd_en),
    .audio_App_rd_en(pcm_App_rd_en),
    .audio_App_rd_addr(pcm_App_rd_addr),
    .diag_status(e944_diag_status_mem),
    .diag_first_bad_addr(e944_diag_bad_addr_mem),
    .diag_first_bad_expected(e944_diag_expected_mem),
    .diag_first_bad_observed(e944_diag_observed_mem)
);

e93_piano_8voice #(
    .AUDIO_BASE(21'd655360),
    .MONO_TEST(H11_MONO_TEST),
    .DUO_TEST(E935_DUO_TEST)
) e93_piano (
    .clk(video_clk), .rst(rst_all),
    .audio_valid(audio_valid),
    .bank_loaded(e93_loaded_video),
    .violin_loaded(e964_violin_ready_video),
    // 4-track note events replace the old global 25-pitch gate bus.
    .note_evt_valid(e936_note_evt_valid),
    .note_evt_ready(e936_note_evt_ready),
    .note_evt_data(e936_note_evt_data),
    .track_mute_mask(e936_track_mute_mask),
    .pcm_out(e93_pcm_audio),
    .mem_req_toggle(e93_mem_req_toggle),
    .mem_req_addr(e93_mem_req_addr),
    .mem_rsp_toggle(e93_mem_rsp_toggle),
    .mem_rsp_word(e93_mem_rsp_word),
    .underrun(e93_pcm_underrun)
);

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_tf),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .ext_mem_busy(pcm_writer_busy | pcm_read_lock |
                  video_returns_pending),
    .priority_busy(video_memory_priority),
    .video_read_fifo_words_mem(video_fifo_words_mem),
    .video_fifo_underflow_v(video_fifo_underflow_v),

    .App_rd_en         (video_App_rd_en),
    .App_rd_addr       (video_App_rd_addr),
    .Sdr_rd_en         (video_Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx_frame),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (video_App_wr_en),
    .App_wr_addr       (video_App_wr_addr),
    .App_wr_din        (video_App_wr_din),
    .App_wr_dm         (video_App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_tf),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：节拍器 + UI按键音效（第5步） =====================
// strong/weak节拍器与按钮click使用两条独立采样播放轨，允许同时发声。
hdmi_audio_tone_i2s_64fs #(
    .STRONG_COUNT     (1614),
    .WEAK_COUNT       (1890),
    .CLICK_COUNT      (6046),
    .METRO_LEFT_SHIFT (7),
    .CLICK_LEFT_SHIFT (6)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk          (audio_mclk),
    .I_rst           (rst_all),
    .I_strong_toggle (metronome_strong_toggle),
    .I_weak_toggle   (metronome_weak_toggle),
    .I_click_toggle  (key_click_toggle),
    .I_piano_c4_gate (E935_KEEP_LEGACY_C4_DIAG ? e926_local_c4_gate : 1'b0),
    .I_piano_d4_gate (1'b0),
    .I_piano_e4_gate (1'b0),

    // V16: TF-loaded C4 PCM cache write port.
    .I_c4_pcm_wr_clk  (sd_card_clk),
    .I_c4_pcm_wr_en   (E935_KEEP_LEGACY_C4_DIAG ? e926_c4_wr_en_sd : 1'b0),
    .I_c4_pcm_wr_addr (e926_c4_wr_addr_sd),
    .I_c4_pcm_wr_data (e926_c4_wr_data_sd),
    .I_c4_pcm_ready   (E935_KEEP_LEGACY_C4_DIAG ? e926_c4_ready_sd : 1'b0),

    .O_i2s_BCLK      (audio_i2s_bclk),
    .O_i2s_LRCK      (audio_i2s_lrck),
    .O_i2s_DOUT      (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
// E971: RGB compositor has one extra synchronous Chinese ROM stage.
// Delay ONLY the terminal AXIS VS/DE by one clock, keeping upstream
// SD/SDRAM FIFO, piano-roll scan, frame/zoom state and MIC untouched.
reg e971_axis_vs_d, e971_axis_de_d;
always @(posedge video_clk or posedge rst_all) begin
    if(rst_all) begin
        e971_axis_vs_d <= 1'b0;
        e971_axis_de_d <= 1'b0;
    end else begin
        e971_axis_vs_d <= vs;
        e971_axis_de_d <= de;
    end
end
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (e971_axis_vs_d),
    .I_de          (e971_axis_de_d),
    .I_rgb         (vout_data),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_to_hdmi),
    .I_audio_right_data (audio_right_to_hdmi),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule

// ============================================================================
// 动态 BPM 节拍器（当前仅用于 LED 验证）
// ----------------------------------------------------------------------------
// 设计思路：
//   先从50MHz产生1ms tick。
//   每1ms把 bpm 加到 beat_acc。
//   当 beat_acc >= 60000 时产生一拍，并减去60000。
// 因为 60,000 ms / BPM = 一拍的毫秒数，所以无需运行时除法。
// 例如：
//   60 BPM  -> 1000ms/拍
//   120 BPM ->  500ms/拍
//   180 BPM ->  333.33ms/拍（相位累加会自动做平均误差补偿）
// ============================================================================
module metronome_bpm_led #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer FLASH_MS    = 50
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [9:0]  bpm,
    input  wire [4:0]  beats_per_bar,
    output reg         beat_tick,
    output reg         strong_tick,
    output reg         weak_tick,
    output reg [4:0]   current_beat,
    output reg         flash
);

localparam integer MS_CYCLES = CLK_FREQ_HZ / 1000;
// P3.3: actual board is 50MHz => 50,000 clock cycles per 1ms.
// 16-bit ms counter and 6-bit flash length are sufficient;
// retain 32/16bits automatically if parameter is increased.
localparam integer MS_COUNTER_BITS = (MS_CYCLES <= 65536) ? 16 : 32;
localparam integer FLASH_COUNTER_BITS = (FLASH_MS <= 64) ? 6 : 16;
reg [MS_COUNTER_BITS-1:0] ms_cnt;
reg [15:0] beat_acc; // always < 60000, bpm max 1023 => sum < 61023
reg [FLASH_COUNTER_BITS-1:0] flash_ms_cnt;
reg [4:0]  beat_in_bar;    // 0表示本小节第1拍
reg [1:0]  enable_sync;

wire enable_s = enable_sync[1];
wire [15:0] beat_sum = beat_acc + bpm;

// SW2同步到50MHz域
always @(posedge clk or posedge rst) begin
    if (rst)
        enable_sync <= 2'b00;
    else
        enable_sync <= {enable_sync[0], enable};
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ms_cnt       <= 0;
        beat_acc     <= 0;
        flash_ms_cnt <= 0;
        beat_in_bar  <= 5'd0;
        beat_tick    <= 1'b0;
        strong_tick  <= 1'b0;
        weak_tick    <= 1'b0;
        current_beat <= 5'd1;
        flash        <= 1'b0;
    end
    else if (!enable_s) begin
        ms_cnt       <= 0;
        beat_acc     <= 0;
        flash_ms_cnt <= 0;
        beat_in_bar  <= 5'd0;
        beat_tick    <= 1'b0;
        strong_tick  <= 1'b0;
        weak_tick    <= 1'b0;
        current_beat <= 5'd1;
        flash        <= 1'b0;
    end
    else begin
        beat_tick   <= 1'b0;
        strong_tick <= 1'b0;
        weak_tick   <= 1'b0;

        if (ms_cnt >= (MS_CYCLES - 1)) begin
            ms_cnt <= 0;

            // LED闪烁宽度计时
            if (flash_ms_cnt != 0) begin
                flash_ms_cnt <= flash_ms_cnt - 1'b1;
                flash <= 1'b1;
            end
            else begin
                flash <= 1'b0;
            end

            // BPM动态节拍累加
            if (beat_sum >= 16'd60000) begin
                beat_acc     <= beat_sum - 16'd60000;
                beat_tick    <= 1'b1;
                flash        <= 1'b1;
                flash_ms_cnt <= FLASH_MS - 1;

                // 每小节第1拍为strong，其余为weak。
                // beats_per_bar动态修改后，最迟下一拍自动按新拍数重新循环。
                if (beat_in_bar == 5'd0)
                    strong_tick <= 1'b1;
                else
                    weak_tick <= 1'b1;

                current_beat <= beat_in_bar + 5'd1;

                if (beat_in_bar >= (beats_per_bar - 1'b1))
                    beat_in_bar <= 5'd0;
                else
                    beat_in_bar <= beat_in_bar + 5'd1;
            end
            else begin
                beat_acc <= beat_sum[15:0];
            end
        end
        else begin
            ms_cnt <= ms_cnt + 1'b1;
        end
    end
end

endmodule


// ============================================================================
// E7.1: 25 semitone, 256-frame rolling history, IMAGE-MODE overlay.
// One 256x25 synchronous read/write bitmap is much smaller in LUT than
// comparing numerous independent left/right note sprites per pixel.
// x=606..607 means the most recent frame, 2 pixels older each frame leftward.
// y=450..453 C4, y=354..357 C6, every semitone occupies four scan lines.
// 25 simultaneous pressed notes are supported, including the legacy FPGA keys.
// ============================================================================
module music_score_osd #(
    parameter E950_KEEP_LIVE_KEY_HISTORY = 1'b1
)(
    input wire video_clk,input wire rst,input wire enable,input wire running,
    input wire show_status,
    input wire vs,input wire de,input wire [23:0] base_rgb,
    input wire [24:0] keys_25,
    input wire image_ready,
    input wire [3:0] bpm_hundreds,input wire [3:0] bpm_tens,input wire [3:0] bpm_ones,
    input wire [4:0] beats_per_bar,input wire [4:0] current_beat,
    input wire c4_gate,input wire d4_gate,input wire e4_gate,
    input wire piano_bank_found,
    output reg [23:0] out_rgb
);
reg [9:0] x;
reg [8:0] y;
reg de_d,vs_d;
wire frame_tick=vs_d && !vs;
reg [1:0] en_s,run_s,img_s,bank_s,bar_s;
reg [24:0] live_sync1,live_sync2;
// MIDI note indices: C4=0 D4=2 E4=4.
wire [24:0] notes_in = keys_25 |
    (c4_gate ? 25'h0000001:25'd0) |
    (d4_gate ? 25'h0000004:25'd0) |
    (e4_gate ? 25'h0000010:25'd0);
reg en_f,run_f,img_f,bank_f,bar_f;
reg [3:0] bhf,btf,bof;
reg [4:0] bpf,cbf;
reg [3:0] bh1,bt1,bo1;
reg [4:0] bp1,cb1;
wire [3:0] mt=(bpf>=5'd10)? 4'd1:4'd0;
wire [3:0] mo=(bpf>=5'd10)?bpf-5'd10:bpf[3:0];

// 256 frames * 25 notes = 6400 bits; should infer two BRAM9K blocks.
// Never reset memory bit-by-bit (that would prevent block RAM inference).
(* ram_style = "block" *) reg [24:0] roll_history [0:255];
reg [7:0] wr_index;
reg [8:0] frames_filled;
reg [24:0] roll_read_data;
// The BRAM read is synchronous. Prefetch one pixel forward so a hit
// corresponds to the current live raster pixel without adding video latency.
wire [9:0] ahead_x=(x<10'd639)?x+10'd1:10'd639;
wire [9:0] ahead_age_10=(10'd607-ahead_x)>>1;
wire [7:0] history_read_address = wr_index-8'd1-ahead_age_10[7:0];
wire [9:0] age10=(10'd607-x)>>1;
wire [8:0] age9={1'b0,age10[7:0]};
wire [8:0] rev_y=9'd453-y;
wire [4:0] shown_pitch=rev_y[6:2];
wire in_roll=(x>=10'd96 && x<10'd608 && y>=9'd354 && y<=9'd453);
wire roll_on=E950_KEEP_LIVE_KEY_HISTORY && in_roll &&
             (rev_y[1:0]!=2'd0) &&
             (frames_filled>age9) && roll_read_data[shown_pitch];

always @(posedge video_clk or posedge rst) begin
    if(rst) begin
        x<=0;y<=0;de_d<=1'b0;vs_d<=1'b1;
        en_s<=0;run_s<=0;img_s<=0;bank_s<=0;bar_s<=0;
        live_sync1<=0;live_sync2<=0;
        bh1<=4'd1;bt1<=4'd2;bo1<=4'd0;bp1<=5'd4;cb1<=5'd1;
        bhf<=4'd1;btf<=4'd2;bof<=4'd0;bpf<=5'd4;cbf<=5'd1;
        en_f<=0;run_f<=0;img_f<=0;bank_f<=0;bar_f<=0;
        wr_index<=8'd0;frames_filled<=9'd0;
    end else begin
        de_d<=de;vs_d<=vs;
        if(!vs) y<=9'd0;
        else if(de_d && !de && y<9'd479) y<=y+9'd1;
        if(de) begin
            if(!de_d) x<=10'd0;
            else if(x<10'd639) x<=x+10'd1;
        end else x<=10'd0;
        en_s<={en_s[0],enable};run_s<={run_s[0],running};
        img_s<={img_s[0],image_ready};bank_s<={bank_s[0],piano_bank_found};
        bar_s<={bar_s[0],show_status};
        live_sync1<=notes_in;live_sync2<=live_sync1;
        bh1<=bpm_hundreds;bt1<=bpm_tens;bo1<=bpm_ones;
        bp1<=beats_per_bar;cb1<=current_beat;
        if(frame_tick) begin
            en_f<=en_s[1];run_f<=run_s[1];
            img_f<=img_s[1];bank_f<=bank_s[1];bar_f<=bar_s[1];
            bhf<=bh1;btf<=bt1;bof<=bo1;bpf<=bp1;cbf<=cb1;
            if(!en_s[1]) begin
                wr_index<=8'd0;
                frames_filled<=9'd0;
            end else begin
                wr_index<=wr_index+8'd1;
                if(frames_filled<9'd256)frames_filled<=frames_filled+9'd1;
            end
        end
    end
end

// Two ports of the same block RAM, video-clock domain only.
always @(posedge video_clk) begin
    roll_read_data <= roll_history[history_read_address];
    if(E950_KEEP_LIVE_KEY_HISTORY && frame_tick && en_s[1])
        roll_history[wr_index] <= live_sync2;
end

function [4:0] font5x7;
 input [7:0] ch; input [2:0] row;
 begin
  font5x7=5'b00000;
  case(ch)
            8'h42: case(row) 0: font5x7=5'b11110; 1: font5x7=5'b10001; 2: font5x7=5'b10001; 3: font5x7=5'b11110; 4: font5x7=5'b10001; 5: font5x7=5'b10001; 6: font5x7=5'b11110; endcase
            8'h50: case(row) 0: font5x7=5'b11110; 1: font5x7=5'b10001; 2: font5x7=5'b10001; 3: font5x7=5'b11110; 4: font5x7=5'b10000; 5: font5x7=5'b10000; 6: font5x7=5'b10000; endcase
            8'h4D: case(row) 0: font5x7=5'b10001; 1: font5x7=5'b11011; 2: font5x7=5'b10101; 3: font5x7=5'b10101; 4: font5x7=5'b10001; 5: font5x7=5'b10001; 6: font5x7=5'b10001; endcase
            8'h30: case(row) 0: font5x7=5'b01110; 1: font5x7=5'b10001; 2: font5x7=5'b10011; 3: font5x7=5'b10101; 4: font5x7=5'b11001; 5: font5x7=5'b10001; 6: font5x7=5'b01110; endcase
            8'h31: case(row) 0: font5x7=5'b00100; 1: font5x7=5'b01100; 2: font5x7=5'b00100; 3: font5x7=5'b00100; 4: font5x7=5'b00100; 5: font5x7=5'b00100; 6: font5x7=5'b01110; endcase
            8'h32: case(row) 0: font5x7=5'b01110; 1: font5x7=5'b10001; 2: font5x7=5'b00001; 3: font5x7=5'b00010; 4: font5x7=5'b00100; 5: font5x7=5'b01000; 6: font5x7=5'b11111; endcase
            8'h33: case(row) 0: font5x7=5'b11110; 1: font5x7=5'b00001; 2: font5x7=5'b00001; 3: font5x7=5'b01110; 4: font5x7=5'b00001; 5: font5x7=5'b00001; 6: font5x7=5'b11110; endcase
            8'h34: case(row) 0: font5x7=5'b00010; 1: font5x7=5'b00110; 2: font5x7=5'b01010; 3: font5x7=5'b10010; 4: font5x7=5'b11111; 5: font5x7=5'b00010; 6: font5x7=5'b00010; endcase
            8'h35: case(row) 0: font5x7=5'b11111; 1: font5x7=5'b10000; 2: font5x7=5'b10000; 3: font5x7=5'b11110; 4: font5x7=5'b00001; 5: font5x7=5'b00001; 6: font5x7=5'b11110; endcase
            8'h36: case(row) 0: font5x7=5'b01110; 1: font5x7=5'b10000; 2: font5x7=5'b10000; 3: font5x7=5'b11110; 4: font5x7=5'b10001; 5: font5x7=5'b10001; 6: font5x7=5'b01110; endcase
            8'h37: case(row) 0: font5x7=5'b11111; 1: font5x7=5'b00001; 2: font5x7=5'b00010; 3: font5x7=5'b00100; 4: font5x7=5'b01000; 5: font5x7=5'b01000; 6: font5x7=5'b01000; endcase
            8'h38: case(row) 0: font5x7=5'b01110; 1: font5x7=5'b10001; 2: font5x7=5'b10001; 3: font5x7=5'b01110; 4: font5x7=5'b10001; 5: font5x7=5'b10001; 6: font5x7=5'b01110; endcase
            8'h39: case(row) 0: font5x7=5'b01110; 1: font5x7=5'b10001; 2: font5x7=5'b10001; 3: font5x7=5'b01111; 4: font5x7=5'b00001; 5: font5x7=5'b00001; 6: font5x7=5'b01110; endcase
            8'h2F: case(row) 0: font5x7=5'b00001; 1: font5x7=5'b00010; 2: font5x7=5'b00010; 3: font5x7=5'b00100; 4: font5x7=5'b01000; 5: font5x7=5'b01000; 6: font5x7=5'b10000; endcase
            8'h54: case(row) 0:font5x7=5'b11111;1:font5x7=5'b00100;2:font5x7=5'b00100;3:font5x7=5'b00100;4:font5x7=5'b00100;5:font5x7=5'b00100;6:font5x7=5'b00100;endcase
            8'h46: case(row) 0:font5x7=5'b11111;1:font5x7=5'b10000;2:font5x7=5'b10000;3:font5x7=5'b11110;4:font5x7=5'b10000;5:font5x7=5'b10000;6:font5x7=5'b10000;endcase
            8'h57: case(row) 0:font5x7=5'b10001;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b10101;4:font5x7=5'b10101;5:font5x7=5'b10101;6:font5x7=5'b01010;endcase
            8'h41: case(row) 0:font5x7=5'b01110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b11111;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            8'h49: case(row) 0:font5x7=5'b11111;1:font5x7=5'b00100;2:font5x7=5'b00100;3:font5x7=5'b00100;4:font5x7=5'b00100;5:font5x7=5'b00100;6:font5x7=5'b11111;endcase
  endcase
 end
endfunction

function [7:0] da;
 input [3:0] d;
 begin da=8'h30+d; end
endfunction


// One shared ASCII font renderer for the 24px BPM/status strip.
reg score_txt_enable;
reg [7:0] score_txt_ch;
reg [9:0] score_txt_x0;
reg [3:0] score_txt_dx,score_txt_dy;
reg [4:0] score_txt_row;
reg score_txt_pixel;
always @* begin
    score_txt_enable=1'b0;
    score_txt_ch=8'h20;
    score_txt_x0=10'd0;
    score_txt_dx=4'd0;score_txt_dy=4'd0;
    score_txt_row=5'd0;
    score_txt_pixel=1'b0;
    if(y>=9'd461 && y<9'd475) begin
        if(x>=10'd10 && x<10'd20) begin score_txt_enable=1'b1;score_txt_ch="B";score_txt_x0=10'd10; end
        else if(x>=10'd22 && x<10'd32) begin score_txt_enable=1'b1;score_txt_ch="P";score_txt_x0=10'd22; end
        else if(x>=10'd34 && x<10'd44) begin score_txt_enable=1'b1;score_txt_ch="M";score_txt_x0=10'd34; end
        else if(x>=10'd50 && x<10'd60) begin score_txt_enable=1'b1;score_txt_ch=da(bhf);score_txt_x0=10'd50; end
        else if(x>=10'd62 && x<10'd72) begin score_txt_enable=1'b1;score_txt_ch=da(btf);score_txt_x0=10'd62; end
        else if(x>=10'd74 && x<10'd84) begin score_txt_enable=1'b1;score_txt_ch=da(bof);score_txt_x0=10'd74; end
        else if(x>=10'd112 && x<10'd122 && mt!=0) begin score_txt_enable=1'b1;score_txt_ch=da(mt);score_txt_x0=10'd112; end
        else if(x>=10'd124 && x<10'd134) begin score_txt_enable=1'b1;score_txt_ch=da(mo);score_txt_x0=10'd124; end
        else if(x>=10'd136 && x<10'd146) begin score_txt_enable=1'b1;score_txt_ch="/";score_txt_x0=10'd136; end
        else if(x>=10'd148 && x<10'd158) begin score_txt_enable=1'b1;score_txt_ch="4";score_txt_x0=10'd148; end
        else if(bank_f && x>=10'd176 && x<10'd186) begin score_txt_enable=1'b1;score_txt_ch="P";score_txt_x0=10'd176; end
        // A visible TF LOAD indicator distinguishes a still-empty image
        // framebuffer from a black BMP file or a broken HDMI output.
        else if(!img_f && x>=10'd450 && x<10'd460) begin score_txt_enable=1'b1;score_txt_ch="T";score_txt_x0=10'd450; end
        else if(!img_f && x>=10'd462 && x<10'd472) begin score_txt_enable=1'b1;score_txt_ch="F";score_txt_x0=10'd462; end
        else if(!img_f && x>=10'd480 && x<10'd490) begin score_txt_enable=1'b1;score_txt_ch="W";score_txt_x0=10'd480; end
        else if(!img_f && x>=10'd492 && x<10'd502) begin score_txt_enable=1'b1;score_txt_ch="A";score_txt_x0=10'd492; end
        else if(!img_f && x>=10'd504 && x<10'd514) begin score_txt_enable=1'b1;score_txt_ch="I";score_txt_x0=10'd504; end
        else if(!img_f && x>=10'd516 && x<10'd526) begin score_txt_enable=1'b1;score_txt_ch="T";score_txt_x0=10'd516; end
        score_txt_dx=x-score_txt_x0;
        score_txt_dy=y-9'd461;
        score_txt_row=font5x7(score_txt_ch,score_txt_dy[3:1]);
        case(score_txt_dx[3:1])
            3'd0:score_txt_pixel=score_txt_row[4];
            3'd1:score_txt_pixel=score_txt_row[3];
            3'd2:score_txt_pixel=score_txt_row[2];
            3'd3:score_txt_pixel=score_txt_row[1];
            3'd4:score_txt_pixel=score_txt_row[0];
            default:score_txt_pixel=1'b0;
        endcase
        score_txt_pixel=score_txt_pixel && score_txt_enable;
    end
end
// Decode the 16 BPM beat dots from a single horizontal counter.
reg [3:0] score_beat_cell;
reg [3:0] score_beat_offset;
always @(posedge video_clk or posedge rst) begin
    if(rst)begin score_beat_cell<=0;score_beat_offset<=0;end
    else if(de) begin
        if(x==10'd204)begin score_beat_cell<=0;score_beat_offset<=0;end
        else if(x>=10'd205 && x<10'd412) begin
            if(score_beat_offset==4'd12)begin
                score_beat_cell<=score_beat_cell+4'd1;
                score_beat_offset<=0;
            end else score_beat_offset<=score_beat_offset+4'd1;
        end
    end
end
wire score_beat_dot=(x>=10'd205 && x<10'd413 &&
                    score_beat_offset<4'd8 &&
                    {1'b0,score_beat_cell}<bpf &&
                    y>=9'd464 && y<9'd472);
always @* begin
    out_rgb=base_rgb;
    if(en_f && de)begin
        if(roll_on) out_rgb=24'h26c6da;
        // A thin note-on marker to the left of history (not over picture
        // except for the currently held pitches).
        if(E950_KEEP_LIVE_KEY_HISTORY &&
           x>=10'd88 && x<10'd94 && y>=9'd354 && y<=9'd453 &&
           live_sync2[shown_pitch])out_rgb=24'hffde57;
        // SW3=0 keeps the original BMP completely visible except for note
        // blocks. SW3=1 shows the legacy BPM bar. Missing BMP always shows
        // TF WAIT, so a blank screen is distinguishable from slow loading.
        if(y>=9'd456 && (bar_f || !img_f)) begin
            out_rgb=24'h101820;
            if(y==9'd456)out_rgb=24'h4a6078;
            if(score_txt_pixel)out_rgb=img_f? 24'hffffff:24'hffc266;
            if(score_beat_dot)
                out_rgb=(run_f && cbf==({1'b0,score_beat_cell}+5'd1))?
                        24'hffd54f:24'h607d8b;
        end
    end
end
endmodule

// ============================================================================
// PCB K1..K5, EC11 and 5x5 matrix, 25 MHz pixel clock.
// Matrix columns are OPEN-DRAIN STYLE: only the selected column is driven LOW;
// all other columns MUST be high impedance at the TOP LEVEL.
module pcb_e6_button_db #(
    parameter integer STABLE_CYCLES = 300000
)(
    input wire clk, input wire rst,
    input wire pin_n,
    output reg pressed,
    output reg down
);
    reg [1:0] sync;
    reg [18:0] counter;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            sync <= 2'b11;
            counter <= 19'd0;
            down <= 1'b0;
            pressed <= 1'b0;
        end else begin
            sync <= {sync[0],pin_n};
            pressed <= 1'b0;
            if((~sync[1]) == down) counter <= 19'd0;
            else if(counter >= STABLE_CYCLES-1) begin
                counter <= 19'd0;
                down <= ~sync[1];
                if(!sync[1]) pressed <= 1'b1;
            end else counter <= counter+1'b1;
        end
    end
endmodule

module pcb_e6_inputs(
    input wire clk, input wire rst, input wire enable,
    input wire [4:0] btn_n,
    input wire enc_sw_n, enc_a_n, enc_b_n,
    input wire [4:0] row_n,
    output wire [4:0] col_drive_low,
    output wire [4:0] btn_pulse,
    output wire [4:0] btn_down,
    output wire enc_press,
    output reg enc_cw,
    output reg enc_ccw,
    output reg [24:0] keys_down,
    output reg [24:0] key_press
);
    wire [4:0] ignored_btn_down;
    assign btn_down = ignored_btn_down;
    genvar g;
    generate for(g=0;g<5;g=g+1) begin: GEN_BTN
        pcb_e6_button_db U_DB (
            .clk(clk),.rst(rst),.pin_n(btn_n[g]),
            .pressed(btn_pulse[g]),.down(ignored_btn_down[g])
        );
    end endgenerate
    wire sw_down_unused;
    pcb_e6_button_db U_ENC_SW (
        .clk(clk),.rst(rst),.pin_n(enc_sw_n),
        .pressed(enc_press),.down(sw_down_unused)
    );

    // One column for 500us; 5 columns = 2.5ms. 8 mismatching scans
    // are required to change one key's debounced state (~20ms).
    reg [13:0] scan_div;
    reg [2:0] col_index;
    reg [4:0] row_sync1, row_sync2;
    reg [3:0] key_filter[0:24];
    reg [24:0] keys_prev;
    integer i;
    integer n;
    assign col_drive_low = enable ? (5'b00001 << col_index) : 5'b00000;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            scan_div <= 14'd0;
            col_index <= 3'd0;
            row_sync1 <= 5'b11111;
            row_sync2 <= 5'b11111;
            keys_down <= 25'd0;
            keys_prev <= 25'd0;
            key_press <= 25'd0;
            for(i=0;i<25;i=i+1) key_filter[i] <= 4'd0;
        end else begin
            row_sync1 <= row_n;
            row_sync2 <= row_sync1;
            keys_prev <= keys_down;
            key_press <= enable ? (keys_down & ~keys_prev) : 25'd0;
            if(!enable) begin
                scan_div <= 14'd0;
                col_index <= 3'd0;
                keys_down <= 25'd0;
                keys_prev <= 25'd0;
                for(i=0;i<25;i=i+1) key_filter[i] <= 4'd0;
            end else if(scan_div == 14'd12499) begin
                scan_div <= 14'd0;
                for(i=0;i<5;i=i+1) begin
                    n=i*5+col_index;
                    if((~row_sync2[i]) == keys_down[n])
                        key_filter[n] <= 4'd0;
                    else if(key_filter[n] == 4'd7) begin
                        key_filter[n] <= 4'd0;
                        keys_down[n] <= ~row_sync2[i];
                    end else key_filter[n] <= key_filter[n]+1'b1;
                end
                if(col_index == 3'd4) col_index <= 3'd0;
                else col_index <= col_index+1'b1;
            end else scan_div <= scan_div+1'b1;
        end
    end

    // EC11 quadrature, 250us sample, 4 valid gray-code transitions per event.
    // Physical CW/CCW might be reversed: swap A/B in the TOP port mapping then.
    reg [1:0] a_sync,b_sync;
    reg [12:0] enc_div;
    reg [1:0] ab_last;
    reg signed [3:0] quarter_count;
    wire [1:0] ab_now={a_sync[1],b_sync[1]};
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            a_sync<=2'b11; b_sync<=2'b11;
            enc_div<=13'd0; ab_last<=2'b11;
            quarter_count<=4'sd0;
            enc_cw<=1'b0;enc_ccw<=1'b0;
        end else begin
            a_sync<={a_sync[0],enc_a_n};
            b_sync<={b_sync[0],enc_b_n};
            enc_cw<=1'b0;enc_ccw<=1'b0;
            if(!enable) begin
                enc_div<=13'd0;
                ab_last<=ab_now;
                quarter_count<=4'sd0;
            end else if(enc_div==13'd6249) begin
                enc_div<=13'd0;
                case({ab_last,ab_now})
                    4'b0001,4'b0111,4'b1110,4'b1000: begin
                        if(quarter_count==4'sd3) begin
                            enc_cw<=1'b1;
                            quarter_count<=4'sd0;
                        end else if(quarter_count<0)
                            quarter_count<=4'sd1;
                        else quarter_count<=quarter_count+4'sd1;
                    end
                    4'b0010,4'b1011,4'b1101,4'b0100: begin
                        if(quarter_count== -4'sd3) begin
                            enc_ccw<=1'b1;
                            quarter_count<=4'sd0;
                        end else if(quarter_count>0)
                            quarter_count<=-4'sd1;
                        else quarter_count<=quarter_count-4'sd1;
                    end
                    default: begin
                        if(ab_last!=ab_now) quarter_count<=4'sd0;
                    end
                endcase
                ab_last<=ab_now;
            end else enc_div<=enc_div+1'b1;
        end
    end
endmodule

// ---------------------------------------------------------------------------
// E6 music editor -- FPGA-only prototype (no nonvolatile song storage).
//  4 independent tracks x 16 note slots, MIDI C4..C6 / 25 pitches.
//  Each note: valid,pitch,start,length,velocity.
//  P1: 4-track 256-slot Note Events -> shared 4-voice Piano PCM engine,
//  plus 25 shared sustained Square oscillators, mixed simultaneously.
//  Note On/Off preserves track ID and stored slot/live-note identity.
//  The third instrument bank (e.g. violin) is NOT included.
// ---------------------------------------------------------------------------

// E8 experimental loop recorder; replaces the previous E7 step editor.
// E8 Loop Lab / 4-track real-time overdub editor.
// Video/audio/control share the existing 25MHz pixel clock.
// The enclosing top and all HDMI/TF/SDRAM interfaces remain unchanged.
// E8 Loop Lab / 4-track real-time overdub editor.
// Video/audio/control share the existing 25MHz pixel clock.
// The enclosing top and all HDMI/TF/SDRAM interfaces remain unchanged.
// E8 Loop Lab / 4-track real-time overdub editor.
// Video/audio/control share the existing 25MHz pixel clock.
// The enclosing top and all HDMI/TF/SDRAM interfaces remain unchanged.
// E8 Loop Lab / 4-track real-time overdub editor.
// Video/audio/control share the existing 25MHz pixel clock.
// The enclosing top and all HDMI/TF/SDRAM interfaces remain unchanged.
module editor_loop_e8 #(
    parameter KEEP_E93_DEBUG_UI = 1'b1
) (
    input  wire        clk, rst,
    input  wire        sw4, sw1, sw2, sw3,
    input  wire [2:0]  board_keys_down, // [0] KEY1, [1] KEY2, [2] KEY4
    output wire        mini_keys_active,
    // Frame-latched by TOP and shared by memory/video zoom controllers.
    output wire [1:0]  full_zoom_level,
    output wire [9:0]  full_zoom_origin_x,
    output wire [8:0]  full_zoom_origin_y,
    input  wire [9:0]  bpm,
    input  wire [3:0]  bpm_hundreds, bpm_tens, bpm_ones,
    input  wire [4:0]  buttons, button_down,
    input  wire        metro_sound_enabled,
    input  wire        enc_button, enc_cw, enc_ccw,
    output wire        tempo_up, tempo_down,
    output reg         metro_strong_toggle, metro_weak_toggle,
    input  wire [24:0] key_down,
    input  wire [2:0]  old_keys, // [0]=C4 [1]=D4 [2]=E4 (only if SW3)
    input  wire        vs, de,
    input  wire [23:0] video_in,
    output reg  [23:0] video_out,
    input  wire        audio_valid,
    input  wire [23:0] audio_left_in, audio_right_in,
    output wire [23:0] audio_left_out, audio_right_out,
    input wire e93_pcm_loaded,
    input wire e964_violin_loaded,
    input wire [23:0] e93_pcm_audio,
    input wire [39:0] e982_mic_bands,
    input wire e982_mic_bands_toggle,
    input wire [10:0] e93_debug_flags,
    input wire e932_pcm_live, e932_hdmi_live,
    input wire [8:0] e933_view_status,
    input wire [31:0] e941_tf_word,
    input wire e941_tf_ready,
    input wire [31:0] e941_c4_read_word,
    input wire e941_c4_read_ready,
    input wire [31:0] e943_first_wr_word,
    input wire [31:0] e943_first_wr_addr,
    input wire e943_first_wr_ready,
    input wire [31:0] e943_wr_count,
    input wire e943_wr_count_ready,
    input wire e943_collision,
    input wire [31:0] e944_diag_status, e944_diag_bad_addr,
    input wire [31:0] e944_diag_expected, e944_diag_observed,
    output wire [174:0] e93_pitch_gain,
    output wire        note_evt_valid,
    input  wire        note_evt_ready,
    output wire [29:0] note_evt_data,
    output wire [3:0]  track_mute_mask,
    input wire [1:0] mic_left_pin_state,
    input wire [1:0] mic_right_pin_state,
    input wire mic_frame_blink,
    input wire mic_word_changed,
    input wire [2:0] mic_voice_level,
    input wire [5:0] mic_pitch_code,  // octave_idx[1:0], note_class[3:0]
    input wire mic_pitch_valid
);
    // 256 notes, 64 per track, each event = pitch/start/duration/velocity.
    // Stored in an inferred RAM, never reset the RAM itself. Only the
    // 256-bit validity map is reset. The RAM has two registered read ports
    // (audio and OSD) and one write port.
    // Two mirrored single-read single-write RAMs deliberately avoid a
    // three-port asynchronous 256x31 mux in synthesis. ~2x BRAM9K total.
    reg [30:0] note_ram_audio [0:255];
    reg [30:0] note_ram_view  [0:255];
    // E956 / P3.12: NO THIRD 256x31 NOTE RAM.
    // Picture-mini "find/delete" queries piggyback on the SAME view_q
    // read already used for the full editor's 64x4-track bitmap build.
    // Audio and view retain independent synchronous RAM read ports.
    reg [255:0] note_valid;
    reg [30:0] audio_q, view_q;
    reg [7:0] audio_addr, view_idx;
    always @(posedge clk) begin
        audio_q <= note_ram_audio[audio_addr];
        view_q  <= note_ram_view[view_idx];
    end

    reg [1:0] en_ff, play_ff;
    reg [9:0] bpm_ff1, bpm_ff2;
    // Recording state is referenced by the shared music clock,
    // so it is explicitly declared here (before the clock wires).
    reg [24:0] held_valid;
    reg event_pending;
    wire editor_on = en_ff[1];
    wire e950_mini_active;
    // P5.0 ONE transport: SW1 enables time/auto-record in both screens.
    // A displayed mini editor is NOT enough to start the music clock.
    // Keep the old flush drain so a mid-press pause still sends NoteOff.
    // SW1 is GLOBAL: stored tracks play while viewing BMP even
    // when the mini editor is hidden. UI modes only affect capture.
    wire e958_transport_play = play_ff[1];
    wire e958_flush_pending;
    wire running = e958_transport_play || e958_flush_pending;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            en_ff <= 2'b00;
            play_ff <= 2'b00;
            bpm_ff1 <= 10'd120;
            bpm_ff2 <= 10'd120;
        end else begin
            en_ff <= {en_ff[0],sw4};
            play_ff <= {play_ff[0],sw1}; // global transport, SW2 only metro
            bpm_ff1 <= bpm;
            bpm_ff2 <= bpm_ff1;
        end
    end

    reg [1:0] track_sel;
    reg e952_mic_rec_enabled; // onboard K4 in full editor: MIC REC ON/OFF
    // P5.0: board KEY4 toggles BMP mini edit whenever SW4=0.
    // SW1=0 still permits viewing notes but NEVER automatic capture.
    // K1/K5 then choose the highlighted real track.
    // E952: PCB onboard KEY1, KEY2, KEY4 are debounced in clk=50MHz,
    // then re-synchronized as stable levels into this video_clk=25MHz.
    // KEY3 is the board's global reset (rst_n/B1) and stays reserved.
    reg e950_mini_show;
    reg [1:0] e952_sw3_ff; // SW1 already synchronized in play_ff
    reg [2:0] e952_board_d1, e952_board_d2, e952_board_prev;
    wire e952_b1 = e952_board_d2[0];
    wire e952_b2 = e952_board_d2[1];
    wire e952_b4 = e952_board_d2[2];
    wire e952_b4_press = e952_b4 && !e952_board_prev[2];
    // Reuse existing debounced board KEY1/2 release events in both
    // full editor and mini image edit, no extra debounce timers.
    wire e952_b1_release = !e952_b1 && e952_board_prev[0];
    wire e952_b2_release = !e952_b2 && e952_board_prev[1];
    wire e950_image_mode = !editor_on; // SW4=0 always displays TF BMP
    wire e952_image_key_mode = e950_image_mode && !e952_sw3_ff[1];
    assign e950_mini_active = e952_image_key_mode && e950_mini_show;
    assign mini_keys_active = e950_mini_active;
    // No extra image buffers.  Board K4 changes the existing panel flag,
    // not TF/PCM loading, and is only active when SW3 is DOWN.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            e950_mini_show <= 1'b0;
            e952_sw3_ff <= 2'b00;
            e952_board_d1 <= 3'b000;
            e952_board_d2 <= 3'b000;
            e952_board_prev <= 3'b000;
        end else begin
            e952_sw3_ff <= {e952_sw3_ff[0],sw3};
            e952_board_d1 <= board_keys_down;
            e952_board_d2 <= e952_board_d1;
            e952_board_prev <= e952_board_d2;
            if (e952_image_key_mode && e952_b4_press)
                e950_mini_show <= ~e950_mini_show;
        end
    end

    // E962/P4.1: Full-screen BMP viewport, sharing the ORIGINAL SDRAM
    // framebuffer and video read FIFO.  NO secondary frame buffer.
    // 0: full 640x480 (1x); 1: 320x240 ROI (2x); 2: 160x120 ROI (4x).
    // Decreasing from 4x -> 2x -> 1x reveals a wider part of the score.
    reg [1:0] e962_level;
    reg [9:0] e962_center_x;
    reg [8:0] e962_center_y;
    wire e962_picture_control = e952_image_key_mode;
    wire e962_zoom_page_loading = e933_view_status[3];

    // E978 / P4.7: preserve all image controls by making BRIGHTNESS
    // a momentary chord: HOLD physical expansion CENTER (K3), rotate
    // EC11 clockwise to brighten or CCW to darken.
    // A short CENTER tap still centers the full-screen zoom, but
    // it is acted upon at RELEASE so a brightness chord does not
    // accidentally reset the user's zoom.
    // No new external switch/pin and no 24-bit long-press counter.
    reg [1:0] e978_brightness_level; // 0=25%, 1=50%, 2=100%.
    reg e978_center_was_down;
    reg e978_center_was_adjusted;
    wire e978_brightness_chord =
        e962_picture_control && button_down[2];
    wire e978_center_tap_release =
        e962_picture_control &&
        e978_center_was_down && !button_down[2] &&
        !e978_center_was_adjusted;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            e978_brightness_level <= 2'd2; // full brightness
            e978_center_was_down <= 1'b0;
            e978_center_was_adjusted <= 1'b0;
        end else begin
            e978_center_was_down <= button_down[2];
            if (!e962_picture_control || !button_down[2])
                e978_center_was_adjusted <= 1'b0;
            else if(enc_cw || enc_ccw)
                e978_center_was_adjusted <= 1'b1;
            if(e978_brightness_chord) begin
                if(enc_cw && e978_brightness_level<2'd2)
                    e978_brightness_level <= e978_brightness_level + 2'd1;
                else if(enc_ccw && e978_brightness_level>2'd0)
                    e978_brightness_level <= e978_brightness_level - 2'd1;
            end
        end
    end

    // Literal RGB shift, not a new multiplier or DSP.
    // This source is selected ONLY for the raw BMP/video input tag 0;
    // the on-screen font, piano roll, mini editor and wavelength use
    // the original untouched full-brightness RGB palette.
    wire [23:0] e978_bmp_half = {
        1'b0,video_in[23:17], 1'b0,video_in[15:9],
        1'b0,video_in[7:1]
    };
    wire [23:0] e978_bmp_quarter = {
        2'b00,video_in[23:18], 2'b00,video_in[15:10],
        2'b00,video_in[7:2]
    };
    wire [23:0] e978_bmp_brightness_rgb =
        !e962_picture_control ? video_in :
        (e978_brightness_level==2'd2) ? video_in :
        (e978_brightness_level==2'd1) ? e978_bmp_half :
                                        e978_bmp_quarter;

    // Clip ROI against the real 640x480 image boundary.
    wire [9:0] e962_cx2 = (e962_center_x <= 10'd160) ? 10'd0 :
        (e962_center_x >= 10'd480) ? 10'd320 :
        e962_center_x - 10'd160;
    wire [9:0] e962_cx4 = (e962_center_x <= 10'd80) ? 10'd0 :
        (e962_center_x >= 10'd560) ? 10'd480 :
        e962_center_x - 10'd80;
    wire [8:0] e962_cy2 = (e962_center_y <= 9'd120) ? 9'd0 :
        (e962_center_y >= 9'd360) ? 9'd240 :
        e962_center_y - 9'd120;
    wire [8:0] e962_cy4 = (e962_center_y <= 9'd60) ? 9'd0 :
        (e962_center_y >= 9'd420) ? 9'd360 :
        e962_center_y - 9'd60;
    assign full_zoom_level = e962_picture_control ? e962_level : 2'd0;
    assign full_zoom_origin_x = (full_zoom_level==2'd1) ? e962_cx2 :
                                (full_zoom_level==2'd2) ? e962_cx4 : 10'd0;
    assign full_zoom_origin_y = (full_zoom_level==2'd1) ? e962_cy2 :
                                (full_zoom_level==2'd2) ? e962_cy4 : 9'd0;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            e962_level <= 2'd0;
            e962_center_x <= 10'd320;
            e962_center_y <= 9'd240;
        end else if(!e962_picture_control ||
                    e962_zoom_page_loading ||
                    // When an actual page key is pressed with MINI hidden,
                    // revert to 1x BEFORE the SD card commits the next BMP.
                    (!e950_mini_active && (e952_b1 || e952_b2))) begin
            e962_level <= 2'd0;
            e962_center_x <= 10'd320;
            e962_center_y <= 9'd240;
        end else begin
            // EC11 clockwise: 1x -> 2x -> 4x; counter-clockwise: reverse.
            if(enc_cw && !e978_brightness_chord &&
               e962_level<2'd2)
                e962_level <= e962_level + 2'd1;
            else if(enc_ccw && !e978_brightness_chord &&
                    e962_level>2'd0)
                e962_level <= e962_level - 2'd1;
            if(enc_button || e978_center_tap_release) begin
                // EC11 press or short physical K3 tap: 1x zoom reset.
                // A hold+rotate modifies BRIGHTNESS without zoom reset.
                e962_level <= 2'd0;
                e962_center_x <= 10'd320;
                e962_center_y <= 9'd240;
            end else begin
                // E963: corrected from the physical board test.
                // Physical LEFT=PCB K2, RIGHT=PCB K4;
                // physical UP=PCB K1, DOWN=PCB K5.
                // Fix only zoom interpretation: full-screen editor
                // already uses UP/DOWN for track and LEFT/RIGHT for
                // timeline, so its working controls remain unchanged.
                if(buttons[1] && e962_center_x>=10'd16)
                    e962_center_x <= e962_center_x - 10'd16;
                else if(buttons[3] && e962_center_x<=10'd623)
                    e962_center_x <= e962_center_x + 10'd16;
                if(buttons[0] && e962_center_y>=9'd16)
                    e962_center_y <= e962_center_y - 9'd16;
                else if(buttons[4] && e962_center_y<=9'd463)
                    e962_center_y <= e962_center_y + 9'd16;
            end
        end
    end

    reg [3:0] track_volume[0:3]; // 00..15, actual audible gain
    // E964: 0=钢琴 PCM, 1=方波, 2=小提琴 sustain PCM.
    // One 2-bit selection per track, NOT a separate new voice engine.
    reg [1:0] track_instrument[0:3];
    reg track_mute[0:3];
    // Shared across both modes: SW4 never silently unmutes tracks.
    assign track_mute_mask =
       {track_mute[3],track_mute[2],track_mute[1],track_mute[0]};
    // 0=BPM, 1=volume, 2=instrument, 3=mute, 4=loop bars
    reg [2:0] enc_param;
    reg [1:0] bars_sel; // 0=1 bar, 1=2 bars, 2=4 bars
    wire [10:0] loop_ticks = bars_sel==2'd0 ? 11'd384 :
                             bars_sel==2'd1 ? 11'd768 : 11'd1536;
    wire [4:0] ticks_per_col = bars_sel==2'd0 ? 5'd6 :
                               bars_sel==2'd1 ? 5'd12 : 5'd24;
    reg [5:0] edit_col;
    reg [4:0] edit_pitch;
    // E952: remove the costly legacy long-K3 fine-edit mode.
    // The BMP mini replaces it with one-key step insert/delete.
    // All full-editor live performance and recording remain intact.
    wire fine_mode = 1'b0;
    reg fine_selected;
    reg [7:0] fine_addr;
    reg [30:0] fine_note;
    reg delete_confirm;

    // Expansion K3 remains the original keyboard REC toggle.
    // No 24-bit long-press timer; onboard KEY4 independently arms MIC.
    wire k3_short = buttons[2];
    reg rec_armed, rec_active;
    reg was_running;
    // P5.0 true PAUSE: keep musical position when SW1=0; if the
    // loop-size parameter changes while paused, reset the cursor
    // once to prevent an out-of-range old tick position.
    reg [1:0] e5_bars_last;
    reg [11:0] divider_125us;
    reg [19:0] fractional_tick;
    reg [10:0] tick_pos;
    reg [5:0] play_col;
    reg [4:0] col_subtick;
    reg [4:0] quant_phase; // 0..23, 16th note @ 96 ticks/beat
    reg [6:0] beat_phase;  // 0..95, exactly shares loop clock
    reg [1:0] beat_number; // 0..3, one strong click per measure
    reg tick_pulse, wrap_pulse;
    wire [19:0] tick_increment = ({10'd0,bpm_ff2}<<6) +
                                 ({10'd0,bpm_ff2}<<5); // bpm * 96
    wire [19:0] tick_sum = fractional_tick + tick_increment;

    assign tempo_up = editor_on && (enc_param == 3'd0) && enc_cw;
    assign tempo_down = editor_on && (enc_param == 3'd0) && enc_ccw;

    // One clock: four-track playback, recorder, 16th-note grid, clicks, LED.
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            rec_active<=1'b0;
            was_running<=1'b0;
            e5_bars_last<=2'd1;
            divider_125us<=12'd0;
            fractional_tick<=20'd0;
            tick_pos<=11'd0;
            play_col<=6'd0;
            col_subtick<=5'd0;
            quant_phase<=5'd0;
            beat_phase<=7'd0;
            beat_number<=2'd0;
            tick_pulse<=1'b0;
            wrap_pulse<=1'b0;
            metro_strong_toggle<=1'b0;
            metro_weak_toggle<=1'b0;
        end else begin
            was_running<=running;
            e5_bars_last<=bars_sel;
            tick_pulse<=1'b0;
            wrap_pulse<=1'b0;
            if(!running) begin
                // True pause: hold tick, beat phase and playhead. Only
                // the sub-125us divider stops. NoteOff is drained by
                // e958_flush_pending before this branch is entered.
                rec_active<=1'b0;
                divider_125us<=12'd0;
                if(bars_sel != e5_bars_last) begin
                    // New 1/2/4 bar loop: restart at a valid beat.
                    fractional_tick<=20'd0;
                    tick_pos<=11'd0;
                    play_col<=6'd0;
                    col_subtick<=5'd0;
                    quant_phase<=5'd0;
                    beat_phase<=7'd0;
                    beat_number<=2'd0;
                end
            end else if(!was_running) begin
                rec_active<=editor_on ? rec_armed : 1'b0;
                divider_125us<=12'd0;
                // Resume at the preserved tick rather than restarting.
                // Beat-one click fires only at an actual beat boundary.
                if(beat_phase==7'd0 && quant_phase==5'd0)
                    metro_strong_toggle<=~metro_strong_toggle;
            end else if(divider_125us == 12'd3124) begin
                divider_125us<=12'd0;
                if(tick_sum >= 20'd480000) begin
                    tick_pulse<=1'b1;
                    fractional_tick<=tick_sum - 20'd480000;
                    if(quant_phase==5'd23) quant_phase<=5'd0;
                    else quant_phase<=quant_phase+5'd1;
                    if(beat_phase==7'd95) begin
                        beat_phase<=7'd0;
                        if(beat_number==2'd3) begin
                            beat_number<=2'd0;
                            metro_strong_toggle<=~metro_strong_toggle;
                        end else begin
                            beat_number<=beat_number+2'd1;
                            metro_weak_toggle<=~metro_weak_toggle;
                        end
                    end else beat_phase<=beat_phase+7'd1;
                    if(tick_pos == loop_ticks - 11'd1) begin
                        tick_pos<=11'd0;
                        play_col<=6'd0;
                        col_subtick<=5'd0;
                        wrap_pulse<=1'b1;
                        // E950: no deferred record switching at loop
                        // boundary. The K3 key controls it immediately.
                    end else begin
                        tick_pos<=tick_pos+11'd1;
                        if(col_subtick == ticks_per_col - 5'd1) begin
                            col_subtick<=5'd0;
                            play_col<=play_col+6'd1;
                        end else col_subtick<=col_subtick+5'd1;
                    end
                end else fractional_tick<=tick_sum;
            end else divider_125us<=divider_125us+12'd1;
            // Immediate record ON/OFF. rec_armed is changed by the
            // existing K3-short handler in its own sequential block,
            // so the NOT of its PREVIOUS value is the new desired state.
            if (!editor_on || !play_ff[1])
                rec_active <= 1'b0;
            else if (e958_transport_play && k3_short && !fine_mode)
                rec_active <= ~rec_armed;
        end
    end

    integer track_reset_i;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            track_sel<=2'd0;
            e952_mic_rec_enabled<=1'b0;
            enc_param<=3'd0;
            bars_sel<=2'd1;
            edit_col<=6'd0;
            rec_armed<=1'b0;
            for(track_reset_i=0;track_reset_i<4;track_reset_i=track_reset_i+1) begin
                track_volume[track_reset_i]<=4'd9;
                track_instrument[track_reset_i]<=2'd0;
                track_mute[track_reset_i]<=1'b0;
            end
        end else begin
            if(!editor_on) begin
                rec_armed<=1'b0;
                e952_mic_rec_enabled<=1'b0; // safe: re-arm MIC on return
                enc_param<=3'd0;
                // P4.0 LIVE picture recording: no cursor, no press-to-place.
                // Board K1/K2 simply choose previous/next track on RELEASE.
                // SW3=UP retains the three onboard live piano notes.
                if (e950_mini_active) begin
                    if (e952_b1_release)
                        track_sel <= track_sel - 2'd1;
                    else if (e952_b2_release)
                        track_sel <= track_sel + 2'd1;
                end
            end else begin
                // P5.0 independent KBD REC (expansion K3), MIC REC
                // (board KEY4); both only WRITE when SW1=1.  A live
                // physical note always SOUNDS even when recording off.
                if(k3_short)
                    rec_armed<=~rec_armed;
                // SW3=UP owns onboard K4 for real-time E4 piano.
                // Only SW3=DOWN lets KEY4 toggle MIC REC.
                if(e952_b4_press && !e952_sw3_ff[1])
                    e952_mic_rec_enabled<=~e952_mic_rec_enabled;
                // P5.0 consistent navigation:
                // Physical expansion up/down (K1/K5) OR onboard
                // KEY1/KEY2 always select previous/next track.
                // When SW3=1 the onboard keys are C4/D4, not navigation.
                // A note's original track is captured at NoteOn, so
                // changing track mid-play cannot rewrite held notes.
                if(!fine_mode || !fine_selected) begin
                    if(buttons[0] ||
                       (!e952_sw3_ff[1] && e952_b1_release))
                        track_sel<=track_sel-2'd1;
                    else if(buttons[4] ||
                            (!e952_sw3_ff[1] && e952_b2_release))
                        track_sel<=track_sel+2'd1;
                end
                // LEFT/RIGHT always navigate the stopped time cursor.
                // Selecting 音色 no longer silently changes this.
                if(!running && (!fine_mode || !fine_selected)) begin
                    if(buttons[1]) edit_col<=edit_col-6'd1;
                    else if(buttons[3]) edit_col<=edit_col+6'd1;
                end
                // EC11 press cycles BPM/volume/instrument/mute/loop;
                // only EC11 rotation edits parameter values.
                if(enc_button && !(fine_mode && fine_selected)) begin
                    if(enc_param==3'd4) enc_param<=3'd0;
                    else enc_param<=enc_param+3'd1;
                end else if(enc_cw || enc_ccw) begin
                    case(enc_param)
                        3'd1: begin
                            if(enc_cw && track_volume[track_sel]<4'd15)
                                track_volume[track_sel]<=track_volume[track_sel]+4'd1;
                            else if(enc_ccw && track_volume[track_sel]>4'd0)
                                track_volume[track_sel]<=track_volume[track_sel]-4'd1;
                        end
                        3'd2: begin
                            // CW increases (piano->square->violin);
                            // CCW decreases. Existing encoder A/B
                            // correction remains unchanged.
                            if(enc_cw)
                                track_instrument[track_sel] <=
                                    (track_instrument[track_sel]==2'd2) ?
                                        2'd0 : track_instrument[track_sel]+2'd1;
                            else if(enc_ccw)
                                track_instrument[track_sel] <=
                                    (track_instrument[track_sel]==2'd0) ?
                                        2'd2 : track_instrument[track_sel]-2'd1;
                        end
                        3'd3: begin
                            if(enc_cw) track_mute[track_sel]<=1'b1;
                            else if(enc_ccw) track_mute[track_sel]<=1'b0;
                        end
                        3'd4: if(!running) begin
                            if(enc_cw) bars_sel <= bars_sel==2'd2 ? 2'd0 : bars_sel+2'd1;
                            else bars_sel <= bars_sel==2'd0 ? 2'd2 : bars_sel-2'd1;
                        end
                        default: ; // 0=BPM is applied at top / 50 MHz
                    endcase
                end
            end
        end
    end

    // The recorder keeps RAW held duration, but snaps start near 1/16 lines.
    // +/-3 ticks / 24-tick 16th grid. No quantization of Note Off.
    // A shared write port updates three mirrored RAMs for recording and edits.
    function [10:0] snap_start_16;
        input [10:0] raw_start;
        input [4:0] phase16;
        input [10:0] loop_total;
        reg [10:0] displacement;
        begin
            snap_start_16=raw_start;
            if(phase16<=5'd3) begin
                if(raw_start>= {6'd0,phase16})
                    snap_start_16=raw_start-{6'd0,phase16};
                else snap_start_16=loop_total+raw_start-{6'd0,phase16};
            end else if(phase16>=5'd21) begin
                displacement=11'd24-{6'd0,phase16};
                if(raw_start+displacement>=loop_total)
                    snap_start_16=raw_start+displacement-loop_total;
                else snap_start_16=raw_start+displacement;
            end
        end
    endfunction

    reg [24:0] key_seen;
    reg [24:0] held_wrapped;
    reg [10:0] held_start[0:24];
    reg [10:0] held_snapped[0:24];
    reg [1:0] held_track[0:24];
    reg [4:0] key_idx;

    // P3.5: microphone note -> VIRTUAL keyboard press in the existing
    // 25-note recorder. No additional note RAM, MIDI FIFO, or write port.
    // This mode operates ONLY while the existing K3/record + SW2/loop
    // recording is active. Physical keyboard input remains available.
    localparam E947_MIC_TO_TRACK = 1'b1;
    // The visual pitch note is held for ~100ms to avoid '--' flicker.
    // Recording releases a key earlier: after ~50ms of missing AMDF
    // confidence. This preserves separate repeated piano strikes better.
    localparam [2:0] E947_RECORD_HOLD_MIN = 3'd3;

    // e946_note_shown={octave_index[1:0],semitone[3:0]}:
    //   octave_index=1 -> C4..B4, index 0..11
    //   octave_index=2 -> C5..B5, index 12..23
    //   octave_index=3, semitone=0 -> C6, index 24
    // Other notes (e.g. C3..B3) cannot fit the existing 25-key editor;
    // refuse them rather than storing an incorrect / wrapped pitch.
    wire [1:0] e947_octave = e946_note_shown[5:4];
    wire [3:0] e947_semitone = e946_note_shown[3:0];
    wire e947_mic_supported =
        (((e947_octave==2'd1) || (e947_octave==2'd2)) &&
          (e947_semitone < 4'd12)) ||
        ((e947_octave==2'd3) && (e947_semitone==4'd0));
    wire [4:0] e947_mic_key_index =
        (e947_octave==2'd1) ? {1'b0,e947_semitone} :
        (e947_octave==2'd2) ? (5'd12+{1'b0,e947_semitone}) :
                              5'd24;

    wire e947_mic_record_active =
        E947_MIC_TO_TRACK && e952_mic_rec_enabled &&
        editor_on && e958_transport_play && e946_note_visible &&
        (e946_note_hold >= E947_RECORD_HOLD_MIN) &&
        e947_mic_supported;
    // Gate the virtual recorder input, NOT the sound synthesizer.
    // Turning REC off releases any captured notes through the existing
    // note-off path. Physical keys keep sounding when REC is off.
    // One physical 5x5 key-down feeds BOTH real-time sound and recording.
    // In picture mini, recording is armed simply by opening the panel,
    // without moving a time cursor or pressing a second REC button.
    wire e947_record_key_now = e958_transport_play &&
        ((((editor_on && rec_active) || e950_mini_active) &&
           key_down[key_idx]) ||
         (e947_mic_record_active && (key_idx==e947_mic_key_index)));

    assign e958_flush_pending = (|held_valid) || event_pending;
    reg event_is_manual;
    reg [1:0] event_track;
    reg [4:0] event_pitch;
    reg [10:0] event_start,event_len;
    reg [5:0] alloc_slot;
    reg memory_full;
    wire [7:0] new_note_addr={event_track,alloc_slot};
    wire [10:0] key_start = held_start[key_idx];
    // E953: single 12-bit subtract, one conditional loop correction.
    // Keeps the prior wrap behavior and clamping in finalized_len.
    wire [11:0] e953_key_delta = {1'b0,tick_pos} - {1'b0,key_start};
    wire [11:0] key_elapsed = e953_key_delta[11] ?
        (e953_key_delta + {1'b0,loop_ticks}) : e953_key_delta;
    wire [10:0] finalized_len =
           (held_wrapped[key_idx] && tick_pos>=key_start) ? loop_ticks-11'd1 :
           (key_elapsed==12'd0) ? 11'd1 :
           (key_elapsed >= {1'b0,loop_ticks}) ? loop_ticks-11'd1 :
           key_elapsed[10:0];

    // Cursor converts 64 display columns back to 96-Tick musical time.
    wire [10:0] cursor_tick = bars_sel==2'd0 ?
             ({5'd0,edit_col}<<2)+({5'd0,edit_col}<<1) :
             bars_sel==2'd1 ?
             ({5'd0,edit_col}<<3)+({5'd0,edit_col}<<2) :
             ({5'd0,edit_col}<<4)+({5'd0,edit_col}<<3);

    wire [30:0] manual_word={event_pitch,event_start,event_len,4'd12};
    // Single registered-write port to all 3 copies, BRAM-friendly.
    reg mem_write_en;
    reg [7:0] mem_write_addr;
    reg [30:0] mem_write_data;
    integer kinit;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            mem_write_en=1'b0;
            mem_write_addr=8'd0;
            mem_write_data=31'd0;
            note_valid<=256'd0;
            key_seen<=25'd0;
            held_valid<=25'd0;
            held_wrapped<=25'd0;
            key_idx<=5'd0;
            event_pending<=1'b0;
            event_is_manual<=1'b0;
            event_track<=2'd0;
            event_pitch<=5'd0;
            event_start<=11'd0;
            event_len<=11'd1;
            alloc_slot<=6'd0;
            memory_full<=1'b0;
            edit_pitch<=5'd0;
            fine_selected<=1'b0;
            fine_addr<=8'd0;
            fine_note<=31'd0;
            delete_confirm<=1'b0;
            for(kinit=0;kinit<25;kinit=kinit+1) begin
                held_start[kinit]<=11'd0;
                held_snapped[kinit]<=11'd0;
                held_track[kinit]<=2'd0;
            end
        end else begin
            mem_write_en=1'b0;
            mem_write_addr=8'd0;
            mem_write_data=31'd0;
            if(wrap_pulse) held_wrapped<=held_wrapped | held_valid;
            if(!running) held_valid<=25'd0;
            if(!editor_on && !e950_mini_active && !e958_flush_pending) begin
                // Mini hidden: record no NEW keys. A formerly held note
                // finishes through the shared Note Off/allocator path.
                key_seen<=key_down;
                event_pending<=1'b0;
                fine_selected<=1'b0;
                delete_confirm<=1'b0;
            end else if(!fine_mode && event_pending) begin
                if(!note_valid[new_note_addr]) begin
                    mem_write_en=1'b1;
                    mem_write_addr=new_note_addr;
                    mem_write_data=manual_word;
                    note_valid[new_note_addr]<=1'b1;
                    event_pending<=1'b0;
                    memory_full<=1'b0;
                    event_is_manual<=1'b0;
                end else if(alloc_slot==6'd63) begin
                    memory_full<=1'b1;
                    event_pending<=1'b0;
                    event_is_manual<=1'b0;
                end else alloc_slot<=alloc_slot+6'd1;
            end else begin
                // Live playback / free real-time recording.
                fine_selected<=1'b0;
                delete_confirm<=1'b0;
                if(e947_record_key_now != key_seen[key_idx]) begin
                    key_seen[key_idx]<=e947_record_key_now;
                    if(e947_record_key_now) begin
                        // e947_record_key_now already checks KBD REC or
                        // MIC REC separately. Do not require rec_active
                        // here or mic-only capture would never open.
                        if(running) begin
                            held_valid[key_idx]<=1'b1;
                            held_start[key_idx]<=tick_pos;
                            held_snapped[key_idx]<=snap_start_16(tick_pos,quant_phase,loop_ticks);
                            held_track[key_idx]<=track_sel;
                            held_wrapped[key_idx]<=1'b0;
                        end
                    end else if(held_valid[key_idx]) begin
                        held_valid[key_idx]<=1'b0;
                        event_pending<=1'b1;
                        event_is_manual<=1'b0;
                        event_track<=held_track[key_idx];
                        event_pitch<=key_idx;
                        event_start<=held_snapped[key_idx];
                        event_len<=finalized_len;
                        alloc_slot<=6'd0;
                    end
                end
                key_idx <= (key_idx==5'd24) ? 5'd0 : key_idx+5'd1;
            end
            if(mem_write_en) begin
                note_ram_audio[mem_write_addr]<=mem_write_data;
                note_ram_view[mem_write_addr]<=mem_write_data;
            end
        end
    end

    // A single pass at every 48-kHz audio sample: 25 live-pitch seeds,
    // 256 stored notes, and 25 oscillator updates. About 307 25MHz clocks;
    // significantly below the ~521 clocks between 48-kHz samples.
    reg [6:0] level_target[0:24];
    genvar e93_pg;
    generate for (e93_pg=0;e93_pg<25;e93_pg=e93_pg+1) begin: e93_pitch_bus
        assign e93_pitch_gain[e93_pg*7 +: 7] = level_target[e93_pg];
    end endgenerate
    reg [8:0] level_env[0:24];
    reg [15:0] phase[0:24];
    reg signed [15:0] synth_acc;
    reg signed [15:0] synth_sample;
    localparam [2:0] A_IDLE=3'd0, A_SEED=3'd1, A_PRIME=3'd2,
                     A_SCAN=3'd3, A_OSC=3'd4;
    reg [2:0] audio_state;
    reg [7:0] audio_idx;
    reg [4:0] synth_idx;
    wire live_this_pitch = key_down[audio_idx[4:0]] ||
                       ((audio_idx==8'd0) && old_keys[0]) ||
                       ((audio_idx==8'd2) && old_keys[1]) ||
                       ((audio_idx==8'd4) && old_keys[2]);
    wire [4:0] note_pitch=audio_q[30:26];
    wire [10:0] note_start=audio_q[25:15];
    wire [10:0] note_length=audio_q[14:4];
    wire [3:0] note_vel=audio_q[3:0];
    wire [1:0] note_track=audio_idx[7:6];
    // E953: full musical loop wrap using a single 12-bit subtract
    // and conditional correction, preserving note-off event timing.
    wire [11:0] e953_note_delta =
        {1'b0,tick_pos} - {1'b0,note_start};
    wire [11:0] note_age = e953_note_delta[11] ?
        (e953_note_delta + {1'b0,loop_ticks}) : e953_note_delta;
    // Musical/tapered gain replaces old volume*3+16 (almost inaudible difference).
    // Instrument IDs are reserved, all currently share the square-wave osc.
    function [6:0] gain_from_volume;
        input [3:0] volume;
        begin
            case(volume)
                4'd0:gain_from_volume=7'd0;
                4'd1:gain_from_volume=7'd1;
                4'd2:gain_from_volume=7'd1;
                4'd3:gain_from_volume=7'd2;
                4'd4:gain_from_volume=7'd4;
                4'd5:gain_from_volume=7'd6;
                4'd6:gain_from_volume=7'd8;
                4'd7:gain_from_volume=7'd11;
                4'd8:gain_from_volume=7'd15;
                4'd9:gain_from_volume=7'd20;
                4'd10:gain_from_volume=7'd26;
                4'd11:gain_from_volume=7'd32;
                4'd12:gain_from_volume=7'd40;
                4'd13:gain_from_volume=7'd49;
                4'd14:gain_from_volume=7'd59;
                default:gain_from_volume=7'd70;
            endcase
        end
    endfunction
    // P1 NOTE EVENT source: real keyboard edges and 256 stored note instances.
    // Stored note ID = {1'b0,slot[7:0]} (slot contains track in bits 7:6).
    // Live note ID = 256 + pitch; track is latched on key press.
    // Each event is 30 bits, transported through a tiny 8-entry FIFO.
    // PCM voices use ON only (full natural tail). Square uses continuous
    // per-pitch gates/envelopes; both share the same track/instrument decision.
    reg [24:0] live_seen_audio;
    reg [1:0] live_owner_track [0:24];
    reg       live_owner_square [0:24];
    reg       live_owner_violin [0:24];
    // E966: Three stored sound-event status bits packed in ONE 256x3
    // synchronous-read BRAM (gate/square/violin). Replaces 768 flip-flops
    // and three 256:1 async read muxes without changing note timing.
    // The RAM is cleared in 256 clocks before the first audio scan.
    // Layout saving will depend on TD BRAM inference, not guaranteed.
    (* ram_style = "block" *)
    reg [2:0] e966_note_state_ram [0:255];
    reg [2:0] e966_note_state_q;
    reg [7:0] e966_state_clear_addr;
    reg e966_state_clear_busy;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            e966_state_clear_addr <= 8'd0;
            e966_state_clear_busy <= 1'b1;
        end else if(e966_state_clear_busy) begin
            e966_state_clear_addr <= e966_state_clear_addr + 8'd1;
            if(e966_state_clear_addr==8'd255)
                e966_state_clear_busy <= 1'b0;
        end
    end
    always @(posedge clk) begin
        // audio_addr is already pipelined one index ahead of audio_idx.
        // RAM data and the original note_ram_audio share read latency.
        e966_note_state_q <= e966_note_state_ram[audio_addr];
        if(e966_state_clear_busy)
            e966_note_state_ram[e966_state_clear_addr] <= 3'b000;
        else if(audio_state==A_SCAN &&
                (!stored_event_change || event_push))
            e966_note_state_ram[audio_idx] <= {
                (stored_note_on ? track_instrument[note_track] :
                                  e966_note_state_q[2:1]),
                stored_note_on
            };
    end

    wire live_new = live_this_pitch && !live_seen_audio[audio_idx[4:0]];
    wire live_end = !live_this_pitch && live_seen_audio[audio_idx[4:0]];
    wire live_edge = live_new || live_end;
    wire [1:0] live_event_track = live_new ?
                 ((editor_on || e950_mini_active) ? track_sel : 2'd0) :
                 live_owner_track[audio_idx[4:0]];
    // P5.0: SW1 no longer chooses piano/square by accident!
    // Direct physical keyboard sound is independent of SW1 PLAY.
    // In editor/mini it uses the selected track's instrument;
    // when the picture mini is closed, use track0 (default Piano).
    // The instrument is latched on NoteOn; NoteOff uses old owner.
    wire [1:0] live_event_inst = live_new ?
                 ((editor_on || e950_mini_active) ?
                    track_instrument[track_sel] : track_instrument[0]) :
                 {live_owner_violin[audio_idx[4:0]],
                  live_owner_square[audio_idx[4:0]]};
    wire live_event_square = (live_event_inst==2'd1);
    // Event gain reflects track volume, not mute: a piano Note-On while
    // muted must still allocate its Voice so it can be heard when unmuted.
    wire [6:0] live_event_gain = !editor_on ? 7'd55 :
                 gain_from_volume(track_volume[live_event_track]);
    wire [29:0] live_event_word = {
        live_new, live_event_inst, live_event_track,
        (9'd256 + {4'd0,audio_idx[4:0]}),
        audio_idx[4:0],4'd12,live_event_gain
    };

    // P1.1: The Note's musical lifecycle MUST NOT depend on track mute.
    // Muting is a mixer-only decision. A held piano note keeps advancing its
    // original PCM sample; unmuting does not create a second Note-On.
    // Square amplitude is separately masked in A_SCAN (and A_SEED).
    // In picture mode record onto the same RAM, but DO NOT start
    // 4-track PCM autoplay from SDRAM behind the BMP reader. Live
    // physical piano keys still sound via A_SEED.
    wire stored_note_on = e958_transport_play && note_valid[audio_idx] &&
             note_length != 11'd0 && note_pitch < 5'd25 &&
             note_age < {1'b0,note_length};
    wire [1:0] stored_last_inst = e966_note_state_q[2:1];
    wire stored_event_change =
          (stored_note_on != e966_note_state_q[0]) ||
          (stored_note_on &&
           (track_instrument[note_track] != stored_last_inst));
    wire [29:0] stored_event_word = {
        stored_note_on,
        (stored_note_on ? track_instrument[note_track] : stored_last_inst),
        note_track,{1'b0,audio_idx},note_pitch,note_vel,
        gain_from_volume(track_volume[note_track])
    };

    wire live_event_emit = (audio_state == A_SEED) && live_edge;
    wire stored_event_emit = (audio_state == A_SCAN) && stored_event_change;
    wire event_emit = live_event_emit || stored_event_emit;
    wire [29:0] event_new_data = live_event_emit ?
                                      live_event_word : stored_event_word;
    // An 8-deep synchronous-write FIFO is enough for normal ≤4-voice usage.
    // On saturation an edge is retried on the NEXT audio scan, never silently
    // marked as already delivered. It adds latency, not a stuck note.
    reg [29:0] event_fifo [0:7];
    reg [3:0] event_wp,event_rp;
    reg event_overflow_sticky;
    assign note_evt_valid = (event_wp != event_rp);
    assign note_evt_data = event_fifo[event_rp[2:0]];
    wire event_pop = note_evt_valid && note_evt_ready;
    wire event_full = (event_wp[2:0] == event_rp[2:0]) &&
                      (event_wp[3] != event_rp[3]);
    wire event_push = event_emit && (!event_full || event_pop);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            event_wp <= 4'd0;
            event_rp <= 4'd0;
            event_overflow_sticky <= 1'b0;
        end else begin
            if(event_push) begin
                event_fifo[event_wp[2:0]] <= event_new_data;
                event_wp <= event_wp + 4'd1;
            end
            if(event_pop) event_rp <= event_rp + 4'd1;
            if(event_emit && !event_push) event_overflow_sticky <= 1'b1;
        end
    end

    wire [7:0] note_strength={1'b0,gain_from_volume(track_volume[note_track])};
    wire [7:0] sum_amp = {1'b0,level_target[note_pitch]} + note_strength;
    // The rotating 25-stage oscillator queue avoids 25 parallel adders.
    wire [15:0] phase_current=phase[0];
    wire [8:0] env_current=level_env[0];
    wire [8:0] env_goal={level_target[synth_idx],2'b00};
    wire [8:0] env_after_attack = (env_goal-env_current <= 9'd2) ? env_goal : env_current+9'd2;
    wire [8:0] env_after_release = (env_current-env_goal <= 9'd1) ? env_goal : env_current-9'd1;
    wire signed [9:0] synth_term = phase_current[15] ?
              -$signed({2'b00,env_current[8:2]}) :
               $signed({2'b00,env_current[8:2]});

    function [15:0] phase_step;
        input [4:0] pitch;
        begin
            case(pitch)
                5'd0:phase_step=16'd357;  5'd1:phase_step=16'd378;
                5'd2:phase_step=16'd401;  5'd3:phase_step=16'd425;
                5'd4:phase_step=16'd450;  5'd5:phase_step=16'd477;
                5'd6:phase_step=16'd505;  5'd7:phase_step=16'd535;
                5'd8:phase_step=16'd567;  5'd9:phase_step=16'd601;
                5'd10:phase_step=16'd636;5'd11:phase_step=16'd674;
                5'd12:phase_step=16'd714;5'd13:phase_step=16'd757;
                5'd14:phase_step=16'd802;5'd15:phase_step=16'd850;
                5'd16:phase_step=16'd900;5'd17:phase_step=16'd954;
                5'd18:phase_step=16'd1010;5'd19:phase_step=16'd1070;
                5'd20:phase_step=16'd1134;5'd21:phase_step=16'd1201;
                5'd22:phase_step=16'd1273;5'd23:phase_step=16'd1349;
                5'd24:phase_step=16'd1429;
                default:phase_step=16'd0;
            endcase
        end
    endfunction
    integer ainit,rot;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            audio_state<=A_IDLE;
            audio_idx<=8'd0;
            audio_addr<=8'd0;
            synth_idx<=5'd0;
            synth_acc<=16'sd0;
            synth_sample<=16'sd0;
            live_seen_audio<=25'd0;
            // Event history RAM cleared by e966_state_clear_busy;
            // never use async reset on distributed FFs.
            for(ainit=0;ainit<25;ainit=ainit+1) begin
                level_target[ainit]<=7'd0;
                level_env[ainit]<=9'd0;
                phase[ainit]<=16'd0;
                live_owner_track[ainit]<=2'd0;
                live_owner_square[ainit]<=1'b0;
                live_owner_violin[ainit]<=1'b0;
            end
        end else begin
            case(audio_state)
                A_IDLE: if(audio_valid && !e966_state_clear_busy) begin
                    audio_state<=A_SEED;
                    audio_idx<=8'd0;
                end
                A_SEED: begin
                    // Live notes belong to the track/instrument chosen on
                    // their Note-On; changing selected track cannot redirect
                    // a held key midway through its sound.
                    if (!live_edge || event_push) begin
                        live_seen_audio[audio_idx[4:0]]<=live_this_pitch;
                        if(live_new) begin
                            live_owner_track[audio_idx[4:0]]<=live_event_track;
                            live_owner_square[audio_idx[4:0]]<=
                                (live_event_inst==2'd1);
                            live_owner_violin[audio_idx[4:0]]<=
                                (live_event_inst==2'd2);
                        end
                    end
                    // Only square voices enter the 25 shared oscillators.
                    // Piano notes are delivered through the event FIFO.
                    level_target[audio_idx[4:0]] <=
                        (!live_this_pitch || !live_event_square ||
                         (editor_on && track_mute[live_event_track])) ?
                        7'd0 : live_event_gain;
                    if(audio_idx==8'd24) begin
                        audio_idx<=8'd0;
                        audio_addr<=8'd0;
                        audio_state<=A_PRIME;
                    end else audio_idx<=audio_idx+8'd1;
                end
                A_PRIME: begin
                    audio_idx<=8'd0;
                    audio_addr<=8'd1;
                    audio_state<=A_SCAN;
                end
                A_SCAN: begin
                    // E966 RAM writer above uses the identical
                    // event-success / FIFO retry predicate.
                    if(stored_note_on &&
                       track_instrument[note_track]==2'd1 &&
                       !track_mute[note_track]) begin
                        if(sum_amp >= 8'd70) level_target[note_pitch]<=7'd70;
                        else level_target[note_pitch]<=sum_amp[6:0];
                    end
                    if(audio_idx==8'd255) begin
                        audio_state<=A_OSC;
                        synth_idx<=5'd0;
                        synth_acc<=16'sd0;
                    end else begin
                        audio_idx<=audio_idx+8'd1;
                        audio_addr<=audio_addr+8'd1;
                    end
                end
                A_OSC: begin
                    for(rot=0;rot<24;rot=rot+1) begin
                        phase[rot]<=phase[rot+1];
                        level_env[rot]<=level_env[rot+1];
                    end
                    phase[24]<=phase_current+phase_step(synth_idx);
                    if(env_current<env_goal) level_env[24]<=env_after_attack;
                    else if(env_current>env_goal) level_env[24]<=env_after_release;
                    else level_env[24]<=env_current;
                    if(synth_idx==5'd24) begin
                        synth_sample<=synth_acc+synth_term;
                        audio_state<=A_IDLE;
                    end else begin
                        synth_acc<=synth_acc+synth_term;
                        synth_idx<=synth_idx+5'd1;
                    end
                end
                default:audio_state<=A_IDLE;
            endcase
        end
    end
    wire signed [23:0] old_synth_pcm =
        $signed({{8{synth_sample[15]}},synth_sample}) <<< 12;
    // PCM and Square may now be active on different tracks simultaneously.
    // The old SW1 global A/B audio selection has been retired; track_instrument
    // controls each part. Single signed 26-bit sum allows two instruments plus
    // original click/metronome, with one final 24-bit saturation.
    wire signed [23:0] square_pcm24 = old_synth_pcm;
    wire signed [23:0] piano_pcm24 = e93_pcm_loaded ? e93_pcm_audio : 24'd0;
    // P3.3: one common 26-bit instrument sum for both stereo lanes.
    // Associative addition retains the EXACT 26-bit signed clip input.
    wire signed [25:0] instrument_sum26 =
        $signed({{2{square_pcm24[23]}},square_pcm24}) +
        $signed({{2{piano_pcm24[23]}},piano_pcm24});
    wire signed [25:0] mix_left26 =
        $signed({{2{audio_left_in[23]}},audio_left_in}) +
        instrument_sum26;
    wire signed [25:0] mix_right26 =
        $signed({{2{audio_right_in[23]}},audio_right_in}) +
        instrument_sum26;
    function [23:0] clip24;
        input signed [25:0] x;
        begin
            if(x > 26'sd8388607) clip24=24'h7fffff;
            else if(x < -26'sd8388608) clip24=24'h800000;
            else clip24=x[23:0];
        end
    endfunction
    // The current I2S source generates identical left/right samples.
    // Setting E945_SHARE_MONO_I2S=1 lets synthesis remove the duplicate
    // right mix/clip, but keeps the stereo formula for future expansion.
    localparam E945_SHARE_MONO_I2S = 1'b1;
    assign audio_left_out = clip24(mix_left26);
    assign audio_right_out = E945_SHARE_MONO_I2S ?
        audio_left_out : clip24(mix_right26);

    // E8.2 AREA FIX: 2 column-major, synchronous-read bitmap memories.
    // The previous implementation used 45 independently addressed 64-bit
    // register rows (2880 flip-flops plus wide indexed-write/read muxes).
    // This version computes each column in 2*256+2 clocks and commits it
    // with one write, making both arrays suitable for true dual-port BRAM.
    // The user-visible resolution remains 64 columns, four tracks x five
    // miniature pitch bands, and 25 detail pitches: NO bitmap downsampling.
    // Declare pixel coordinates before the synchronous RAM read address.
    reg [9:0] px;
    reg [8:0] py;
    reg de_prev;
    reg [19:0] overview_bitmap [0:63]; // column -> 4*5 band flags
    reg [24:0] detail_bitmap   [0:63]; // column -> 25 pitch flags
    reg [19:0] overview_acc;
    reg [24:0] detail_acc;
    reg [19:0] overview_pixel_q;
    reg [24:0] detail_pixel_q;
    reg maps_ready;

    localparam [2:0] V_IDLE=3'd0, V_SETUP=3'd1, V_FETCH=3'd2,
                     V_ACC=3'd3, V_COMMIT=3'd4;
    reg [2:0] vstate;
    reg [5:0] view_col;
    reg vs_last;

    // P3.4: video-domain presentation filter, not an audio/pitch algorithm.
    // Same 640x480 VS pulse already used by the note bitmap below.
    //   * hold last detected note for ~100ms across short AMDF dropouts;
    //   * accept a different note after two consecutive displayed frames;
    //   * gently slew the four existing VU bars by 1 level per frame.
    // No additional clock dividers, PLL, BRAM, DSP or separate time counters.
    localparam [2:0] E946_NOTE_HOLD_FRAMES = 3'd5;  // ~100ms @ 60Hz
    localparam E946_LARGE_PITCH_CARD = 1'b0; // set 0 for tiny original UI
    localparam E946_SMOOTH_VOLUME = 1'b1;     // set 0 to cut UI VU smoothing

    reg [5:0] e946_note_shown;
    reg [5:0] e946_note_pending;
    reg       e946_note_visible;
    reg       e946_note_recent;
    reg       e946_pending_once;
    reg [2:0] e946_note_hold;
    reg [2:0] e946_vu_frame;
    wire [2:0] e946_vu_draw = E946_SMOOTH_VOLUME ?
                            e946_vu_frame : mic_voice_level;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            e946_note_shown <= 6'd0;
            e946_note_pending <= 6'd0;
            e946_note_visible <= 1'b0;
            e946_note_recent <= 1'b0;
            e946_pending_once <= 1'b0;
            e946_note_hold <= 3'd0;
            e946_vu_frame <= 3'd0;
        end else if (vs_last && !vs) begin
            // Apply all changes between frames: the same text is drawn
            // for an entire 640x480 image (no mid-frame tearing).
            e946_note_recent <= mic_pitch_valid;
            if (mic_pitch_valid) begin
                e946_note_hold <= E946_NOTE_HOLD_FRAMES;
                if (!e946_note_visible) begin
                    e946_note_shown <= mic_pitch_code;
                    e946_note_visible <= 1'b1;
                    e946_pending_once <= 1'b0;
                end else if (mic_pitch_code == e946_note_shown) begin
                    e946_pending_once <= 1'b0;
                end else if (e946_pending_once &&
                             mic_pitch_code == e946_note_pending) begin
                    e946_note_shown <= mic_pitch_code;
                    e946_pending_once <= 1'b0;
                end else begin
                    e946_note_pending <= mic_pitch_code;
                    e946_pending_once <= 1'b1;
                end
            end else begin
                e946_pending_once <= 1'b0;
                if (e946_note_visible) begin
                    if (e946_note_hold != 3'd0)
                        e946_note_hold <= e946_note_hold - 3'd1;
                    else
                        e946_note_visible <= 1'b0;
                end
            end

            // Anti-flicker on the existing four-bar volume indicator.
            if (E946_SMOOTH_VOLUME) begin
                if (e946_vu_frame < mic_voice_level)
                    e946_vu_frame <= e946_vu_frame + 3'd1;
                else if (e946_vu_frame > mic_voice_level)
                    e946_vu_frame <= e946_vu_frame - 3'd1;
            end
        end
    end

    wire [10:0] column_start= bars_sel==2'd0 ?
              ({5'd0,view_col}<<2)+({5'd0,view_col}<<1) :
              bars_sel==2'd1 ?
              ({5'd0,view_col}<<3)+({5'd0,view_col}<<2) :
              ({5'd0,view_col}<<4)+({5'd0,view_col}<<3);
    wire [10:0] v_start=view_q[25:15];
    wire [10:0] v_len=view_q[14:4];
    wire [4:0] v_pitch=view_q[30:26];
    wire [2:0] v_band = (v_pitch<5'd5) ? 3'd4 :
                        (v_pitch<5'd10) ? 3'd3 :
                        (v_pitch<5'd15) ? 3'd2 :
                        (v_pitch<5'd20) ? 3'd1 : 3'd0;
    wire [4:0] v_band_address = {view_idx[7:6],2'b00} +
                                 {3'd0,view_idx[7:6]} + {2'd0,v_band};
    // E953/P3.11: one subtraction + optional wrap correction.
    // Unlike the previous forward/reverse modulo chains, a note onset
    // only needs to be tested inside the current 1/64 loop cell.
    // All values are <loop_ticks; the last cell ends AT loop_ticks,
    // so an onset never needs a second wrap-around comparator.
    wire [11:0] e953_col_delta =
        {1'b0,column_start} - {1'b0,v_start};
    wire [11:0] col_age = e953_col_delta[11] ?
        (e953_col_delta + {1'b0,loop_ticks}) : e953_col_delta;
    wire [10:0] e953_cell_end =
        column_start + {6'd0,ticks_per_col};
    wire e953_onset_in_cell =
        (v_start >= column_start) && (v_start < e953_cell_end);
    wire mask_hit = (col_age < {1'b0,v_len}) || e953_onset_in_cell;
    // One eligibility predicate serves BOTH the full editor bitmap
    // and image-mini point-edit search, using the SAME note RAM data.
    wire e956_view_note_ok =
        note_valid[view_idx] && (v_pitch<5'd25) &&
        (v_start<loop_ticks) && (v_len!=11'd0);
    wire bitmap_write=(vstate==V_COMMIT);

    // Pixel clock and the bitmap memories share the SAME 25 MHz domain.
    // Synchronous read has a one-cycle latency, so prefetch the column for
    // the *next* horizontal pixel. 8-pixel cell boundaries stay aligned.
    wire [9:0] next_px=(de && de_prev && px<10'd639) ? px+10'd1 : px;
    // One 6-bit subtract in both modes: full 8px grid starts at x104
    // (=13 cells); mini 4px grid starts at x352 (=24 modulo64).
    // Truncating to 6 bits is exact because the addresses wrap mod64.
    wire [5:0] e951_prefetch_cell =
        editor_on ? next_px[8:3] : next_px[7:2];
    wire [5:0] e951_prefetch_bias =
        editor_on ? 6'd13 : 6'd24;
    wire [5:0] prefetch_col =
        e951_prefetch_cell - e951_prefetch_bias;
    always @(posedge clk) begin
        overview_pixel_q <= overview_bitmap[prefetch_col];
        detail_pixel_q   <= detail_bitmap[prefetch_col];
        if(bitmap_write) begin
            overview_bitmap[view_col] <= overview_acc;
            detail_bitmap[view_col]   <= detail_acc;
        end
    end

    // E962 live-performance strip and sliding Note-On squares restored.
    // SW3=UP: direct 25 pitch blocks appear immediately when held.
    // A 64-frame x25-bit VIDEO BRAM drives the left-of-mini scrolling
    // performance roll. This is a quarter of the old 256-frame history,
    // and does not add 256x25 audio/edit Note RAM or a full BMP buffer.
    localparam E962_KEEP_LIVE_HISTORY = 1'b1; // 0: only pressed-key bar
    wire [24:0] e962_live_keys =
        key_down | (old_keys[0] ? 25'h000001 : 25'd0) |
                   (old_keys[1] ? 25'h000004 : 25'd0) |
                   (old_keys[2] ? 25'h000010 : 25'd0);
    reg [5:0] e962_history_wr;
    reg [6:0] e962_history_count;
    // E963: 1 sample per TWO actual HDMI VS frames; same 64x25 BRAM,
    // roughly 2 seconds of trail instead of 1 second at ~60Hz.
    reg e963_history_half;
    wire e963_history_tick =
        (vs_last && !vs) && !e963_history_half;
    (* ram_style = "block" *) reg [24:0] e962_live_history[0:63];
    reg [24:0] e962_history_q;
    wire [9:0] e962_roll_px_ahead =
        (next_px>=10'd320) ? 10'd320 : next_px;
    wire [5:0] e962_history_age =
        (10'd320 - e962_roll_px_ahead) >> 2;
    wire [5:0] e962_history_rd =
        e962_history_wr - 6'd1 - e962_history_age;
    always @(posedge clk) begin
        e962_history_q <= e962_live_history[e962_history_rd];
        if(E962_KEEP_LIVE_HISTORY && e963_history_tick &&
           e952_sw3_ff[1])
            e962_live_history[e962_history_wr] <= e962_live_keys;
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            e962_history_wr<=6'd0;
            e962_history_count<=7'd0;
            e963_history_half<=1'b0;
        end else if(!e952_sw3_ff[1]) begin
            e962_history_wr<=6'd0;
            e962_history_count<=7'd0;
            e963_history_half<=1'b0;
        end else if(vs_last && !vs) begin
            e963_history_half<=~e963_history_half;
            if(!e963_history_half) begin
                e962_history_wr<=e962_history_wr+6'd1;
                if(e962_history_count<7'd64)
                    e962_history_count<=e962_history_count+7'd1;
            end
        end
    end
    wire [4:0] e962_roll_pitch = (9'd453-py)>>2;
    wire e962_roll_hit =
        E962_KEEP_LIVE_HISTORY && e952_sw3_ff[1] &&
        !editor_on && de &&
        px>=10'd64 && px<10'd320 &&
        py>=9'd354 && py<=9'd453 &&
        // E963: draw every pixel of the note, not the previous
        // dotted/checkerboard mask, so it is a continuous solid block.
        ({1'b0,e962_history_age}<e962_history_count) &&
        e962_history_q[e962_roll_pitch];
    wire [9:0] e962_tile_dx=px-10'd96;
    wire [4:0] e962_tile_pitch=e962_tile_dx[8:4];
    wire e962_tile_pressed =
        e962_live_keys[e962_tile_pitch];

    // P4.8: video-clock snapshot of a complete real MIC DFT frame.
    // The 40-bit bus is held for >10ms in audio_mclk between toggles.
    // This is a static bundled-data CDC with 3FF toggle synchronizer.
    reg [2:0] e982_spec_toggle_sync;
    reg [39:0] e982_bars_display;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            e982_spec_toggle_sync <= 3'd0;
            e982_bars_display <= 40'd0;
        end else begin
            e982_spec_toggle_sync <=
                {e982_spec_toggle_sync[1:0],e982_mic_bands_toggle};
            if(e982_spec_toggle_sync[2] != e982_spec_toggle_sync[1])
                e982_bars_display <= e982_mic_bands;
        end
    end
    // x32..287: 8 bins at 32px per bin; 26px bar, 6px gap.
    // A 3-bit slice replaces division by a non-power-of-two constant.
    wire [2:0] e982_spec_col = px[7:5] - 3'd1;
    wire [5:0] e982_spec_bit_address =
        {1'b0,e982_spec_col,2'b00}+{3'd0,e982_spec_col};
    wire [4:0] e982_spec_height =
        e982_bars_display[e982_spec_bit_address +: 5];
    wire [8:0] e982_spec_top_y =
        9'd447-{4'd0,e982_spec_height};
    // P5.1: SW control guide ticker, one-way right-to-left forever.
    // One 8-bit phase counter replaces the old bouncing X+direction
    // controls. No new clock, timer, framebuffer, CJK ROM or BRAM.
    reg [7:0] e990_sw_scroll;
    wire e978_caption_rec =
        e958_transport_play &&
        (e950_mini_active || (editor_on && e952_mic_rec_enabled) ||
         (editor_on && rec_active));
    // 256px virtual ticker, four 64px segments. Constant origin
    // avoids an extra dynamic X offset / wide mux for the REC badge.
    // 8bit natural wrap keeps the text continuous at phase 255->0.
    wire [7:0] e990_caption_cell =
        px[7:0] - 8'd20 + e990_sw_scroll;
    always @(posedge clk or posedge rst) begin
        if(rst)
            e990_sw_scroll <= 8'd0;
        else if(vs_last && !vs && e962_picture_control)
            e990_sw_scroll <= e990_sw_scroll + 8'd1;
    end

    // Transpose note-major data into column-major bitmaps. Two passes per
    // note are not required: synchronous note RAM read is primed in V_FETCH,
    // then consumed in V_ACC, totaling 32,768 scan cycles per full frame.
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            vs_last<=1'b1;
            vstate<=V_IDLE;
            view_idx<=8'd0;
            view_col<=6'd0;
            overview_acc<=20'd0;
            detail_acc<=25'd0;
            maps_ready<=1'b0;
        end else begin
            vs_last<=vs;
            if(vs_last && !vs) begin
                vstate<=V_SETUP;
                view_col<=6'd0;
                maps_ready<=1'b0;
            end else begin
                case(vstate)
                    V_IDLE: ;
                    V_SETUP: begin
                        view_idx<=8'd0;
                        overview_acc<=20'd0;
                        detail_acc<=25'd0;
                        vstate<=V_FETCH;
                    end
                    V_FETCH: vstate<=V_ACC;
                    V_ACC: begin
                        if(e956_view_note_ok && mask_hit) begin
                            overview_acc[v_band_address]<=1'b1;
                            if(view_idx[7:6]==track_sel)
                                detail_acc[v_pitch]<=1'b1;
                        end
                        if(view_idx==8'd255)
                            vstate<=V_COMMIT;
                        else begin
                            view_idx<=view_idx+8'd1;
                            vstate<=V_FETCH;
                        end
                    end
                    V_COMMIT: begin
                        if(view_col==6'd63) begin
                            vstate<=V_IDLE;
                            maps_ready<=1'b1;
                        end else begin
                            view_col<=view_col+6'd1;
                            vstate<=V_SETUP;
                        end
                    end
                    default: vstate<=V_IDLE;
                endcase
            end
        end
    end

    // E950 low-cost mini track overview in BMP mode. 4 tracks x 5
    // real note bands, x64 time columns. All indices are bit slices,
    // shift/add only; no division, no second memory or score recompute.
    // Y=320=10*32, so lane indexing needs ONLY a 2-bit
    // subtract; no 9-bit vertical-coordinate arithmetic.
    wire [6:0] e950_mini_dy = { (py[6:5] - 2'd2), py[4:0] };
    wire [1:0] e950_mini_lane = e950_mini_dy[6:5];
    wire [2:0] e950_mini_band = e950_mini_dy[4:2];
    wire [4:0] e950_mini_bitmap_bit =
        {e950_mini_lane,2'b00} + {3'd0,e950_mini_lane} +
        {2'd0,e950_mini_band};
    wire [5:0] e950_mini_col = px[7:2] - 6'd24;
    wire e950_mini_note = maps_ready &&
        (e950_mini_band < 3'd5) && e951_band_hit;

    // HDMI OSD pixel counters; all drawing logic is behind SW4.
    always @(posedge clk or posedge rst) begin
        if(rst) begin px<=10'd0;py<=9'd0;de_prev<=1'b0; end
        else begin
            de_prev<=de;
            if(!vs) py<=9'd0;
            else if(de_prev && !de && py<9'd479) py<=py+9'd1;
            if(de) begin
                if(!de_prev) px<=10'd0;
                else if(px<10'd639) px<=px+10'd1;
            end else px<=10'd0;
        end
    end
    wire in_time=px>=10'd104 && px<10'd616;
    wire [5:0] grid_col=px[8:3] - 6'd13;
    wire [4:0] grid_row=(py-9'd250)>>3;
    wire [4:0] current_pitch=5'd24-grid_row;
    reg [1:0] lane;
    reg [8:0] lane_text_y;
    reg in_lane;
    reg [23:0] lane_color;
    always @* begin
        lane=2'd0;in_lane=1'b0;lane_text_y=9'd76;
        if(py>=9'd72 && py<9'd108) begin lane=2'd0;in_lane=1'b1;lane_text_y=9'd76;end
        else if(py>=9'd111 && py<9'd147) begin lane=2'd1;in_lane=1'b1;lane_text_y=9'd115;end
        else if(py>=9'd150 && py<9'd186) begin lane=2'd2;in_lane=1'b1;lane_text_y=9'd154;end
        else if(py>=9'd189 && py<9'd225) begin lane=2'd3;in_lane=1'b1;lane_text_y=9'd193;end
        // Full editor and mini never paint at the same time; share
        // the one 4-color RGB palette decoder across both modes.
        case(editor_on ? lane : e950_mini_lane)
            2'd0: lane_color=24'h55bdd7;
            2'd1: lane_color=24'he6a86e;
            2'd2: lane_color=24'haf8fdb;
            default: lane_color=24'h69cfaa;
        endcase
    end

    // Five stacked mini note-lines per track. Selection uses a thin border,
    // never the whole 36-pixel row as a note itself.
    wire [5:0] lane_relative = py - (lane_text_y - 9'd4);
    wire [2:0] small_band = (lane_relative < 6'd10) ? 3'd0 :
                            (lane_relative < 6'd16) ? 3'd1 :
                            (lane_relative < 6'd22) ? 3'd2 :
                            (lane_relative < 6'd28) ? 3'd3 : 3'd4;
    wire small_band_in = (lane_relative>=6'd4 && lane_relative<=6'd32 &&
                         lane_relative!=6'd9 && lane_relative!=6'd15 &&
                         lane_relative!=6'd21 && lane_relative!=6'd27);
    wire [4:0] small_band_idx = {lane,2'b00} +
                                  {3'd0,lane} + {2'd0,small_band};
    // Main editor and picture mini are mutually exclusive.
    // One shared 20:1 selection network instead of two separate
    // dynamic bit-select muxes, both fed by the same BRAM read port.
    wire [4:0] e951_band_index =
        editor_on ? small_band_idx : e950_mini_bitmap_bit;
    wire e951_band_hit = overview_pixel_q[e951_band_index];

    // ASCII 5x7 for compact numerals and MIDI octave guide, shared decoder.
    function [4:0] digit_row;
        input [4:0] num;
        input [2:0] row;
        reg [34:0] p;
        begin
            case(num)
                4'd0:p=35'b01110100011001110101110011000101110;
                4'd1:p=35'b00100011000010000100001000010001110;
                4'd2:p=35'b01110100010000100010001000100011111;
                4'd3:p=35'b11110000010000101110000010000111110;
                4'd4:p=35'b00010001100101010010111110001000010;
                4'd5:p=35'b11111100001000011110000010000111110;
                4'd6:p=35'b01110100001000011110100011000101110;
                4'd7:p=35'b11111000010001000100010000100001000;
                4'd8:p=35'b01110100011000101110100011000101110;
                4'd9:p=35'b01110100011000101111000010000101110;
                4'd10:p=35'b01111100001000010000100001000001111; // C
                4'd11:p=35'b11110100011000111110100011000111110; // B
                4'd12:p=35'b11110100011000111110100001000010000; // P
                4'd13:p=35'b10001110111010110101100011000110001; // M
                4'd14:p=35'b01110100011010110101101011000101110; // Q

                5'd15:p=35'b11110100011000110001100011000111110; // D
                5'd16:p=35'b11111100001000011110100001000011111; // E
                5'd17:p=35'b11111100001000011110100001000010000; // F
                5'd18:p=35'b01111100001000010111100011000101111; // G
                5'd19:p=35'b01110100011000111111100011000110001; // A
                5'd20:p=35'b11111001000010000100001000010011111; // I
                5'd21:p=35'b11111001000010000100001000010000100; // T
                5'd22:p=35'b01111100001000001110000010000111110; // S: SW control hint
                5'd23:p=35'b10001100011000110101101011010101010; // W: SW control hint
                default:p=35'd0;
            endcase
            case(row)
                0:digit_row=p[34:30];1:digit_row=p[29:25];
                2:digit_row=p[24:20];3:digit_row=p[19:15];
                4:digit_row=p[14:10];5:digit_row=p[9:5];
                default:digit_row=p[4:0];
            endcase
        end
    endfunction

    // Change to 1'b0 only if P3.0 placement exceeds the MSlice limit:
    // suppress the old C/D/L/R bring-up UI; pitch names and VU stay.
    localparam E942_SHOW_OLD_MIC_DEBUG = 1'b0;

    // P3.1: share the existing digit_row 5x7 decoder between the note
    // letter and the octave; do NOT instantiate a second 7-letter decoder.
    // Pixel coordinates already exist in the editor.
    reg [4:0] e942_note_char;
    always @* begin
        case (e946_note_shown[3:0])
            4'd0,4'd1:   e942_note_char=5'd10; // C
            4'd2,4'd3:   e942_note_char=5'd15; // D
            4'd4:        e942_note_char=5'd16; // E
            4'd5,4'd6:   e942_note_char=5'd17; // F
            4'd7,4'd8:   e942_note_char=5'd18; // G
            4'd9,4'd10:  e942_note_char=5'd19; // A
            default:     e942_note_char=5'd11; // B
        endcase
    end

    wire e942_has_sharp =
            (e946_note_shown[3:0] == 4'd1) ||
            (e946_note_shown[3:0] == 4'd3) ||
            (e946_note_shown[3:0] == 4'd6) ||
            (e946_note_shown[3:0] == 4'd8) ||
            (e946_note_shown[3:0] == 4'd10);

    // One 5x7 glyph lookup for both the note letter and octave.
    // A compile-time UI selection controls pixel scale and placement;
    // no second glyph ROM or redraw buffer is ever instantiated.
    wire [2:0] e942_glyph_row = E946_LARGE_PITCH_CARD ?
            ((py - 9'd31) >> 1) : (py - 9'd36);
    wire e943_oct_region = E946_LARGE_PITCH_CARD ?
            (px >= 10'd412) : (px >= 10'd391);
    wire [4:0] e943_char_mux = e943_oct_region ?
            ({3'd0,e946_note_shown[5:4]} + 5'd3) : e942_note_char;
    wire [4:0] e943_shared_glyph =
            digit_row(e943_char_mux,e942_glyph_row);
    // Low four X bits suffice within each 10-pixel scaled glyph region.
    wire [3:0] e946_big_glyph_offset = e943_oct_region ?
            (px[3:0] - 4'd12) : (px[3:0] - 4'd8);
    wire [2:0] e943_glyph_col = E946_LARGE_PITCH_CARD ?
            e946_big_glyph_offset[3:1] :
            (e943_oct_region ? (px - 10'd391) : (px - 10'd368));

    // E9.3.12 I: compact 5x7 hexadecimal digits for direct SDRAM evidence.
    function [4:0] e941_hex_row;
        input [3:0] nibble;
        input [2:0] row;
        reg [34:0] p;
        begin
            p=35'd0;
            case(nibble)
                4'hA:p=35'b01110100011000111111100011000110001;
                4'hB:p=35'b11110100011000111110100011000111110;
                4'hC:p=35'b01111100001000010000100001000001111;
                4'hD:p=35'b11110100011000110001100011000111110;
                4'hE:p=35'b11111100001000011110100001000011111;
                4'hF:p=35'b11111100001000011110100001000010000;
                default:p=35'd0;
            endcase
            if(nibble <= 4'd9)
                e941_hex_row=digit_row(nibble,row);
            else begin
                case(row)
                    3'd0:e941_hex_row=p[34:30];
                    3'd1:e941_hex_row=p[29:25];
                    3'd2:e941_hex_row=p[24:20];
                    3'd3:e941_hex_row=p[19:15];
                    3'd4:e941_hex_row=p[14:10];
                    3'd5:e941_hex_row=p[9:5];
                    default:e941_hex_row=p[4:0];
                endcase
            end
        end
    endfunction
    // Chinese glyph table generated from 16x16 CJK raster; single lookup.
    // E954 AREA HUD: render the same two 16px-wide label slots with
    // a compact 5x7 Latin abbreviation, enlarged 2x without a
    // 24-glyph * 16-row * 16-pixel CJK combinational decoder.
    // All note/tempo/track controls remain implemented as before.
    // P4.6/E971: ALL 26 ORIGINAL Chinese glyphs, exactly preserved,
    // now stored as ONE 512x16 synchronous block ROM (8192 bits).
    // Older nested 26x16 combinational case generated wide LUT muxes.
    // +1BRAM9K candidate, but placement must verify TD inference.
    // All 512 locations explicitly initialized: self-contained TOP .v,
    // NO additional .mem, .mif or font files in the TD project.
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    reg [15:0] e971_cjk_rom [0:511];
    initial begin
        e971_cjk_rom[9'd0] = 16'h0000;
        e971_cjk_rom[9'd1] = 16'h0180;
        e971_cjk_rom[9'd2] = 16'h1FFE;
        e971_cjk_rom[9'd3] = 16'h1FFE;
        e971_cjk_rom[9'd4] = 16'h0418;
        e971_cjk_rom[9'd5] = 16'h0630;
        e971_cjk_rom[9'd6] = 16'h3FFE;
        e971_cjk_rom[9'd7] = 16'h3FFE;
        e971_cjk_rom[9'd8] = 16'h0000;
        e971_cjk_rom[9'd9] = 16'h0FFC;
        e971_cjk_rom[9'd10] = 16'h080C;
        e971_cjk_rom[9'd11] = 16'h080C;
        e971_cjk_rom[9'd12] = 16'h0FFC;
        e971_cjk_rom[9'd13] = 16'h080C;
        e971_cjk_rom[9'd14] = 16'h0FFC;
        e971_cjk_rom[9'd15] = 16'h0FFC;
        e971_cjk_rom[9'd16] = 16'h0000;
        e971_cjk_rom[9'd17] = 16'h0C60;
        e971_cjk_rom[9'd18] = 16'h0C60;
        e971_cjk_rom[9'd19] = 16'h3F60;
        e971_cjk_rom[9'd20] = 16'h3FFC;
        e971_cjk_rom[9'd21] = 16'h1EFC;
        e971_cjk_rom[9'd22] = 16'h166C;
        e971_cjk_rom[9'd23] = 16'h366C;
        e971_cjk_rom[9'd24] = 16'h3F6C;
        e971_cjk_rom[9'd25] = 16'h3F6C;
        e971_cjk_rom[9'd26] = 16'h066C;
        e971_cjk_rom[9'd27] = 16'h1F4C;
        e971_cjk_rom[9'd28] = 16'h3FCE;
        e971_cjk_rom[9'd29] = 16'h06CF;
        e971_cjk_rom[9'd30] = 16'h078E;
        e971_cjk_rom[9'd31] = 16'h0684;
        e971_cjk_rom[9'd32] = 16'h0000;
        e971_cjk_rom[9'd33] = 16'h1FFC;
        e971_cjk_rom[9'd34] = 16'h1FFC;
        e971_cjk_rom[9'd35] = 16'h1FF8;
        e971_cjk_rom[9'd36] = 16'h1FF8;
        e971_cjk_rom[9'd37] = 16'h0018;
        e971_cjk_rom[9'd38] = 16'h3FFE;
        e971_cjk_rom[9'd39] = 16'h3FFE;
        e971_cjk_rom[9'd40] = 16'h18CC;
        e971_cjk_rom[9'd41] = 16'h0EDC;
        e971_cjk_rom[9'd42] = 16'h05F0;
        e971_cjk_rom[9'd43] = 16'h0FB8;
        e971_cjk_rom[9'd44] = 16'h3C9C;
        e971_cjk_rom[9'd45] = 16'h358E;
        e971_cjk_rom[9'd46] = 16'h0380;
        e971_cjk_rom[9'd47] = 16'h0000;
        e971_cjk_rom[9'd48] = 16'h0000;
        e971_cjk_rom[9'd49] = 16'h180C;
        e971_cjk_rom[9'd50] = 16'h1BFE;
        e971_cjk_rom[9'd51] = 16'h19A4;
        e971_cjk_rom[9'd52] = 16'h3CAC;
        e971_cjk_rom[9'd53] = 16'h3FFE;
        e971_cjk_rom[9'd54] = 16'h1878;
        e971_cjk_rom[9'd55] = 16'h18EC;
        e971_cjk_rom[9'd56] = 16'h1FA6;
        e971_cjk_rom[9'd57] = 16'h7F02;
        e971_cjk_rom[9'd58] = 16'h79FE;
        e971_cjk_rom[9'd59] = 16'h1926;
        e971_cjk_rom[9'd60] = 16'h19FE;
        e971_cjk_rom[9'd61] = 16'h1926;
        e971_cjk_rom[9'd62] = 16'h39FE;
        e971_cjk_rom[9'd63] = 16'h3106;
        e971_cjk_rom[9'd64] = 16'h0000;
        e971_cjk_rom[9'd65] = 16'h0C30;
        e971_cjk_rom[9'd66] = 16'h0C20;
        e971_cjk_rom[9'd67] = 16'h3F60;
        e971_cjk_rom[9'd68] = 16'h3F7F;
        e971_cjk_rom[9'd69] = 16'h187F;
        e971_cjk_rom[9'd70] = 16'h18E4;
        e971_cjk_rom[9'd71] = 16'h1FEC;
        e971_cjk_rom[9'd72] = 16'h1FEC;
        e971_cjk_rom[9'd73] = 16'h1B3C;
        e971_cjk_rom[9'd74] = 16'h1B38;
        e971_cjk_rom[9'd75] = 16'h1B18;
        e971_cjk_rom[9'd76] = 16'h1338;
        e971_cjk_rom[9'd77] = 16'h337C;
        e971_cjk_rom[9'd78] = 16'h7EEF;
        e971_cjk_rom[9'd79] = 16'h2EC2;
        e971_cjk_rom[9'd80] = 16'h0000;
        e971_cjk_rom[9'd81] = 16'h0C60;
        e971_cjk_rom[9'd82] = 16'h0FFE;
        e971_cjk_rom[9'd83] = 16'h0800;
        e971_cjk_rom[9'd84] = 16'h1800;
        e971_cjk_rom[9'd85] = 16'h19FC;
        e971_cjk_rom[9'd86] = 16'h3904;
        e971_cjk_rom[9'd87] = 16'h79FC;
        e971_cjk_rom[9'd88] = 16'h3800;
        e971_cjk_rom[9'd89] = 16'h1BFE;
        e971_cjk_rom[9'd90] = 16'h1A02;
        e971_cjk_rom[9'd91] = 16'h1BFE;
        e971_cjk_rom[9'd92] = 16'h1830;
        e971_cjk_rom[9'd93] = 16'h1830;
        e971_cjk_rom[9'd94] = 16'h18F0;
        e971_cjk_rom[9'd95] = 16'h18E0;
        e971_cjk_rom[9'd96] = 16'h0000;
        e971_cjk_rom[9'd97] = 16'h00C0;
        e971_cjk_rom[9'd98] = 16'h00C0;
        e971_cjk_rom[9'd99] = 16'h00C0;
        e971_cjk_rom[9'd100] = 16'h08C0;
        e971_cjk_rom[9'd101] = 16'h08C0;
        e971_cjk_rom[9'd102] = 16'h08FE;
        e971_cjk_rom[9'd103] = 16'h08FE;
        e971_cjk_rom[9'd104] = 16'h08C0;
        e971_cjk_rom[9'd105] = 16'h08C0;
        e971_cjk_rom[9'd106] = 16'h08C0;
        e971_cjk_rom[9'd107] = 16'h08C0;
        e971_cjk_rom[9'd108] = 16'h08C0;
        e971_cjk_rom[9'd109] = 16'h3FFE;
        e971_cjk_rom[9'd110] = 16'h3FFE;
        e971_cjk_rom[9'd111] = 16'h0000;
        e971_cjk_rom[9'd112] = 16'h0000;
        e971_cjk_rom[9'd113] = 16'h080C;
        e971_cjk_rom[9'd114] = 16'h1BFE;
        e971_cjk_rom[9'd115] = 16'h33F0;
        e971_cjk_rom[9'd116] = 16'h7310;
        e971_cjk_rom[9'd117] = 16'h2FFE;
        e971_cjk_rom[9'd118] = 16'h1BFE;
        e971_cjk_rom[9'd119] = 16'h3B30;
        e971_cjk_rom[9'd120] = 16'h7BFE;
        e971_cjk_rom[9'd121] = 16'h3BC6;
        e971_cjk_rom[9'd122] = 16'h1BFE;
        e971_cjk_rom[9'd123] = 16'h1BC6;
        e971_cjk_rom[9'd124] = 16'h1AFE;
        e971_cjk_rom[9'd125] = 16'h1AC6;
        e971_cjk_rom[9'd126] = 16'h1EFE;
        e971_cjk_rom[9'd127] = 16'h1AC6;
        e971_cjk_rom[9'd128] = 16'h0000;
        e971_cjk_rom[9'd129] = 16'h7DFE;
        e971_cjk_rom[9'd130] = 16'h7DFE;
        e971_cjk_rom[9'd131] = 16'h1830;
        e971_cjk_rom[9'd132] = 16'h1870;
        e971_cjk_rom[9'd133] = 16'h3C78;
        e971_cjk_rom[9'd134] = 16'h3CFC;
        e971_cjk_rom[9'd135] = 16'h18FE;
        e971_cjk_rom[9'd136] = 16'h19B6;
        e971_cjk_rom[9'd137] = 16'h1932;
        e971_cjk_rom[9'd138] = 16'h1C30;
        e971_cjk_rom[9'd139] = 16'h7C30;
        e971_cjk_rom[9'd140] = 16'h3030;
        e971_cjk_rom[9'd141] = 16'h0030;
        e971_cjk_rom[9'd142] = 16'h0030;
        e971_cjk_rom[9'd143] = 16'h0000;
        e971_cjk_rom[9'd144] = 16'h0000;
        e971_cjk_rom[9'd145] = 16'h0830;
        e971_cjk_rom[9'd146] = 16'h1C30;
        e971_cjk_rom[9'd147] = 16'h39FE;
        e971_cjk_rom[9'd148] = 16'h31FE;
        e971_cjk_rom[9'd149] = 16'h2C30;
        e971_cjk_rom[9'd150] = 16'h0FFF;
        e971_cjk_rom[9'd151] = 16'h1BFF;
        e971_cjk_rom[9'd152] = 16'h380C;
        e971_cjk_rom[9'd153] = 16'h3BFE;
        e971_cjk_rom[9'd154] = 16'h1BFE;
        e971_cjk_rom[9'd155] = 16'h198C;
        e971_cjk_rom[9'd156] = 16'h19CC;
        e971_cjk_rom[9'd157] = 16'h18CC;
        e971_cjk_rom[9'd158] = 16'h183C;
        e971_cjk_rom[9'd159] = 16'h1838;
        e971_cjk_rom[9'd160] = 16'h0000;
        e971_cjk_rom[9'd161] = 16'h00C0;
        e971_cjk_rom[9'd162] = 16'h01C0;
        e971_cjk_rom[9'd163] = 16'h0770;
        e971_cjk_rom[9'd164] = 16'h1E3C;
        e971_cjk_rom[9'd165] = 16'h3FFE;
        e971_cjk_rom[9'd166] = 16'h27F6;
        e971_cjk_rom[9'd167] = 16'h0000;
        e971_cjk_rom[9'd168] = 16'h1F7C;
        e971_cjk_rom[9'd169] = 16'h1F7C;
        e971_cjk_rom[9'd170] = 16'h1364;
        e971_cjk_rom[9'd171] = 16'h1364;
        e971_cjk_rom[9'd172] = 16'h1F7C;
        e971_cjk_rom[9'd173] = 16'h1F7C;
        e971_cjk_rom[9'd174] = 16'h1060;
        e971_cjk_rom[9'd175] = 16'h0060;
        e971_cjk_rom[9'd176] = 16'h0000;
        e971_cjk_rom[9'd177] = 16'h0FF8;
        e971_cjk_rom[9'd178] = 16'h0808;
        e971_cjk_rom[9'd179] = 16'h0FF8;
        e971_cjk_rom[9'd180] = 16'h0808;
        e971_cjk_rom[9'd181] = 16'h0FF8;
        e971_cjk_rom[9'd182] = 16'h3FFE;
        e971_cjk_rom[9'd183] = 16'h1FFC;
        e971_cjk_rom[9'd184] = 16'h188C;
        e971_cjk_rom[9'd185] = 16'h1FFC;
        e971_cjk_rom[9'd186] = 16'h188C;
        e971_cjk_rom[9'd187] = 16'h1FFC;
        e971_cjk_rom[9'd188] = 16'h0080;
        e971_cjk_rom[9'd189] = 16'h1FFC;
        e971_cjk_rom[9'd190] = 16'h0080;
        e971_cjk_rom[9'd191] = 16'h3FFE;
        e971_cjk_rom[9'd192] = 16'h0000;
        e971_cjk_rom[9'd193] = 16'h00C0;
        e971_cjk_rom[9'd194] = 16'h00C0;
        e971_cjk_rom[9'd195] = 16'h00C0;
        e971_cjk_rom[9'd196] = 16'h00C0;
        e971_cjk_rom[9'd197] = 16'h0CD8;
        e971_cjk_rom[9'd198] = 16'h18CC;
        e971_cjk_rom[9'd199] = 16'h18CC;
        e971_cjk_rom[9'd200] = 16'h18C6;
        e971_cjk_rom[9'd201] = 16'h38C6;
        e971_cjk_rom[9'd202] = 16'h30C6;
        e971_cjk_rom[9'd203] = 16'h70C3;
        e971_cjk_rom[9'd204] = 16'h00C2;
        e971_cjk_rom[9'd205] = 16'h00C0;
        e971_cjk_rom[9'd206] = 16'h0780;
        e971_cjk_rom[9'd207] = 16'h0780;
        e971_cjk_rom[9'd208] = 16'h0000;
        e971_cjk_rom[9'd209] = 16'h0630;
        e971_cjk_rom[9'd210] = 16'h3FFE;
        e971_cjk_rom[9'd211] = 16'h3FFE;
        e971_cjk_rom[9'd212] = 16'h0630;
        e971_cjk_rom[9'd213] = 16'h0630;
        e971_cjk_rom[9'd214] = 16'h0000;
        e971_cjk_rom[9'd215] = 16'h3FFC;
        e971_cjk_rom[9'd216] = 16'h3FFC;
        e971_cjk_rom[9'd217] = 16'h030C;
        e971_cjk_rom[9'd218] = 16'h030C;
        e971_cjk_rom[9'd219] = 16'h030C;
        e971_cjk_rom[9'd220] = 16'h033C;
        e971_cjk_rom[9'd221] = 16'h033C;
        e971_cjk_rom[9'd222] = 16'h0300;
        e971_cjk_rom[9'd223] = 16'h0300;
        e971_cjk_rom[9'd224] = 16'h0000;
        e971_cjk_rom[9'd225] = 16'h0C20;
        e971_cjk_rom[9'd226] = 16'h3F60;
        e971_cjk_rom[9'd227] = 16'h0CFC;
        e971_cjk_rom[9'd228] = 16'h3FC8;
        e971_cjk_rom[9'd229] = 16'h0CD8;
        e971_cjk_rom[9'd230] = 16'h7F7E;
        e971_cjk_rom[9'd231] = 16'h0036;
        e971_cjk_rom[9'd232] = 16'h3FFF;
        e971_cjk_rom[9'd233] = 16'h33FF;
        e971_cjk_rom[9'd234] = 16'h3F36;
        e971_cjk_rom[9'd235] = 16'h337E;
        e971_cjk_rom[9'd236] = 16'h3F36;
        e971_cjk_rom[9'd237] = 16'h3330;
        e971_cjk_rom[9'd238] = 16'h3370;
        e971_cjk_rom[9'd239] = 16'h3670;
        e971_cjk_rom[9'd240] = 16'h0000;
        e971_cjk_rom[9'd241] = 16'h0000;
        e971_cjk_rom[9'd242] = 16'h0000;
        e971_cjk_rom[9'd243] = 16'h0060;
        e971_cjk_rom[9'd244] = 16'h6060;
        e971_cjk_rom[9'd245] = 16'h37FE;
        e971_cjk_rom[9'd246] = 16'h1860;
        e971_cjk_rom[9'd247] = 16'h03FC;
        e971_cjk_rom[9'd248] = 16'h0264;
        e971_cjk_rom[9'd249] = 16'h7264;
        e971_cjk_rom[9'd250] = 16'h1264;
        e971_cjk_rom[9'd251] = 16'h13FC;
        e971_cjk_rom[9'd252] = 16'h10F0;
        e971_cjk_rom[9'd253] = 16'h11F8;
        e971_cjk_rom[9'd254] = 16'h1364;
        e971_cjk_rom[9'd255] = 16'h1460;
        e971_cjk_rom[9'd256] = 16'h0000;
        e971_cjk_rom[9'd257] = 16'h0000;
        e971_cjk_rom[9'd258] = 16'h0000;
        e971_cjk_rom[9'd259] = 16'h0080;
        e971_cjk_rom[9'd260] = 16'h0080;
        e971_cjk_rom[9'd261] = 16'h3FFE;
        e971_cjk_rom[9'd262] = 16'h2210;
        e971_cjk_rom[9'd263] = 16'h2210;
        e971_cjk_rom[9'd264] = 16'h2FFE;
        e971_cjk_rom[9'd265] = 16'h2210;
        e971_cjk_rom[9'd266] = 16'h2210;
        e971_cjk_rom[9'd267] = 16'h23F0;
        e971_cjk_rom[9'd268] = 16'h2000;
        e971_cjk_rom[9'd269] = 16'h2FFC;
        e971_cjk_rom[9'd270] = 16'h2208;
        e971_cjk_rom[9'd271] = 16'h2330;
        e971_cjk_rom[9'd272] = 16'h0000;
        e971_cjk_rom[9'd273] = 16'h0000;
        e971_cjk_rom[9'd274] = 16'h0000;
        e971_cjk_rom[9'd275] = 16'h0000;
        e971_cjk_rom[9'd276] = 16'h0400;
        e971_cjk_rom[9'd277] = 16'h0FE0;
        e971_cjk_rom[9'd278] = 16'h1860;
        e971_cjk_rom[9'd279] = 16'h30C0;
        e971_cjk_rom[9'd280] = 16'h7FFC;
        e971_cjk_rom[9'd281] = 16'h5084;
        e971_cjk_rom[9'd282] = 16'h1084;
        e971_cjk_rom[9'd283] = 16'h1084;
        e971_cjk_rom[9'd284] = 16'h1FFC;
        e971_cjk_rom[9'd285] = 16'h1004;
        e971_cjk_rom[9'd286] = 16'h1000;
        e971_cjk_rom[9'd287] = 16'h1002;
        e971_cjk_rom[9'd288] = 16'h0000;
        e971_cjk_rom[9'd289] = 16'h0000;
        e971_cjk_rom[9'd290] = 16'h0000;
        e971_cjk_rom[9'd291] = 16'h1020;
        e971_cjk_rom[9'd292] = 16'h1020;
        e971_cjk_rom[9'd293] = 16'h11FE;
        e971_cjk_rom[9'd294] = 16'h2102;
        e971_cjk_rom[9'd295] = 16'h2502;
        e971_cjk_rom[9'd296] = 16'h49FE;
        e971_cjk_rom[9'd297] = 16'h7900;
        e971_cjk_rom[9'd298] = 16'h11FE;
        e971_cjk_rom[9'd299] = 16'h21AA;
        e971_cjk_rom[9'd300] = 16'h7BAA;
        e971_cjk_rom[9'd301] = 16'h72AA;
        e971_cjk_rom[9'd302] = 16'h02FE;
        e971_cjk_rom[9'd303] = 16'h1AAA;
        e971_cjk_rom[9'd304] = 16'h0000;
        e971_cjk_rom[9'd305] = 16'h0000;
        e971_cjk_rom[9'd306] = 16'h0000;
        e971_cjk_rom[9'd307] = 16'h1000;
        e971_cjk_rom[9'd308] = 16'h10FC;
        e971_cjk_rom[9'd309] = 16'h7E84;
        e971_cjk_rom[9'd310] = 16'h30FC;
        e971_cjk_rom[9'd311] = 16'h2000;
        e971_cjk_rom[9'd312] = 16'h29FE;
        e971_cjk_rom[9'd313] = 16'h2884;
        e971_cjk_rom[9'd314] = 16'h4884;
        e971_cjk_rom[9'd315] = 16'h7CFC;
        e971_cjk_rom[9'd316] = 16'h0884;
        e971_cjk_rom[9'd317] = 16'h08FC;
        e971_cjk_rom[9'd318] = 16'h1E84;
        e971_cjk_rom[9'd319] = 16'h7884;
        e971_cjk_rom[9'd320] = 16'h3000;
        e971_cjk_rom[9'd321] = 16'h31FE;
        e971_cjk_rom[9'd322] = 16'h7FFE;
        e971_cjk_rom[9'd323] = 16'h7F8E;
        e971_cjk_rom[9'd324] = 16'h61DE;
        e971_cjk_rom[9'd325] = 16'h7DFE;
        e971_cjk_rom[9'd326] = 16'h3DF6;
        e971_cjk_rom[9'd327] = 16'h19B6;
        e971_cjk_rom[9'd328] = 16'h7FB6;
        e971_cjk_rom[9'd329] = 16'h7FB6;
        e971_cjk_rom[9'd330] = 16'h19FE;
        e971_cjk_rom[9'd331] = 16'h19FE;
        e971_cjk_rom[9'd332] = 16'h1FCE;
        e971_cjk_rom[9'd333] = 16'h1FC6;
        e971_cjk_rom[9'd334] = 16'h398E;
        e971_cjk_rom[9'd335] = 16'h110C;
        e971_cjk_rom[9'd336] = 16'h7FFE;
        e971_cjk_rom[9'd337] = 16'h1818;
        e971_cjk_rom[9'd338] = 16'h7E7E;
        e971_cjk_rom[9'd339] = 16'h1C38;
        e971_cjk_rom[9'd340] = 16'h7E7E;
        e971_cjk_rom[9'd341] = 16'h7FFE;
        e971_cjk_rom[9'd342] = 16'h03C0;
        e971_cjk_rom[9'd343] = 16'h0FF0;
        e971_cjk_rom[9'd344] = 16'h7DBE;
        e971_cjk_rom[9'd345] = 16'h718E;
        e971_cjk_rom[9'd346] = 16'h1FF8;
        e971_cjk_rom[9'd347] = 16'h1FF0;
        e971_cjk_rom[9'd348] = 16'h0070;
        e971_cjk_rom[9'd349] = 16'h00E0;
        e971_cjk_rom[9'd350] = 16'h00C0;
        e971_cjk_rom[9'd351] = 16'h0000;
        e971_cjk_rom[9'd352] = 16'h0180;
        e971_cjk_rom[9'd353] = 16'h0180;
        e971_cjk_rom[9'd354] = 16'h0180;
        e971_cjk_rom[9'd355] = 16'h7FFE;
        e971_cjk_rom[9'd356] = 16'h7FFE;
        e971_cjk_rom[9'd357] = 16'h0600;
        e971_cjk_rom[9'd358] = 16'h0600;
        e971_cjk_rom[9'd359] = 16'h07FC;
        e971_cjk_rom[9'd360] = 16'h0FFC;
        e971_cjk_rom[9'd361] = 16'h0C0C;
        e971_cjk_rom[9'd362] = 16'h0C1C;
        e971_cjk_rom[9'd363] = 16'h1C18;
        e971_cjk_rom[9'd364] = 16'h1818;
        e971_cjk_rom[9'd365] = 16'h3818;
        e971_cjk_rom[9'd366] = 16'h71F8;
        e971_cjk_rom[9'd367] = 16'h60F0;
        e971_cjk_rom[9'd368] = 16'h2060;
        e971_cjk_rom[9'd369] = 16'h3860;
        e971_cjk_rom[9'd370] = 16'h1BFE;
        e971_cjk_rom[9'd371] = 16'h03FE;
        e971_cjk_rom[9'd372] = 16'h0366;
        e971_cjk_rom[9'd373] = 16'h6366;
        e971_cjk_rom[9'd374] = 16'h73FC;
        e971_cjk_rom[9'd375] = 16'h07FC;
        e971_cjk_rom[9'd376] = 16'h078C;
        e971_cjk_rom[9'd377] = 16'h16DC;
        e971_cjk_rom[9'd378] = 16'h1EF8;
        e971_cjk_rom[9'd379] = 16'h3678;
        e971_cjk_rom[9'd380] = 16'h3678;
        e971_cjk_rom[9'd381] = 16'h6EFC;
        e971_cjk_rom[9'd382] = 16'h6FCE;
        e971_cjk_rom[9'd383] = 16'h0586;
        e971_cjk_rom[9'd384] = 16'h0080;
        e971_cjk_rom[9'd385] = 16'h0080;
        e971_cjk_rom[9'd386] = 16'h0080;
        e971_cjk_rom[9'd387] = 16'h0080;
        e971_cjk_rom[9'd388] = 16'h0080;
        e971_cjk_rom[9'd389] = 16'h0C90;
        e971_cjk_rom[9'd390] = 16'h1888;
        e971_cjk_rom[9'd391] = 16'h1084;
        e971_cjk_rom[9'd392] = 16'h2082;
        e971_cjk_rom[9'd393] = 16'h4081;
        e971_cjk_rom[9'd394] = 16'h4081;
        e971_cjk_rom[9'd395] = 16'h0080;
        e971_cjk_rom[9'd396] = 16'h0080;
        e971_cjk_rom[9'd397] = 16'h0080;
        e971_cjk_rom[9'd398] = 16'h0080;
        e971_cjk_rom[9'd399] = 16'h0300;
        e971_cjk_rom[9'd400] = 16'h31FC;
        e971_cjk_rom[9'd401] = 16'h2104;
        e971_cjk_rom[9'd402] = 16'h25FC;
        e971_cjk_rom[9'd403] = 16'hFD04;
        e971_cjk_rom[9'd404] = 16'h2104;
        e971_cjk_rom[9'd405] = 16'h21FC;
        e971_cjk_rom[9'd406] = 16'h2505;
        e971_cjk_rom[9'd407] = 16'h3FFF;
        e971_cjk_rom[9'd408] = 16'h7020;
        e971_cjk_rom[9'd409] = 16'hA120;
        e971_cjk_rom[9'd410] = 16'h2322;
        e971_cjk_rom[9'd411] = 16'h233F;
        e971_cjk_rom[9'd412] = 16'h2320;
        e971_cjk_rom[9'd413] = 16'h24A0;
        e971_cjk_rom[9'd414] = 16'hA4E0;
        e971_cjk_rom[9'd415] = 16'h683F;
        e971_cjk_rom[9'd416] = 16'h1020;
        e971_cjk_rom[9'd417] = 16'h1020;
        e971_cjk_rom[9'd418] = 16'h1440;
        e971_cjk_rom[9'd419] = 16'hFDFE;
        e971_cjk_rom[9'd420] = 16'h1104;
        e971_cjk_rom[9'd421] = 16'h1104;
        e971_cjk_rom[9'd422] = 16'h1504;
        e971_cjk_rom[9'd423] = 16'h1904;
        e971_cjk_rom[9'd424] = 16'hF1FC;
        e971_cjk_rom[9'd425] = 16'h9104;
        e971_cjk_rom[9'd426] = 16'h1104;
        e971_cjk_rom[9'd427] = 16'h1104;
        e971_cjk_rom[9'd428] = 16'h1104;
        e971_cjk_rom[9'd429] = 16'h11FC;
        e971_cjk_rom[9'd430] = 16'h7106;
        e971_cjk_rom[9'd431] = 16'h2104;
        e971_cjk_rom[9'd432] = 16'h4040;
        e971_cjk_rom[9'd433] = 16'h2042;
        e971_cjk_rom[9'd434] = 16'h27FE;
        e971_cjk_rom[9'd435] = 16'h1404;
        e971_cjk_rom[9'd436] = 16'h9FF8;
        e971_cjk_rom[9'd437] = 16'hD040;
        e971_cjk_rom[9'd438] = 16'h27FC;
        e971_cjk_rom[9'd439] = 16'h2444;
        e971_cjk_rom[9'd440] = 16'h2444;
        e971_cjk_rom[9'd441] = 16'h47FC;
        e971_cjk_rom[9'd442] = 16'hC444;
        e971_cjk_rom[9'd443] = 16'h47FC;
        e971_cjk_rom[9'd444] = 16'h4584;
        e971_cjk_rom[9'd445] = 16'h430C;
        e971_cjk_rom[9'd446] = 16'h4406;
        e971_cjk_rom[9'd447] = 16'h0000;
        e971_cjk_rom[9'd448] = 16'h0100;
        e971_cjk_rom[9'd449] = 16'h0304;
        e971_cjk_rom[9'd450] = 16'h7FFC;
        e971_cjk_rom[9'd451] = 16'h0210;
        e971_cjk_rom[9'd452] = 16'h7FF8;
        e971_cjk_rom[9'd453] = 16'h0404;
        e971_cjk_rom[9'd454] = 16'hFFFE;
        e971_cjk_rom[9'd455] = 16'h0840;
        e971_cjk_rom[9'd456] = 16'h1FE0;
        e971_cjk_rom[9'd457] = 16'h2118;
        e971_cjk_rom[9'd458] = 16'h421E;
        e971_cjk_rom[9'd459] = 16'hBFF0;
        e971_cjk_rom[9'd460] = 16'h0200;
        e971_cjk_rom[9'd461] = 16'h0470;
        e971_cjk_rom[9'd462] = 16'h0818;
        e971_cjk_rom[9'd463] = 16'h3000;
        e971_cjk_rom[9'd464] = 16'h0100;
        e971_cjk_rom[9'd465] = 16'h0104;
        e971_cjk_rom[9'd466] = 16'hFFFE;
        e971_cjk_rom[9'd467] = 16'h1FF0;
        e971_cjk_rom[9'd468] = 16'h1020;
        e971_cjk_rom[9'd469] = 16'h1020;
        e971_cjk_rom[9'd470] = 16'h1FE0;
        e971_cjk_rom[9'd471] = 16'h1024;
        e971_cjk_rom[9'd472] = 16'h7FFE;
        e971_cjk_rom[9'd473] = 16'h4004;
        e971_cjk_rom[9'd474] = 16'h87C0;
        e971_cjk_rom[9'd475] = 16'h0440;
        e971_cjk_rom[9'd476] = 16'h0C42;
        e971_cjk_rom[9'd477] = 16'h0842;
        e971_cjk_rom[9'd478] = 16'h107E;
        e971_cjk_rom[9'd479] = 16'hC000;
        e971_cjk_rom[9'd480] = 16'h1802;
        e971_cjk_rom[9'd481] = 16'h52FE;
        e971_cjk_rom[9'd482] = 16'h5E10;
        e971_cjk_rom[9'd483] = 16'h5020;
        e971_cjk_rom[9'd484] = 16'h52FE;
        e971_cjk_rom[9'd485] = 16'hFF84;
        e971_cjk_rom[9'd486] = 16'h0094;
        e971_cjk_rom[9'd487] = 16'h5A94;
        e971_cjk_rom[9'd488] = 16'h5294;
        e971_cjk_rom[9'd489] = 16'h5494;
        e971_cjk_rom[9'd490] = 16'h9CA4;
        e971_cjk_rom[9'd491] = 16'h08B4;
        e971_cjk_rom[9'd492] = 16'h102C;
        e971_cjk_rom[9'd493] = 16'h20C6;
        e971_cjk_rom[9'd494] = 16'hC302;
        e971_cjk_rom[9'd495] = 16'h0000;
        e971_cjk_rom[9'd496] = 16'h0100;
        e971_cjk_rom[9'd497] = 16'h0100;
        e971_cjk_rom[9'd498] = 16'h0100;
        e971_cjk_rom[9'd499] = 16'h7FFC;
        e971_cjk_rom[9'd500] = 16'h4104;
        e971_cjk_rom[9'd501] = 16'h4104;
        e971_cjk_rom[9'd502] = 16'h4104;
        e971_cjk_rom[9'd503] = 16'h4104;
        e971_cjk_rom[9'd504] = 16'h7FFC;
        e971_cjk_rom[9'd505] = 16'h4104;
        e971_cjk_rom[9'd506] = 16'h0100;
        e971_cjk_rom[9'd507] = 16'h0100;
        e971_cjk_rom[9'd508] = 16'h0100;
        e971_cjk_rom[9'd509] = 16'h0100;
        e971_cjk_rom[9'd510] = 16'h0100;
        e971_cjk_rom[9'd511] = 16'h0100;
    end
    reg [15:0] e971_cjk_row_q;
    reg [23:0] e971_overlay_color_q;
    reg e971_cjk_on_q;
    reg [3:0] e971_glyph_x_q;
    // E972 area optimization: move the large priority COLOR mux
    // from 24 parallel RGB bits to one registered 7-bit palette tag.
    // RAW BMP pixels and four track colors use a SINGLE 24-bit
    // dynamic bypass register; every ordinary editor rectangle uses
    // the same palette decoder after the tag register.
    reg [6:0] e972_tag_q;
    reg [23:0] e972_dynamic_q;
    reg [23:0] e972_palette_color;

    // RAM read and the corresponding whole RGB pixel are clocked
    // together at the original video_clk edge. The AXI RGB/DE/VS
    // receives a matching 1-pixel pipeline at the TOP interface.
    always @(posedge clk) begin
        e971_cjk_row_q <= e971_cjk_rom[{cjk_id,glyph_y}];
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            e972_tag_q <= 7'd0;
            e972_dynamic_q <= 24'd0;
            e971_overlay_color_q <= 24'd0;
            e971_cjk_on_q <= 1'b0;
            e971_glyph_x_q <= 4'd0;
        end else begin
            e972_tag_q <= pic_tag;
            // Only three cases ever need a 24-bit dynamic color.
            // The others are literals encoded in the palette tag.
            case(pic_tag)
                7'd0: e972_dynamic_q <= e978_bmp_brightness_rgb;
                7'd87: e972_dynamic_q <= lane_color;
                7'd88: e972_dynamic_q <=
                                lane_color_for_track(track_sel);
                default: e972_dynamic_q <= 24'd0;
            endcase
            e971_overlay_color_q <= overlay_color;
            e971_cjk_on_q <= cjk_on;
            e971_glyph_x_q <= glyph_x;
        end
    end

    // Small diagnostic status strip in the editor header.  Each of the 11
    // cells has a 5x7 letter and one red/green/gray horizontal indicator.
    function [4:0] e931_dbg_glyph;
        input [3:0] id;
        input [2:0] gy;
        begin
            e931_dbg_glyph = 5'b00000;
            case (id)
                4'd0: case (gy) // P
                    3'd0: e931_dbg_glyph = 5'b11110;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b11110;
                    3'd4: e931_dbg_glyph = 5'b10000;
                    3'd5: e931_dbg_glyph = 5'b10000;
                    3'd6: e931_dbg_glyph = 5'b10000;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd1: case (gy) // H
                    3'd0: e931_dbg_glyph = 5'b10001;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b11111;
                    3'd4: e931_dbg_glyph = 5'b10001;
                    3'd5: e931_dbg_glyph = 5'b10001;
                    3'd6: e931_dbg_glyph = 5'b10001;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd2: case (gy) // D
                    3'd0: e931_dbg_glyph = 5'b11110;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b10001;
                    3'd4: e931_dbg_glyph = 5'b10001;
                    3'd5: e931_dbg_glyph = 5'b10001;
                    3'd6: e931_dbg_glyph = 5'b11110;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd3: case (gy) // L
                    3'd0: e931_dbg_glyph = 5'b10000;
                    3'd1: e931_dbg_glyph = 5'b10000;
                    3'd2: e931_dbg_glyph = 5'b10000;
                    3'd3: e931_dbg_glyph = 5'b10000;
                    3'd4: e931_dbg_glyph = 5'b10000;
                    3'd5: e931_dbg_glyph = 5'b10000;
                    3'd6: e931_dbg_glyph = 5'b11111;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd4: case (gy) // K
                    3'd0: e931_dbg_glyph = 5'b10001;
                    3'd1: e931_dbg_glyph = 5'b10010;
                    3'd2: e931_dbg_glyph = 5'b10100;
                    3'd3: e931_dbg_glyph = 5'b11000;
                    3'd4: e931_dbg_glyph = 5'b10100;
                    3'd5: e931_dbg_glyph = 5'b10010;
                    3'd6: e931_dbg_glyph = 5'b10001;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd5: case (gy) // G
                    3'd0: e931_dbg_glyph = 5'b01111;
                    3'd1: e931_dbg_glyph = 5'b10000;
                    3'd2: e931_dbg_glyph = 5'b10000;
                    3'd3: e931_dbg_glyph = 5'b10111;
                    3'd4: e931_dbg_glyph = 5'b10001;
                    3'd5: e931_dbg_glyph = 5'b10001;
                    3'd6: e931_dbg_glyph = 5'b01111;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd6: case (gy) // Q
                    3'd0: e931_dbg_glyph = 5'b01110;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b10001;
                    3'd4: e931_dbg_glyph = 5'b10101;
                    3'd5: e931_dbg_glyph = 5'b10010;
                    3'd6: e931_dbg_glyph = 5'b01101;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd7: case (gy) // R
                    3'd0: e931_dbg_glyph = 5'b11110;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b11110;
                    3'd4: e931_dbg_glyph = 5'b10100;
                    3'd5: e931_dbg_glyph = 5'b10010;
                    3'd6: e931_dbg_glyph = 5'b10001;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd8: case (gy) // A
                    3'd0: e931_dbg_glyph = 5'b01110;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b11111;
                    3'd4: e931_dbg_glyph = 5'b10001;
                    3'd5: e931_dbg_glyph = 5'b10001;
                    3'd6: e931_dbg_glyph = 5'b10001;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd9: case (gy) // F
                    3'd0: e931_dbg_glyph = 5'b11111;
                    3'd1: e931_dbg_glyph = 5'b10000;
                    3'd2: e931_dbg_glyph = 5'b10000;
                    3'd3: e931_dbg_glyph = 5'b11110;
                    3'd4: e931_dbg_glyph = 5'b10000;
                    3'd5: e931_dbg_glyph = 5'b10000;
                    3'd6: e931_dbg_glyph = 5'b10000;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                4'd10: case (gy) // U
                    3'd0: e931_dbg_glyph = 5'b10001;
                    3'd1: e931_dbg_glyph = 5'b10001;
                    3'd2: e931_dbg_glyph = 5'b10001;
                    3'd3: e931_dbg_glyph = 5'b10001;
                    3'd4: e931_dbg_glyph = 5'b10001;
                    3'd5: e931_dbg_glyph = 5'b10001;
                    3'd6: e931_dbg_glyph = 5'b01110;
                    default: e931_dbg_glyph = 5'b00000;
                endcase
                default: e931_dbg_glyph = 5'b00000;
            endcase
        end
    endfunction
    reg [3:0] e931_di, e931_dx;
    reg [3:0] e931_dy;
    reg [4:0] e931_glyph;
    reg [3:0] e933_idx, e933_dx;
    reg [4:0] e933_dy, e933_digit_bits;
    reg e931_status_on;
    reg cjk_on,digit_on;
    reg [4:0] cjk_id;
    reg [3:0] glyph_x,glyph_y;
    reg [4:0] ascii_digit;
    reg [2:0] ascii_x,ascii_y;
    reg [2:0] e941_hex_idx, e941_hex_px;
    reg [3:0] e941_hex_nibble;
    reg [4:0] e941_hex_pixels;
    reg [23:0] overlay_color;
    reg [6:0] pic_tag;
    reg [4:0] ascii_pixels;
    always @* begin
        cjk_on=1'b0;digit_on=1'b0;cjk_id=5'd0;
        glyph_x=4'd0;glyph_y=4'd0;
        ascii_digit=4'd0;ascii_x=3'd0;ascii_y=3'd0;
        e941_hex_idx=3'd0;
        e941_hex_px=3'd0;
        e941_hex_nibble=4'd0;
        e941_hex_pixels=5'd0;
        overlay_color=24'hffffff;
        e931_di=4'd0;
        e931_dx=4'd0;
        e931_dy=4'd0;
        e931_glyph=5'b00000;
        e931_status_on=1'b0;
        e933_idx=4'd0;
        e933_dx=4'd0;
        e933_dy=5'd0;
        e933_digit_bits=5'd0;
        pic_tag=7'd0;
        if(de && editor_on) begin
            pic_tag=7'd1;
            if(py<9'd52) pic_tag=7'd5;
            else if(py>=9'd235 && py<9'd247) pic_tag=7'd14;
            else if(py>=9'd455) pic_tag=7'd3;
            if(memory_full && px>=10'd598 && py<9'd47) pic_tag=7'd60;
            if(in_lane && px>=10'd16 && px<10'd616) begin
                pic_tag=lane==track_sel ? 7'd19 : 7'd9;
                if(in_time) begin
                    if(maps_ready && small_band_in && e951_band_hit)
                        pic_tag=track_mute[lane] ? 7'd46 : 7'd87;
                    if(px[2:0]==3'd0) pic_tag=7'd31;
                    if(running && grid_col==play_col && px[2:0]<3'd2)
                        pic_tag=7'd84;
                end
                if(lane==track_sel && (py==lane_text_y-9'd4 ||
                                      py==lane_text_y+9'd31 || px==10'd16 ||
                                      px==10'd615)) pic_tag=7'd83;
                // Per-track tapered-volume meter, left of note lane.
                if(px>=10'd19 && px<10'd79 &&
                   py>=lane_text_y+9'd23 && py<lane_text_y+9'd28)
                    pic_tag=(px-10'd19 < ({6'd0,track_volume[lane]}<<2)) ?
                         7'd87 : 7'd34;
            end
            if(in_time && py>=9'd250 && py<9'd450) begin
                pic_tag=grid_row[0] ? 7'd6 : 7'd8;
                if(maps_ready && grid_row<5'd25 && detail_pixel_q[current_pitch])
                    pic_tag=7'd88;
                if(grid_col[2:0]==3'd0 && px[2:0]==3'd0)
                    pic_tag=7'd35;
                if(py[2:0]==3'd0) pic_tag=7'd22;
                if(running && grid_col==play_col && px[2:0]<3'd2)
                    pic_tag=7'd84;
                if(!running && grid_col==edit_col && px[2:0]<3'd2)
                    pic_tag=7'd63;
                if(fine_mode && grid_row<5'd25 && current_pitch==edit_pitch &&
                   px>=10'd104 && px<10'd111)
                    pic_tag=fine_selected ? 7'd80 : 7'd85;
                if(key_down[current_pitch] && grid_row<5'd25 && px>=10'd111 && px<10'd117)
                    pic_tag=7'd85;
            end
            // E965/P4.3.1: a visible 3-choice palette ONLY when 音色
            // is selected.  Even if VIOLIN.BIN is unavailable, the
            // 小提琴 option stays visible; green=loaded, amber=missing.
            // The same 16x16 glyph ROM paints the labels below.
            if(enc_param==3'd2 && py>=9'd12 && py<9'd28 &&
               px>=10'd518 && px<10'd640) begin
                if(px<10'd554)
                    pic_tag=(track_instrument[track_sel]==2'd0) ?
                        7'd26 : 7'd17;
                else if(px<10'd590)
                    pic_tag=(track_instrument[track_sel]==2'd1) ?
                        7'd20 : 7'd17;
                else
                    pic_tag=(track_instrument[track_sel]==2'd2) ?
                        (e964_violin_loaded ? 7'd21 : 7'd54) :
                        7'd17;
            end
            // One CJK glyph selection network drives one 16x16 ROM.
            if(py>=9'd12 && py<9'd28) begin
                if(px>=10'd20 && px<10'd52) begin
                    cjk_on=1'b1;cjk_id=(px<10'd36)? 5'd0:5'd1;
                    glyph_x=(px-10'd20)&10'hf;glyph_y=py-9'd12;
                    overlay_color=24'hffffff;
                end else if(px>=10'd196 && px<10'd228) begin
                    cjk_on=1'b1;
                    if(rec_active || (e952_mic_rec_enabled && running))
                        cjk_id=(px<10'd212)? 5'd2:5'd0; // 中文：录音
                    else if(rec_armed) cjk_id=(px<10'd212)? 5'd9:5'd10; // 待命
                    else if(running) cjk_id=(px<10'd212)? 5'd3:5'd4; // 播放
                    else cjk_id=(px<10'd212)? 5'd5:5'd6; // 停止
                    glyph_x=(px-10'd196)&10'hf;glyph_y=py-9'd12;
                    overlay_color=rec_active? 24'hff9d93:24'hbbdbf1;
                end else if(enc_param==3'd2 &&
                            px>=10'd470 && px<10'd518) begin
                    // 0钢琴 1方波 2小提琴, original CJK renderer shared.
                    if(track_instrument[track_sel]==2'd2 ||
                       px<10'd502) begin
                        cjk_on=1'b1;
                        cjk_id=(track_instrument[track_sel]==2'd2) ?
                            (px<10'd486 ? 5'd24 :
                             px<10'd502 ? 5'd25 : 5'd21) :
                            (track_instrument[track_sel]==2'd1) ?
                            (px<10'd486 ? 5'd22 : 5'd23) :
                            (px<10'd486 ? 5'd20 : 5'd21);
                        glyph_x=(px-10'd470)&10'hf;
                        glyph_y=py-9'd12;
                        // Never hide a selectable sound behind a dim
                    // 'not loaded' color.  Selection and loading are
                    // independent hardware states.
                    overlay_color=(track_instrument[track_sel]==2'd2 &&
                                       !e964_violin_loaded) ?
                                       24'hffdda7 : 24'hf3c47c;
                    end
                end else if(enc_param==3'd2 && px>=10'd520 &&
                            px<10'd638) begin
                    // Persistent choices: 钢琴  方波  小提琴.
                    // ONE shared chinese_row() invocation remains below.
                    // Label is visible regardless of VIOLIN bank ready.
                    if(px<10'd552) begin
                        cjk_on=1'b1;
                        cjk_id=(px<10'd536) ? 5'd20 : 5'd21;
                        glyph_x=(px-10'd520)&10'hf;
                        overlay_color=(track_instrument[track_sel]==2'd0) ?
                                     24'hffefc6 : 24'ha8b9c7;
                    end else if(px>=10'd556 && px<10'd588) begin
                        cjk_on=1'b1;
                        cjk_id=(px<10'd572) ? 5'd22 : 5'd23;
                        glyph_x=(px-10'd556)&10'hf;
                        overlay_color=(track_instrument[track_sel]==2'd1) ?
                                     24'hffefc6 : 24'ha8b9c7;
                    end else if(px>=10'd590 && px<10'd638) begin
                        cjk_on=1'b1;
                        cjk_id=(px<10'd606) ? 5'd24 :
                               (px<10'd622) ? 5'd25 : 5'd21;
                        glyph_x=(px-10'd590)&10'hf;
                        overlay_color=(track_instrument[track_sel]==2'd2) ?
                            (e964_violin_loaded ? 24'hadffda : 24'hffdda7) :
                            24'ha8b9c7;
                    end
                    glyph_y=py-9'd12;
                end else if(px>=10'd428 && px<10'd460) begin
                    cjk_on=1'b1;
                    cjk_id=(enc_param==3'd0) ?
                           ((px<10'd444) ? 5'd15 : 5'd16) : // 速度
                           (enc_param==3'd1) ?
                           ((px<10'd444) ? 5'd0 : 5'd11) :  // 中文：音量
                           (enc_param==3'd2) ?
                           ((px<10'd444) ? 5'd0 : 5'd17) :  // 中文：音色
                           (enc_param==3'd3) ?
                           ((px<10'd444) ? 5'd14 : 5'd0) : // 静音
                           ((px<10'd444) ? 5'd7 : 5'd8);   // 循环
                    glyph_x=(px-10'd428)&10'hf;glyph_y=py-9'd12;
                    overlay_color=24'hf3c47c;
                end
            end
            // Track names: 音轨 + numeral 1-4 on left.
            if(in_lane && px>=10'd18 && px<10'd50 && py>=lane_text_y &&
               py<lane_text_y+9'd16) begin
                cjk_on=1'b1;
                cjk_id=px<10'd34 ? 5'd0:5'd1;
                glyph_x=(px-10'd18)&10'hf;
                glyph_y=py-lane_text_y;
                overlay_color=lane_color;
            end
            if(in_lane && px>=10'd56 && px<10'd61 && py>=lane_text_y+9'd5 &&
               py<lane_text_y+9'd12) begin
                digit_on=1'b1;ascii_digit={2'd0,lane}+4'd1;
                ascii_x=px-10'd56;ascii_y=py-(lane_text_y+9'd5);
            end
            // One shared CJK ROM, all four lanes identify the real sound.
            // Piano:钢琴, square:方波, violin:小提琴. A missing optional
            // VIOLIN.BIN draws gray text rather than pretending loaded.
            if(in_lane && px>=10'd84 && px<10'd132 &&
               py>=lane_text_y && py<lane_text_y+9'd16) begin
                if(track_instrument[lane]==2'd2 || px<10'd116) begin
                    cjk_on=1'b1;
                    cjk_id=(track_instrument[lane]==2'd2) ?
                        (px<10'd100 ? 5'd24 :
                         px<10'd116 ? 5'd25 : 5'd21) :
                        (track_instrument[lane]==2'd1) ?
                        (px<10'd100 ? 5'd22:5'd23) :
                        (px<10'd100 ? 5'd20:5'd21);
                    glyph_x=(px-10'd84)&10'hf;
                    glyph_y=py-lane_text_y;
                    overlay_color=track_mute[lane] ?
                       24'h7c8795 :
                       (track_instrument[lane]==2'd2 && !e964_violin_loaded) ?
                       24'he6bc8a : 24'he8efe9;
                end
            end
            // BPM legend and three-digit tempo at top, not hidden by menu.
            if(py>=9'd16 && py<9'd23 && px>=10'd286 && px<10'd304) begin
                digit_on=1'b1;
                if(px<10'd291) begin
                    ascii_digit=4'd11;ascii_x=px-10'd286;
                end else if(px>=10'd292 && px<10'd297) begin
                    ascii_digit=4'd12;ascii_x=px-10'd292;
                end else if(px>=10'd298 && px<10'd303) begin
                    ascii_digit=4'd13;ascii_x=px-10'd298;
                end else digit_on=1'b0;
                ascii_y=py-9'd16;
            end
            if(py>=9'd16 && py<9'd23 && px>=10'd319 && px<10'd336) begin
                if(px<10'd324 || (px>=10'd325 && px<10'd330) ||
                   (px>=10'd331 && px<10'd336)) begin
                    digit_on=1'b1;
                    if(px<10'd324) begin
                        ascii_digit=bpm_hundreds;ascii_x=px-10'd319;
                    end else if(px<10'd330) begin
                        ascii_digit=bpm_tens;ascii_x=px-10'd325;
                    end else begin
                        ascii_digit=bpm_ones;ascii_x=px-10'd331;
                    end
                    ascii_y=py-9'd16;
                end
            end
            // Piano-roll C4/C5/C6 anchors (top/middle/bottom).
            if(py>=9'd250 && py<9'd450 &&
               (grid_row==5'd0 || grid_row==5'd12 || grid_row==5'd24) &&
               px>=10'd72 && px<10'd83 &&
               ((px-10'd72)<10'd5 || (px-10'd72)>=10'd6)) begin
                digit_on=1'b1;
                ascii_digit=(px<10'd77)? 4'd10:
                             (grid_row==5'd0 ? 4'd6:grid_row==5'd12 ? 4'd5:4'd4);
                ascii_x=(px<10'd77)?px-10'd72:px-10'd78;
                ascii_y=(py-9'd250)&9'd7;
            end
            // Below overview: "循环" and "小节" labels.
            if(py>=9'd230 && py<9'd246 && px>=10'd20 && px<10'd52) begin
                cjk_on=1'b1;cjk_id=px<10'd36 ? 5'd7:5'd8;
                glyph_x=(px-10'd20)&10'hf;glyph_y=py-9'd230;
                overlay_color=24'hffffff;
            end
            if(py>=9'd230 && py<9'd246 && px>=10'd135 && px<10'd167) begin
                cjk_on=1'b1;cjk_id=px<10'd151 ? 5'd12:5'd13;
                glyph_x=(px-10'd135)&10'hf;glyph_y=py-9'd230;
                overlay_color=24'hffffff;
            end
            if(fine_mode && py>=9'd459 && py<9'd475 &&
               px>=10'd20 && px<10'd52) begin
                cjk_on=1'b1;
                cjk_id=px<10'd36 ? 5'd18 : 5'd19; // 编辑
                glyph_x=(px-10'd20)&10'hf;
                glyph_y=py-9'd459;
                overlay_color=delete_confirm ? 24'hff8880 : 24'hffe1a7;
            end
            // Quantize indicator: Q16 = starts within +/-3 ticks only.
            if(py>=9'd463 && py<9'd470 && px>=10'd558 && px<10'd575) begin
                if(px<10'd563 || (px>=10'd564 && px<10'd569) ||
                   (px>=10'd570 && px<10'd575)) begin
                    digit_on=1'b1;
                    if(px<10'd563) begin ascii_digit=4'd14;ascii_x=px-10'd558;end
                    else if(px<10'd569) begin ascii_digit=4'd1;ascii_x=px-10'd564;end
                    else begin ascii_digit=4'd6;ascii_x=px-10'd570;end
                    ascii_y=py-9'd463;
                end
            end
            // Fixed small digits, all through one 5x7 ROM.
            if(enc_param!=3'd2 && py>=9'd16 && py<9'd23 &&
               px>=10'd470 && px<10'd481) begin
                if((px-10'd470)<10'd5 || (px-10'd470)>=10'd6) begin
                    digit_on=1'b1;
                    if(enc_param==3'd0) begin
                        ascii_digit=px<10'd475 ? bpm_tens : bpm_ones;
                    end else if(enc_param==3'd1) begin
                        if(px<10'd475)
                            ascii_digit=(track_volume[track_sel]>=4'd10) ? 4'd1 : 4'd0;
                        else ascii_digit=(track_volume[track_sel]>=4'd10) ?
                                         track_volume[track_sel]-4'd10 : track_volume[track_sel];
                    end else if(enc_param==3'd3) begin
                        ascii_digit=px<10'd475 ? 4'd0 :
                                      {3'd0,track_mute[track_sel]};
                    end else begin
                        ascii_digit=px<10'd475 ? 4'd0 :
                                     bars_sel==2'd0 ? 4'd1 :
                                     bars_sel==2'd1 ? 4'd2 : 4'd4;
                    end
                    ascii_x=(px<10'd475)?px-10'd470:px-10'd476;
                    ascii_y=py-9'd16;
                end
            end else if(py>=9'd230 && py<9'd237 && px>=10'd76 && px<10'd81) begin
                digit_on=1'b1;
                ascii_digit=bars_sel==2'd0 ? 4'd1:bars_sel==2'd1 ? 4'd2:4'd4;
                ascii_x=px-10'd76;ascii_y=py-9'd230;
            end
        end
        // CJK glyphs are applied AFTER the one-pixel ROM pipeline.
        // Existing Latin digits still use the original combinational
        // glyph ROM. CJK boxes are y12..27, lane labels y76..210,
        // y230..245 and y459..474, disjoint from later mic/VU/mini
        // video effects in the deployed KEEP_E93_DEBUG_UI=0 profile.
        // E982/P4.8: only two media-specific cards + a MIC STATUS card.
        // Old page number and MM:SS panel are COMPLETELY removed.
        // LEFT: scrolling Chinese subtitle; CENTER: real SPH0645 MIC;
        // RIGHT: existing brightness readout (unchanged controls).
        if(de && e952_image_key_mode &&
           py>=9'd68 && py<9'd98) begin
            if(px>=10'd12 && px<10'd310) begin
                pic_tag=7'd2;
                if(px>=10'd16 && px<10'd20) pic_tag=7'd25;
            end else if(px>=10'd324 && px<10'd458) begin
                pic_tag=7'd2;
                if(px>=10'd328 && px<10'd331) pic_tag=7'd55;
                // Four real microphone amplitude bars (P2.5 21ms P2P).
                if(py>=9'd77 && py<9'd83 &&
                   px>=10'd402 && px<10'd446) begin
                    if(px>=10'd402 && px<10'd410)
                        pic_tag=(e946_vu_frame>=3'd1) ? 7'd25 : 7'd34;
                    else if(px>=10'd414 && px<10'd422)
                        pic_tag=(e946_vu_frame>=3'd2) ? 7'd25 : 7'd34;
                    else if(px>=10'd426 && px<10'd434)
                        pic_tag=(e946_vu_frame>=3'd3) ? 7'd25 : 7'd34;
                    else if(px>=10'd438 && px<10'd446)
                        pic_tag=(e946_vu_frame>=3'd4) ? 7'd25 : 7'd34;
                end
                // Pitch recognition status: bright green=confident note,
                // cyan=MIC has some energy, dim gray=silence.
                if(px>=10'd445 && px<10'd454 &&
                   py>=9'd86 && py<9'd94)
                    pic_tag=e946_note_visible ? 7'd33 :
                         (e946_vu_frame!=3'd0) ? 7'd32 : 7'd37;
            end else if(px>=10'd474 && px<10'd626) begin
                pic_tag=7'd2;
                if(px>=10'd478 && px<10'd481) pic_tag=7'd75;
            end
            if(py>=9'd76 && py<9'd92) begin
                // P5.1: four actual control functions, continuous
                // ONE-DIRECTION scrolling. 64px per item, four items:
                // SW1播放 | SW2节拍 | SW3演奏 | SW4编辑.
                // '录音中' takes a fixed 48px label while recording;
                // the SW ticker keeps moving to its right.
                if(e978_caption_rec &&
                   px>=10'd20 && px<10'd68) begin
                    cjk_on=1'b1;
                    cjk_id=(px<10'd36) ? 5'd2 :
                           (px<10'd52) ? 5'd0 : 5'd31;
                    glyph_x=(px-10'd20)&10'hf;
                    glyph_y=py-9'd76;
                    overlay_color=24'hffb17d;
                end else if(px>=10'd20 && px<10'd305 &&
                            (!e978_caption_rec || px>=10'd82)) begin
                    if(e990_caption_cell[5:0]>=6'd20 &&
                       e990_caption_cell[5:0]<6'd52) begin
                        cjk_on=1'b1;
                        case(e990_caption_cell[7:6])
                            2'd0: cjk_id =
                                (e990_caption_cell[5:0]<6'd36) ?
                                5'd3 : 5'd4;    // 播 放
                            2'd1: cjk_id =
                                (e990_caption_cell[5:0]<6'd36) ?
                                5'd13 : 5'd26;  // 节 拍
                            2'd2: cjk_id =
                                (e990_caption_cell[5:0]<6'd36) ?
                                5'd27 : 5'd28;  // 演 奏
                            default: cjk_id =
                                (e990_caption_cell[5:0]<6'd36) ?
                                5'd18 : 5'd19;  // 编 辑
                        endcase
                        // Both glyphs start at x=20 or36 (mod16=4).
                        glyph_x=e990_caption_cell[3:0]-4'd4;
                        glyph_y=py-9'd76;
                        overlay_color=24'h62e6e9;
                    end
                    if(py>=9'd80 && py<9'd87) begin
                        if(e990_caption_cell[5:0]<6'd5) begin
                            digit_on=1'b1;
                            ascii_digit=5'd22; // S
                            ascii_x=e990_caption_cell[2:0];
                        end else if(e990_caption_cell[5:0]>=6'd6 &&
                                    e990_caption_cell[5:0]<6'd11) begin
                            digit_on=1'b1;
                            ascii_digit=5'd23; // W
                            ascii_x=e990_caption_cell[2:0]-3'd6;
                        end else if(e990_caption_cell[5:0]>=6'd12 &&
                                    e990_caption_cell[5:0]<6'd17) begin
                            digit_on=1'b1;
                            ascii_digit={3'd0,e990_caption_cell[7:6]}+
                                        5'd1; // 1,2,3,4
                            ascii_x=e990_caption_cell[2:0]-3'd4;
                        end
                        ascii_y=py-9'd80;
                    end
                end
                // Center "音量" stays Chinese; actual MIC monitoring
                // is always on, independent of MIC REC-to-track switch.
                if(px>=10'd368 && px<10'd400) begin
                    cjk_on=1'b1;
                    cjk_id=(px<10'd384) ? 5'd0 : 5'd11;
                    glyph_x=(px-10'd368)&10'hf;
                    glyph_y=py-9'd76;
                    overlay_color=24'hbbdbf1;
                end
                if(px>=10'd486 && px<10'd518) begin
                    cjk_on=1'b1;
                    cjk_id=(px<10'd502) ? 5'd29 : 5'd16; // 亮度
                    glyph_x=(px-10'd486)&10'hf;
                    glyph_y=py-9'd76;
                    overlay_color=24'hf4c15c;
                end
            end
            // Reuse single shared digit_row 5x7 decoder for MIC/notes.
            if(py>=9'd77 && py<9'd84 &&
               px>=10'd335 && px<10'd359) begin
                if(px<10'd340) begin
                    digit_on=1'b1; ascii_digit=5'd13; // M
                    ascii_x=px-10'd335;
                end else if(px>=10'd344 && px<10'd349) begin
                    digit_on=1'b1; ascii_digit=5'd20; // I
                    ascii_x=px-10'd344;
                end else if(px>=10'd353 && px<10'd358) begin
                    digit_on=1'b1; ascii_digit=5'd10; // C
                    ascii_x=px-10'd353;
                end
                ascii_y=py-9'd77;
            end
            // Current recognized pitch under the MIC VU meter, C4..C6.
            if(e946_note_visible &&
               py>=9'd86 && py<9'd93 &&
               px>=10'd404 && px<10'd421) begin
                if(px<10'd409) begin
                    digit_on=1'b1;ascii_digit=e942_note_char;
                    ascii_x=px-10'd404;
                end else if(px>=10'd415 && px<10'd420) begin
                    digit_on=1'b1;
                    ascii_digit={3'd0,e946_note_shown[5:4]}+5'd3;
                    ascii_x=px-10'd415;
                end
                ascii_y=py-9'd86;
            end
            if(py>=9'd81 && py<9'd88) begin
                if(px>=10'd542 && px<10'd547 &&
                   e978_brightness_level==2'd2) begin
                    digit_on=1'b1;ascii_digit=5'd1;
                    ascii_x=px-10'd542;
                    ascii_y=py-9'd81;
                end else if(px>=10'd550 && px<10'd555) begin
                    digit_on=1'b1;
                    ascii_digit=(e978_brightness_level==2'd0) ?
                                  5'd2 :
                                (e978_brightness_level==2'd1) ?
                                  5'd5 : 5'd0;
                    ascii_x=px-10'd550;
                    ascii_y=py-9'd81;
                end else if(px>=10'd558 && px<10'd563) begin
                    digit_on=1'b1;
                    ascii_digit=(e978_brightness_level==2'd0) ?
                                  5'd5 : 5'd0;
                    ascii_x=px-10'd558;
                    ascii_y=py-9'd81;
                end
            end
        end
        // Existing Latin digits and all Chinese characters share the
        // SAME post-compositor 1-pixel RGB/DE/VS pipeline.
        ascii_pixels=digit_row(ascii_digit,ascii_y);
        if(digit_on && ascii_pixels[4-ascii_x]) pic_tag=7'd86;
        // Diagnostic-only RGB/hex glyph network. Default ON preserves baseline.
        // Set KEEP_E93_DEBUG_UI=0 to prune without altering the normal editor UI.
        if (KEEP_E93_DEBUG_UI) begin
        // E9.3.2 live amplitude bars directly below the existing flags.
        // Left bar is PCM output (before switch); right bar is the REAL
        // sample currently connected to the HDMI transmitter input.
        // A green bar requires a recent sample louder than -48dBFS;
        // gray means no such sample within the recent activity window.
        if (de && editor_on && py>=9'd50 && py<9'd58 &&
            px>=10'd464 && px<10'd636) begin
            if (px < 10'd546)
                pic_tag=e932_pcm_live ? 7'd44 : 7'd41;
            else if (px < 10'd552)
                pic_tag=7'd7;
            else
                pic_tag=e932_hdmi_live ? 7'd44 : 7'd41;
        end
        // E9.3.7 D: 0 scan_done; 1 BMP pushed >=307200 pixels;
        // 2 VIDEO writer issued >=307200 SDRAM write commands;
        // 3 load_busy; 4 first image committed; 5 display_valid;
        // 6 source_done_seen; 7 write_done_seen;
        // 8 RED if TF source finished but VIDEO writer issued no commands.
        // These are real-time flags for the current frame, except 4/5.
        // Only the nine status rectangles in this strip are new video pixels.
        if (de && editor_on && px>=10'd464 && px<10'd608 &&
            py>=9'd66 && py<9'd87) begin
            e933_idx=(px-10'd464)>>4;
            e933_dx=(px-10'd464)&10'd15;
            e933_dy=py-9'd66;
            pic_tag=e933_view_status[e933_idx] ?
                ((e933_idx==4'd8) ? 7'd70 : 7'd29) : 7'd41;
            if (e933_dx>=4'd5 && e933_dx<4'd10 &&
                e933_dy>=5'd5 && e933_dy<5'd12) begin
                e933_digit_bits = digit_row(e933_idx,e933_dy-5'd5);
                if (e933_digit_bits[4-(e933_dx-4'd5)])
                    pic_tag=7'd86;
            end
        end
        // E9.3.12 I: 32-bit hexadecimal data snapshots immediately below 0..8.
        // LEFT cyan: TF parser's first E93F PCM payload word.
        // RIGHT amber: C4 SDRAM read response (prioritize first BAD response).
        // Coordinates use spare row y=90..96. No audio or SDRAM logic affected.
        if (de && editor_on && py>=9'd90 && py<9'd97) begin
            if (e941_tf_ready && px>=10'd466 && px<10'd530) begin
                e941_hex_idx=(px-10'd466)>>3;
                e941_hex_px=(px-10'd466)&10'd7;
                e941_hex_nibble =
                    e941_tf_word[{~e941_hex_idx, 2'b00} +: 4];
                e941_hex_pixels=e941_hex_row(e941_hex_nibble,py-9'd90);
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=7'd45;
            end else if (e941_c4_read_ready &&
                         px>=10'd548 && px<10'd612) begin
                e941_hex_idx=(px-10'd548)>>3;
                e941_hex_px=(px-10'd548)&10'd7;
                e941_hex_nibble =
                    e941_c4_read_word[{~e941_hex_idx, 2'b00} +: 4];
                e941_hex_pixels=e941_hex_row(e941_hex_nibble,py-9'd90);
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=7'd81;
            end
        end
        // J diagnostic rows under cells 0..8:
        // y=90  left cyan=TF source word, right amber=SDRAM C4 return (I).
        // y=100 left green=first actual SDRAM PCM write data,
        //       right violet=first actual SDRAM PCM write address.
        // y=110 left blue=PCM App_wr_en total (after L). Red box on right
        //       indicates at least one simultaneous PCM/video command.
        if (de && editor_on && py>=9'd100 && py<9'd107) begin
            if (e943_first_wr_ready && px>=10'd466 && px<10'd530) begin
                e941_hex_idx=(px-10'd466)>>3;
                e941_hex_px=(px-10'd466)&10'd7;
                e941_hex_nibble=
                    e943_first_wr_word[{~e941_hex_idx,2'b00} +: 4];
                e941_hex_pixels=e941_hex_row(e941_hex_nibble,py-9'd100);
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=7'd52;
            end else if (e943_first_wr_ready &&
                         px>=10'd548 && px<10'd612) begin
                e941_hex_idx=(px-10'd548)>>3;
                e941_hex_px=(px-10'd548)&10'd7;
                e941_hex_nibble=
                    e943_first_wr_addr[{~e941_hex_idx,2'b00} +: 4];
                e941_hex_pixels=e941_hex_row(e941_hex_nibble,py-9'd100);
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=7'd66;
            end
        end
        if (de && editor_on && py>=9'd110 && py<9'd117) begin
            if (e943_wr_count_ready && px>=10'd466 && px<10'd530) begin
                e941_hex_idx=(px-10'd466)>>3;
                e941_hex_px=(px-10'd466)&10'd7;
                e941_hex_nibble=
                    e943_wr_count[{~e941_hex_idx,2'b00} +: 4];
                e941_hex_pixels=e941_hex_row(e941_hex_nibble,py-9'd110);
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=7'd57;
            end
            if (e943_collision && px>=10'd606 && px<10'd616)
                pic_tag=7'd78;
        end
        // N-DIAG additional HEX overlays in editor top-right:
        // y=120: LEFT status RRBBSSFF; RIGHT first mismatch word addr.
        // y=130: LEFT baseline block checksum; RIGHT later checksum.
        // RR=completed 1-minute rounds, BB=mismatched blocks,
        // SS=PCM cache-miss reads exceeding 10 us, FF=flags (see notes).
        // All are PASSIVE video registers; the probes use the existing
        // audio SDRAM port only about 48 words per minute.
        if (de && editor_on && e93_pcm_loaded &&
            ((py>=9'd120 && py<9'd127) ||
             (py>=9'd130 && py<9'd137))) begin
            if (px>=10'd466 && px<10'd530) begin
                e941_hex_idx=(px-10'd466)>>3;
                e941_hex_px=(px-10'd466)&10'd7;
                e941_hex_nibble = (py<9'd127) ?
                    e944_diag_status[{~e941_hex_idx,2'b00} +: 4] :
                    e944_diag_expected[{~e941_hex_idx,2'b00} +: 4];
                e941_hex_pixels = e941_hex_row(
                    e941_hex_nibble, (py<9'd127) ?
                        (py-9'd120) : (py-9'd130));
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=(py<9'd127) ? 7'd50 : 7'd56;
            end else if (px>=10'd548 && px<10'd612) begin
                e941_hex_idx=(px-10'd548)>>3;
                e941_hex_px=(px-10'd548)&10'd7;
                e941_hex_nibble=(py<9'd127) ?
                    e944_diag_bad_addr[{~e941_hex_idx,2'b00} +: 4] :
                    e944_diag_observed[{~e941_hex_idx,2'b00} +: 4];
                e941_hex_pixels = e941_hex_row(
                    e941_hex_nibble, (py<9'd127) ?
                        (py-9'd120) : (py-9'd130));
                if (e941_hex_px<3'd5 &&
                    e941_hex_pixels[4-e941_hex_px])
                    pic_tag=(py<9'd127) ? 7'd67 : 7'd79;
            end
        end
        // E9.3.1 debug: editor top-right, 11 indicators in order
        // P/H/D/L/K/G/Q/R/A/F/U. Green = achieved, gray = waiting,
        // red F/U = FIFO overflow / audio deadline missed.
        if (de && editor_on && px>=10'd464 && px<10'd640 &&
            py>=9'd30 && py<9'd45) begin
            e931_di = (px-10'd464) >> 4;
            e931_dx = (px-10'd464) & 10'd15;
            e931_dy = py-9'd30;
            e931_status_on=e93_debug_flags[e931_di];
            pic_tag=7'd7;
            if (e931_dy>=4'd2 && e931_dy<4'd9 &&
                e931_dx>=4'd5 && e931_dx<4'd10) begin
                e931_glyph=e931_dbg_glyph(e931_di,e931_dy[2:0]-3'd2);
                if (e931_glyph[4-(e931_dx-4'd5)]) pic_tag=7'd64;
            end
            if(e931_dy>=4'd11 && e931_dy<4'd15 &&
               e931_dx>=4'd2 && e931_dx<4'd14)
                pic_tag=e931_status_on ?
                      ((e931_di>=4'd9)?7'd74:7'd44) :
                      7'd41;
        end
        end // KEEP_E93_DEBUG_UI

    // P2.3 minimal MIC pin diagnostic is INSIDE this same combinational
    // always @* block that owns the pic signal (no procedural assignments
    // are allowed at Verilog module scope).

    // LEFT square = LRCL=0; RIGHT square = LRCL=1.
    // Slate = only zero, amber = only one, green = zero AND one observed.
    // Dark navy = no 85ms snapshot yet (or clock/reset not operating).
    // The colors indicate GPIO bit levels, NOT correct PCM audio.
    if (de && py>=9'd32 && py<9'd48) begin
        if (!E946_LARGE_PITCH_CARD &&
            px>=10'd359 && px<10'd415) begin
            // P3.4 compact fallback: same font layout, but smoothed note.
            // Dark box with '--' means no confident monophonic pitch.
            // This does NOT change audio output, track notes or keyboard.
            pic_tag=e946_note_visible ? 7'd10 : 7'd16;
            if (py==9'd32 || py==9'd47 ||
                px==10'd359 || px==10'd414)
                pic_tag=e946_note_visible ? 7'd40 : 7'd39;
            if (py>=9'd36 && py<9'd43) begin
                if (e946_note_visible) begin
                    if (px>=10'd368 && px<10'd373 &&
                        e943_shared_glyph[4-e943_glyph_col])
                        pic_tag=7'd86;
                    // Sharp symbol appears for C#,D#,F#,G#,A#.
                    if (e942_has_sharp &&
                        ((px==10'd379 || px==10'd382) ||
                         ((py==9'd38 || py==9'd40) &&
                          px>=10'd377 && px<=10'd384)) &&
                        px>=10'd377 && px<=10'd384)
                        pic_tag=7'd69;
                    if (px>=10'd391 && px<10'd396 &&
                        e943_shared_glyph[4-e943_glyph_col])
                        pic_tag=7'd86;
                end else if (py==9'd39 &&
                    ((px>=10'd369 && px<=10'd373) ||
                     (px>=10'd389 && px<=10'd393)))
                    pic_tag=7'd61; // waiting/silence: --
            end
        end else if (E942_SHOW_OLD_MIC_DEBUG && px>=10'd422 && px<10'd447) begin
            // C: source audio clock heartbeat (green/dark alternating).
            pic_tag=mic_frame_blink ? 7'd30 : 7'd16;
            // Minimal 7-pixel-high C, no ROM or per-pixel counters.
            if ((py==9'd36 || py==9'd43) &&
                px>=10'd430 && px<=10'd437)
                pic_tag=7'd86;
            if (px==10'd430 && py>=9'd36 && py<=9'd43)
                pic_tag=7'd86;
        end else if (E942_SHOW_OLD_MIC_DEBUG && px>=10'd452 && px<10'd477) begin
            // D: at least two different 16-bit sample words.
            pic_tag=mic_word_changed ? 7'd30 : 7'd48;
            if (px==10'd460 && py>=9'd36 && py<=9'd43)
                pic_tag=7'd86;
            if ((py==9'd36 || py==9'd43) &&
                px>=10'd460 && px<=10'd465)
                pic_tag=7'd86;
            if (px==10'd466 && py>=9'd38 && py<=9'd41)
                pic_tag=7'd86;
        end else if (px>=10'd482 && px<10'd507) begin
            // V: actual LEFT PCM peak-to-peak amplitude, 0 to 4 bars.
            case(e946_vu_draw)
                3'd4: pic_tag=7'd27;
                3'd3: pic_tag=7'd30;
                3'd2: pic_tag=7'd30;
                3'd1: pic_tag=7'd71;
                default: pic_tag=7'd48;
            endcase
            if (((px==10'd489 || px==10'd498) &&
                 py>=9'd36 && py<=9'd39) ||
                ((px==10'd491 || px==10'd496) &&
                 py>=9'd40 && py<=9'd41) ||
                ((px==10'd493 || px==10'd494) && py==9'd42))
                pic_tag=7'd86;
        end else if (E942_SHOW_OLD_MIC_DEBUG && px>=10'd512 && px<10'd540) begin
            case (mic_left_pin_state)
                2'b11: pic_tag=7'd30;  // both levels
                2'b10: pic_tag=7'd71;  // high only
                2'b01: pic_tag=7'd48;  // low only
                default: pic_tag=7'd15;
            endcase
            // Tiny 'L' label without a character ROM.
            if ((px==10'd522 && py>=9'd36 && py<=9'd43) ||
                (py==9'd43 && px>=10'd522 && px<=10'd528))
                pic_tag=7'd86;
        end else if (E942_SHOW_OLD_MIC_DEBUG && px>=10'd547 && px<10'd575) begin
            case (mic_right_pin_state)
                2'b11: pic_tag=7'd30;
                2'b10: pic_tag=7'd71;
                2'b01: pic_tag=7'd48;
                default: pic_tag=7'd15;
            endcase
            // Tiny 'R' label.
            if ((px==10'd557 && py>=9'd36 && py<=9'd43) ||
                ((py==9'd36 || py==9'd39) &&
                 px>=10'd557 && px<=10'd563) ||
                (px==10'd563 && py>=9'd37 && py<=9'd38) ||
                (px==10'd560 && py==9'd40) ||
                (px==10'd562 && py>=9'd41 && py<=9'd43))
                pic_tag=7'd86;
        end else if (px>=10'd515 && px<10'd563) begin
            // E963 MIC VU: compact four-LED strip NEXT TO the V mic icon
            // at x482..506. The previous isolated x578..597 floating
            // strip was hard to read and overlapped editor status.
            // No change to SPH0645 I2S, AMDF or volume smoothing.
            pic_tag=7'd11;
            if (py>=9'd35 && py<9'd45) begin
                if (px>=10'd518 && px<10'd526)
                    pic_tag=(e946_vu_draw>=3'd1) ?
                        7'd68 : 7'd28;
                else if (px>=10'd529 && px<10'd537)
                    pic_tag=(e946_vu_draw>=3'd2) ?
                        7'd55 : 7'd28;
                else if (px>=10'd540 && px<10'd548)
                    pic_tag=(e946_vu_draw>=3'd3) ?
                        7'd36 : 7'd28;
                else if (px>=10'd551 && px<10'd559)
                    pic_tag=(e946_vu_draw>=3'd4) ?
                        7'd23 : 7'd28;
            end
        end
    end

    // P3.4: more readable pitch card, DRAWN AFTER the existing VU boxes.
    // Cropped strictly to x348..443 y26..51, does not touch C/D/V or BMP
    // outside this rectangle. All glyphs share the digit_row ROM.
    if (E946_LARGE_PITCH_CARD && de &&
        px>=10'd348 && px<10'd444 &&
        py>=9'd26 && py<9'd52) begin
        // Held notes are dimmer, but do not jump back to "--" immediately.
        pic_tag=e946_note_visible ?
             (e946_note_recent ? 7'd4 : 7'd18) :
             7'd12;
        if (px==10'd348 || px==10'd443 ||
            py==9'd26 || py==9'd51)
            pic_tag=e946_note_visible ?
                 (e946_note_recent ? 7'd43 : 7'd51) :
                 7'd42;

        if (py>=9'd31 && py<9'd45) begin
            if (e946_note_visible) begin
                // Two horizontal pixels per glyph bit, two scanlines per
                // glyph row. x360..369 = pitch letter, x412..421 = octave.
                if (px>=10'd360 && px<10'd370 &&
                    e943_shared_glyph[4-e943_glyph_col])
                    pic_tag=e946_note_recent ? 7'd86 : 7'd65;

                // Bright 2x scale sharp sign; do not emit # for natural
                // notes. Retain letter+octave when sharp is absent.
                if (e942_has_sharp &&
                    ((px>=10'd384 && px<=10'd385) ||
                     (px>=10'd390 && px<=10'd391) ||
                     ((py==9'd35 || py==9'd36 ||
                       py==9'd40 || py==9'd41) &&
                      px>=10'd381 && px<=10'd394)) &&
                    px>=10'd381 && px<=10'd394)
                    pic_tag=e946_note_recent ? 7'd72 : 7'd62;

                if (px>=10'd412 && px<10'd422 &&
                    e943_shared_glyph[4-e943_glyph_col])
                    pic_tag=e946_note_recent ? 7'd86 : 7'd65;
            end else if (py>=9'd37 && py<=9'd38 &&
                         ((px>=10'd361 && px<=10'd370) ||
                          (px>=10'd413 && px<=10'd422))) begin
                pic_tag=7'd59; // no confident note: --
            end
        end
        // Small stable/hold indicator. No animation clock or pixel RAM.
        if (px>=10'd429 && px<10'd435 &&
            py>=9'd35 && py<9'd42)
            pic_tag=!e946_note_visible ? 7'd39 :
                  e946_note_recent ? 7'd33 : 7'd58;
    end

    // Header status: left=metronome amber, middle=KBD REC red,
    // right=MIC REC cyan. The onboard KEY4 toggles MIC (independent
    // of expansion K3 keyboard recording). All 3 are visible at once.
    if (de && editor_on && py>=9'd35 && py<9'd45) begin
        if (px>=10'd578 && px<10'd588)
            pic_tag=metro_sound_enabled ? 7'd75 : 7'd37;
        if (px>=10'd602 && px<10'd612)
            pic_tag=rec_active ? 7'd73 : 7'd37;
        if (px>=10'd626 && px<10'd636)
            pic_tag=e952_mic_rec_enabled ? 7'd32 : 7'd37;
    end

    // E965: third-option palette itself indicates bank readiness.
    // No overlap with its Chinese label and no added horizontal line.

    // E951 AREA: light-weight, real 4-track picture Piano Roll.
    // The old compositor contained multiple 24-bit nested color muxes.
    // This version uses one note-color selection, one cursor predicate
    // and one shared 20:1 note bitmap selector. No new RAM.
    if (de && e950_mini_active &&
        px>=10'd336 && px<10'd625 &&
        py>=9'd300 && py<9'd453) begin
        pic_tag=7'd2;
        if (py==9'd300 || py==9'd452 ||
            px==10'd336 || px==10'd624)
            pic_tag=7'd47;

        // Record and metronome controls remain visible in the title bar.
        if (py>=9'd305 && py<9'd315) begin
            // Red=mini auto-capture is ACTIVE only with SW1=1.
            // When paused, notes can be viewed and direct keys still sound.
            if (px>=10'd571 && px<10'd582)
                pic_tag=e958_transport_play ? 7'd73 : 7'd37;
            if (px>=10'd594 && px<10'd605)
                pic_tag=sw2 ? 7'd75 : 7'd38;
        end

        if (py>=9'd320 && py<9'd448) begin
            // Four lanes: each 32 pixels high, 5 note bands per lane.
            if (e950_mini_dy[4:0]==5'd0)
                pic_tag=7'd49;
            if (px>=10'd340 && px<10'd347 &&
                e950_mini_lane==track_sel)
                pic_tag=7'd76;

            if (px>=10'd352 && px<10'd608 &&
                e950_mini_band<3'd5) begin
                pic_tag=e950_mini_note ?
                    (track_mute[e950_mini_lane] ?
                     7'd53 : 7'd87) :
                    7'd13;
                // Time follows real BPM play/record head automatically;
                // no manual horizontal cursor is required when playing.
                if (e950_mini_col==play_col && px[1:0]<2'd2)
                    pic_tag=7'd77;
            end
        end
    end

    // E963: unobtrusive solid piano blocks, no underlining artifact.
    // The full-screen BMP remains untouched above y=456.  Each of 25
    // pitch blocks is 15x19px with a one-pixel gap (not an outline).
    if(de && !editor_on && e952_sw3_ff[1]) begin
        if(e962_roll_hit)
            pic_tag=7'd25; // continuous 4x4 cyan history blocks
        if(px>=10'd96 && px<10'd496 &&
           py>=9'd457 && py<9'd477 &&
           e962_tile_dx[3:0]<4'd15) begin
            pic_tag=e962_tile_pressed ? 7'd82 : 7'd24;
        end
        // The former y=454 horizontal divider has been removed.
    end
    // E982 actual SPH0645 MIC 8-band frequency spectrum (Goertzel DFT).
    // Source is I2S MIC pcm12, NEVER the HDMI output mix.
    // No second framebuffer. SW3=1 original solid keyboard history
    // still owns the bottom-left area. MINI x336..624 is untouched.
    if(de && e952_image_key_mode &&
       px>=10'd24 && px<10'd306 &&
       py>=9'd388 && py<9'd453) begin
        pic_tag=7'd2;
        if(px>=10'd27 && px<10'd30)
            pic_tag=7'd25;
        // 8 columns: 94,188,281,375,469,562,750,1031 Hz at 3kHz.
        // Height comes from the VIDEO-LATCHED Goertzel DFT results.
        if(px>=10'd38 && px<10'd286 &&
           px[4:0]>=5'd7 && px[4:0]<5'd29 &&
           py>=9'd416 && py<9'd447 &&
           py>=e982_spec_top_y)
            pic_tag=(e982_spec_col[0]) ? 7'd25 : 7'd55;
        // Bottom 8 subtle reference ticks, not a long divider.
        if(py==9'd448 && px>=10'd38 && px<10'd286 &&
           px[4:0]==5'd16)
            pic_tag=7'd34;
        // "音频" instead of misleading "FFT": Goertzel gives
        // real 8 frequency bins, NOT full radix-2 FFT.
        if(py>=9'd393 && py<9'd409 &&
           px>=10'd34 && px<10'd66) begin
            cjk_on=1'b1;
            cjk_id=(px<10'd50) ? 5'd0 : 5'd30;
            glyph_x=(px-10'd34)&10'hf;
            glyph_y=py-9'd393;
            overlay_color=24'h62e6e9;
        end
    end

    end // end of the pic always @* block; includes P2.5 audio meter
    // Align CJK ROM row and the saved original background pixel.
    // This matches every original Chinese pixel, just 1 video tick
    // later; the top-level AXI input DE/VS are delayed in lockstep.
    // Registered palette value maps back to EXACT original RGB colors.
    // The ROM has at most 89 tags; no UI color quantization.
    always @* begin
        case(e972_tag_q)
            7'd0:e972_palette_color=e972_dynamic_q;
            7'd1:e972_palette_color=24'h101b2b;
            7'd2:e972_palette_color=24'h162b3d;
            7'd3:e972_palette_color=24'h172c42;
            7'd4:e972_palette_color=24'h174c47;
            7'd5:e972_palette_color=24'h18344f;
            7'd6:e972_palette_color=24'h192b3d;
            7'd7:e972_palette_color=24'h192f43;
            7'd8:e972_palette_color=24'h1b3045;
            7'd9:e972_palette_color=24'h1c3144;
            7'd10:e972_palette_color=24'h1f625b;
            7'd11:e972_palette_color=24'h203b4c;
            7'd12:e972_palette_color=24'h223344;
            7'd13:e972_palette_color=24'h22364b;
            7'd14:e972_palette_color=24'h223a51;
            7'd15:e972_palette_color=24'h233344;
            7'd16:e972_palette_color=24'h243a49;
            7'd17:e972_palette_color=24'h263d50;
            7'd18:e972_palette_color=24'h2d4147;
            7'd19:e972_palette_color=24'h2d4357;
            7'd20:e972_palette_color=24'h305a70;
            7'd21:e972_palette_color=24'h306e53;
            7'd22:e972_palette_color=24'h32495d;
            7'd23:e972_palette_color=24'h32a99b;
            7'd24:e972_palette_color=24'h34566a;
            7'd25:e972_palette_color=24'h35d5c8;
            7'd26:e972_palette_color=24'h385c50;
            7'd27:e972_palette_color=24'h38c7ae;
            7'd28:e972_palette_color=24'h395565;
            7'd29:e972_palette_color=24'h39b998;
            7'd30:e972_palette_color=24'h3bbd95;
            7'd31:e972_palette_color=24'h40586c;
            7'd32:e972_palette_color=24'h41d5df;
            7'd33:e972_palette_color=24'h44d3ac;
            7'd34:e972_palette_color=24'h47596a;
            7'd35:e972_palette_color=24'h486077;
            7'd36:e972_palette_color=24'h4cbea9;
            7'd37:e972_palette_color=24'h4d6276;
            7'd38:e972_palette_color=24'h516778;
            7'd39:e972_palette_color=24'h52697d;
            7'd40:e972_palette_color=24'h52cba5;
            7'd41:e972_palette_color=24'h536577;
            7'd42:e972_palette_color=24'h53697d;
            7'd43:e972_palette_color=24'h54cfaa;
            7'd44:e972_palette_color=24'h55d6a4;
            7'd45:e972_palette_color=24'h55e4d3;
            7'd46:e972_palette_color=24'h596878;
            7'd47:e972_palette_color=24'h5d8196;
            7'd48:e972_palette_color=24'h607487;
            7'd49:e972_palette_color=24'h627b90;
            7'd50:e972_palette_color=24'h62e6e9;
            7'd51:e972_palette_color=24'h657f84;
            7'd52:e972_palette_color=24'h66fa91;
            7'd53:e972_palette_color=24'h6b7687;
            7'd54:e972_palette_color=24'h725138;
            7'd55:e972_palette_color=24'h75cfaf;
            7'd56:e972_palette_color=24'h78e69a;
            7'd57:e972_palette_color=24'h80cfff;
            7'd58:e972_palette_color=24'h829697;
            7'd59:e972_palette_color=24'h899eae;
            7'd60:e972_palette_color=24'h8b302e;
            7'd61:e972_palette_color=24'h8da3b5;
            7'd62:e972_palette_color=24'hadbec0;
            7'd63:e972_palette_color=24'hc6ce88;
            7'd64:e972_palette_color=24'hc8deef;
            7'd65:e972_palette_color=24'hc9d6db;
            7'd66:e972_palette_color=24'hcf9fff;
            7'd67:e972_palette_color=24'hcfa2ff;
            7'd68:e972_palette_color=24'hdfae65;
            7'd69:e972_palette_color=24'he2f0cb;
            7'd70:e972_palette_color=24'he76666;
            7'd71:e972_palette_color=24'he7ad54;
            7'd72:e972_palette_color=24'he7f3d8;
            7'd73:e972_palette_color=24'hed655d;
            7'd74:e972_palette_color=24'hed675f;
            7'd75:e972_palette_color=24'hf4c15c;
            7'd76:e972_palette_color=24'hf7d48a;
            7'd77:e972_palette_color=24'hfce8a1;
            7'd78:e972_palette_color=24'hff384e;
            7'd79:e972_palette_color=24'hff9e7e;
            7'd80:e972_palette_color=24'hffb17d;
            7'd81:e972_palette_color=24'hffbe73;
            7'd82:e972_palette_color=24'hffcf71;
            7'd83:e972_palette_color=24'hffd78d;
            7'd84:e972_palette_color=24'hffe69a;
            7'd85:e972_palette_color=24'hfff4a3;
            7'd86:e972_palette_color=24'hffffff;
            7'd87:e972_palette_color=e972_dynamic_q;
            7'd88:e972_palette_color=e972_dynamic_q;
            default:e972_palette_color=24'd0;
        endcase
    end
    always @* begin
        video_out = (e971_cjk_on_q &&
                     e971_cjk_row_q[15-e971_glyph_x_q]) ?
                    e971_overlay_color_q : e972_palette_color;
    end

    function [23:0] lane_color_for_track;
        input [1:0] t;
        begin
            case(t)
                2'd0:lane_color_for_track=24'h55bdd7;
                2'd1:lane_color_for_track=24'he6a86e;
                2'd2:lane_color_for_track=24'haf8fdb;
                default:lane_color_for_track=24'h69cfaa;
            endcase
        end
    endfunction
endmodule

// =============================================================
// P2.3 minimal SPH0645 data-pin tester.
// Input: 12.29 MHz audio_mclk from the existing PLL.
// Outputs: ~3.074 MHz BCLK, ~48.03 kHz LRCLK.
// In each 32-bit slot, the first bit clock is the I2S delay;
// positions 1..24 represent the microphone's 24 data bits.
//
// State encodings per channel:
//   00: no snapshot / no samples
//   01: only logic 0 observed
//   10: only logic 1 observed
//   11: both observed
//
// No audio samples are saved, no pulse-rate/dB/PCM is calculated.
// This does NOT prove correct microphone audio, only GPIO levels.
// =============================================================
module e939_mic_bit_probe #(
    parameter ENABLE_METER = 1'b1,
    parameter integer WINDOW_BITS = 12,
    parameter [15:0] LEVEL1_P2P = 16'd120,
    parameter [15:0] LEVEL2_P2P = 16'd300,
    parameter [15:0] LEVEL3_P2P = 16'd750,
    parameter [15:0] LEVEL4_P2P = 16'd1800,
    parameter [15:0] PITCH_MIN_P2P = 16'd160
)(
    input  wire audio_mclk,
    input  wire rst,
    input  wire dout,
    output wire mic_bclk,
    output wire mic_lrcl,
    output reg [1:0] left_state,
    output reg [1:0] right_state,
    output reg frame_blink,      // toggles every 4096 I2S frames
    output reg word_changed,     // left PCM sample changed within last window
    output reg [2:0] voice_level, // 0..4 actual LEFT PCM amplitude bars
    // P3.0 shares this very same 12-bit PCM sample; no second I2S decoder.
    output wire signed [11:0] pcm12,
    output reg pcm_strobe,
    output reg signed [11:0] dc_center,
    output reg pitch_gate
);
    // Reuse the existing audio clock. 12.295MHz /4 /64 = ~48.03kHz.
    reg [1:0] bit_div;
    reg [5:0] slot;
    reg [WINDOW_BITS-1:0] frame_counter;
    reg [1:0] left_seen, right_seen;

    // Use only the upper 12 signed PCM bits (slots 1..12).
    // This is a compact VU/diagnostic source, not a full audio decoder.
    // IMPORTANT: Ignore RIGHT unselected (high-impedance) half-frame!
    reg [11:0] sample_shift;
    wire [11:0] new_sample = {sample_shift[10:0], dout};
    reg [11:0] last_left;
    reg changed_in_window;

    // Peak-to-peak amplitude of the left PCM waveform over 1024 samples
    // (~21.3ms). Unlike "count of changing samples", this does not
    // saturate from low-amplitude background noise. Subtracting min/max
    // automatically cancels constant microphone DC bias.
    reg signed [11:0] min_pcm;
    reg signed [11:0] max_pcm;
    wire [11:0] p2p12 = max_pcm - min_pcm;
    wire [15:0] p2p = {p2p12, 4'b0000}; // approximate 16-bit units
    wire [2:0] level_target =
         (p2p >= LEVEL4_P2P) ? 3'd4 :
         (p2p >= LEVEL3_P2P) ? 3'd3 :
         (p2p >= LEVEL2_P2P) ? 3'd2 :
         (p2p >= LEVEL1_P2P) ? 3'd1 : 3'd0;
    reg [1:0] decay_windows;
    wire signed [12:0] center_sum =
        $signed({min_pcm[11], min_pcm}) +
        $signed({max_pcm[11], max_pcm});
    assign pcm12 = $signed(last_left);

    assign mic_bclk = bit_div[1];   // approximately 3.074 MHz
    assign mic_lrcl = slot[5];      // approximately 48.03 kHz

    always @(posedge audio_mclk or posedge rst) begin
        if (rst) begin
            bit_div <= 2'd0;
            slot <= 6'd0;
            frame_counter <= {WINDOW_BITS{1'b0}};
            left_seen <= 2'd0;
            right_seen <= 2'd0;
            left_state <= 2'd0;
            right_state <= 2'd0;
            frame_blink <= 1'b0;
            sample_shift <= 12'd0;
            last_left <= 12'd0;
            changed_in_window <= 1'b0;
            word_changed <= 1'b0;
            min_pcm <= 12'sh7ff;
            max_pcm <= 12'sh800;
            voice_level <= 3'd0;
            decay_windows <= 2'd0;
            pcm_strobe <= 1'b0;
            dc_center <= 12'sd0;
            pitch_gate <= 1'b0;
        end else begin
            bit_div <= bit_div + 2'd1;
            pcm_strobe <= 1'b0;
            // At bit_div=3 we create BCLK's falling edge. The mic changes
            // DOUT following BCLK's rising edge, giving a half-cycle
            // data-settling interval. Preserve P2.4's proven pin timing.
            if (bit_div == 2'd3) begin
                slot <= slot + 6'd1;

                // L/R checks use only 24 I2S data bits in each half-frame.
                if (slot[4:0] >= 5'd1 && slot[4:0] <= 5'd24) begin
                    if (slot[5]) begin
                        if (dout) right_seen[1] <= 1'b1;
                        else      right_seen[0] <= 1'b1;
                    end else begin
                        if (dout) left_seen[1] <= 1'b1;
                        else      left_seen[0] <= 1'b1;
                    end
                end

                // Measure only the physically selected LEFT microphone.
                // This avoids counting the idle/Hi-Z right slot as sound.
                if (!slot[5] &&
                    slot[4:0] >= 5'd1 && slot[4:0] <= 5'd12) begin
                    sample_shift <= new_sample;
                    if (slot[4:0] == 5'd12) begin
                        if (new_sample != last_left)
                            changed_in_window <= 1'b1;
                        last_left <= new_sample;
                        pcm_strobe <= 1'b1;
                        if (ENABLE_METER) begin
                            if ($signed(new_sample) < min_pcm)
                                min_pcm <= $signed(new_sample);
                            if ($signed(new_sample) > max_pcm)
                                max_pcm <= $signed(new_sample);
                        end
                    end
                end

                if (slot == 6'd63) begin
                    frame_counter <= frame_counter + 1'b1;

                    // Four-meter-bar refresh: once per 1024 frames (21ms).
                    // Reuse the bottom 10 bits of the existing frame counter;
                    // no extra audio sample counter or large sample buffer.
                    if (&frame_counter[9:0]) begin
                        min_pcm <= 12'sh7ff;
                        max_pcm <= 12'sh800;
                        // DC offset is the mid-point of the previous
                        // 21ms block. A constant bias should not be
                        // mistaken for positive/negative voice crossings.
                        dc_center <= center_sum[12:1];
                        pitch_gate <= ENABLE_METER &&
                                     (p2p >= PITCH_MIN_P2P);
                        if (!ENABLE_METER) begin
                            voice_level <= 3'd0;
                            decay_windows <= 2'd0;
                        end else if (level_target > voice_level) begin
                            // Attack quickly as the sound gets louder.
                            voice_level <= level_target;
                            decay_windows <= 2'd0;
                        end else if (level_target == voice_level) begin
                            decay_windows <= 2'd0;
                        end else if (decay_windows >= 2'd2) begin
                            // Release one bar in ~64ms, avoids flicker.
                            voice_level <= voice_level - 3'd1;
                            decay_windows <= 2'd0;
                        end else begin
                            decay_windows <= decay_windows + 2'd1;
                        end
                    end

                    // Keep C/D/L/R status update at 4096 frames (~85ms).
                    if (&frame_counter) begin
                        left_state <= left_seen;
                        right_state <= right_seen;
                        left_seen <= 2'd0;
                        right_seen <= 2'd0;
                        frame_blink <= ~frame_blink;
                        word_changed <= changed_in_window;
                        changed_in_window <= 1'b0;
                    end
                end
            end
        end
    end
endmodule
// ============================================================================
// P3.0 / C3..C6 low-area single-pitch estimator (MONOPHONIC)
//
// One 48kHz LEFT PCM12 sample enters per sample_valid pulse, already supplied
// by e939_mic_bit_probe. No full waveform RAM, FFT, division or DSP.
//
// DC removal: subtract the P2.5 min/max midpoint each 21ms.
// Harmonic reduction: one-pole Q3 low-pass (shift-only).
// Pitch: hysteretic negative->positive crossings, intervals in SAMPLE clocks.
// Quantization: normalize periods into C4-B4 range, sequentially check
// 12 note classes using a SINGLE 8-bit comparator (one threshold per MCLK).
// Stability: 4 matching period candidates before asserting pitch_valid.
//
// pitch_code = {octave_index[1:0],semitone[3:0]}
//   octave_index 0,1,2,3 maps to C3,C4,C5,C6
//   semitone 0..11 maps to C,C#,D,D#,E,F,F#,G,G#,A,A#,B.
// Only C6 is included above B5.
// Intended as first-stage pitch proof-of-concept, NOT polyphonic analysis.
// ============================================================================
module e942_mic_period_pitch(
    input  wire clk,                  // ~12.295 MHz existing audio MCLK
    input  wire rst,
    input  wire signed [11:0] pcm12, // P2.5 LEFT I2S sample
    input  wire sample_valid,         // 1 cycle per 48.03k LEFT sample
    input  wire signed [11:0] center12,
    input  wire energy_ok,            // P2.5 amplitude gate
    output reg [5:0] pitch_code,      // {octave_idx[1:0], semitone[3:0]}
    output reg pitch_valid
);
    // ==================================================================
    // P3.2 monophonic piano + voice fundamental frequency estimator.
    // 48k PCM -> 24k (average pairs) -> double-buffered 512x11 RAM.
    // Sequential AMDF: lags 23..184, 128 pairs per lag, TWO PASSES.
    //   pass0: find global best + confidence baseline
    //   pass1: earliest local difference-function minimum <= 1.5x best
    // Excludes transient-dominated frames with no strong periodicity.
    // All lag comparisons share ONE signed subtraction/abs/accumulator.
    // No FFT, multiplier, DSP or 128 parallel correlators are instantiated.
    // Critical: sample_history MUST map to EMB block RAM, not LUT registers.
    // ==================================================================

    // Signed center-corrected sample and 2:1 averaging. Sum/8 yields
    // signed 11-bit data even for the most extreme PCM12/DC combination.
    wire signed [12:0] dc_sample =
        $signed({pcm12[11],pcm12}) -
        $signed({center12[11],center12});
    reg signed [12:0] prev_pcm;
    wire signed [13:0] pair_sum =
        $signed({prev_pcm[12],prev_pcm}) +
        $signed({dc_sample[12],dc_sample});
    wire signed [10:0] reduced_pcm = pair_sum >>> 3;

    reg half_sample;
    reg [8:0] write_pos;             // 0..511
    reg write_bank;
    reg completed_toggle;
    reg completed_bank;
    wire sample_write = sample_valid && half_sample;

    // Exactly 11 * 1024 = 11,264 bits of sample history.
    // Synchronous-read simple dual-port RAM inference pattern.
    // No initial/clear loops on the RAM array (would cause FF/LUT use).
    reg signed [10:0] sample_history [0:1023];
    reg signed [10:0] read_pcm;
    reg [9:0] read_addr;
    always @(posedge clk) begin
        if (sample_write)
            sample_history[{write_bank,write_pos}] <= reduced_pcm;
        read_pcm <= sample_history[read_addr];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            half_sample <= 1'b0;
            prev_pcm <= 13'sd0;
            write_pos <= 9'd0;
            write_bank <= 1'b0;
            completed_bank <= 1'b0;
            completed_toggle <= 1'b0;
        end else if (sample_valid) begin
            prev_pcm <= dc_sample;
            half_sample <= ~half_sample;
            if (half_sample) begin
                write_pos <= write_pos + 9'd1;
                if (&write_pos) begin
                    completed_bank <= write_bank;
                    completed_toggle <= ~completed_toggle;
                    write_bank <= ~write_bank;
                end
            end
        end
    end

    localparam [3:0]
        S_IDLE     = 4'd0,
        S_READ_A   = 4'd1,
        S_WAIT_A   = 4'd2,
        S_READ_B   = 4'd3,
        S_WAIT_B   = 4'd4,
        S_DIFF     = 4'd5,
        S_FIN_LAG  = 4'd6,
        S_LAST_LAG = 4'd7,
        S_GATE     = 4'd8,
        S_NOTE     = 4'd9,
        S_SEMITONE = 4'd10;

    reg [3:0] state;
    reg seen_completed;
    reg analysis_bank;
    reg analysis_gate;
    reg pass;                       // 0: find best, 1: choose first valley
    reg [7:0] lag;                  // 23..184
    reg [6:0] pair_index;           // 0..127
    reg [8:0] ref_index, delay_index;
    reg signed [10:0] sample_a;
    reg [17:0] accumulated;
    // P3.3: retain full 18bit accumulator, trim 6 fractional
    // least significant bits of each lag score (relative AMDF only).
    reg [11:0] lag_cost;
    reg [11:0] raw_prev2, raw_prev1;

    wire signed [11:0] signed_delta =
        $signed({sample_a[10],sample_a}) -
        $signed({read_pcm[10],read_pcm});
    wire [11:0] absolute_delta =
        signed_delta[11] ? -signed_delta : signed_delta;
    wire [17:0] accumulated_next =
        accumulated + {6'd0,absolute_delta};

    // Smoothed AMDF in units of 4: cost[l-1]+2*cost[l]+cost[l+1]
    // Smoothing helps prevent ±1 sample quantization at higher notes
    // from looking worse than a doubled (lower-octave) period.
    wire [13:0] edge_cost = {lag_cost,2'b00};
    wire [13:0] smoothed_cost =
        {2'b00,raw_prev2} +
        {1'b0,raw_prev1,1'b0} +
        {2'b00,lag_cost};

    reg [13:0] best_cost;
    reg [7:0] best_lag;
    reg [15:0] baseline_sum;       // 4 spaced lags' AMDF scores
    wire [16:0] best_times_five =
        {1'b0,best_cost,2'b00} + {3'b000,best_cost};
    wire confident = analysis_gate &&
                     (best_times_five < {1'b0,baseline_sum});
    wire [14:0] valley_limit =
        {1'b0,best_cost} + ({1'b0,best_cost} >> 1);

    // During second pass, keep only 2 recent smoothed costs to find
    // local minima. No 162*20-bit score RAM is necessary.
    reg [13:0] sm_prev2, sm_prev1;
    reg first_found;
    reg [7:0] first_lag;
    reg [7:0] selected_lag;

    // Same 12-semitone frequency-bin table from P3.1, now fed by
    // the AMDF-estimated 24kHz sample period * 2.
    wire [8:0] selected_period48 = {selected_lag,1'b0};
    wire [1:0] selected_octave =
        (selected_period48 >= 9'd189) ? 2'd0 :
        (selected_period48 >= 9'd95)  ? 2'd1 :
        (selected_period48 >= 9'd47)  ? 2'd2 : 2'd3;
    wire [7:0] normalized_period =
        (selected_period48 >= 9'd189) ? selected_period48[8:1] :
        (selected_period48 >= 9'd95)  ? selected_period48[7:0] :
        (selected_period48 >= 9'd47)  ? {selected_period48[6:0],1'b0} :
                                       {selected_period48[5:0],2'b00};
    reg [7:0] normalized_hold;
    reg [1:0] octave_hold;
    reg [3:0] semitone_idx;
    reg [5:0] previous_note;
    wire [5:0] current_candidate = {octave_hold,semitone_idx};
    reg [7:0] threshold;
    always @* begin
        case (semitone_idx)
             4'd0: threshold=8'd178;
             4'd1: threshold=8'd168;
             4'd2: threshold=8'd159;
             4'd3: threshold=8'd150;
             4'd4: threshold=8'd142;
             4'd5: threshold=8'd134;
             4'd6: threshold=8'd126;
             4'd7: threshold=8'd119;
             4'd8: threshold=8'd112;
             4'd9: threshold=8'd106;
            4'd10: threshold=8'd100;
            default: threshold=8'd94;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            seen_completed <= 1'b0;
            analysis_bank <= 1'b0;
            analysis_gate <= 1'b0;
            pass <= 1'b0;
            lag <= 8'd23;
            pair_index <= 7'd0;
            ref_index <= 9'd511;
            delay_index <= 9'd488;
            sample_a <= 11'sd0;
            read_addr <= 10'd0;
            accumulated <= 18'd0;
            lag_cost <= 12'd0;
            raw_prev2 <= 12'd0;
            raw_prev1 <= 12'd0;
            best_cost <= 14'hffff;
            best_lag <= 8'd23;
            baseline_sum <= 16'd0;
            sm_prev2 <= 14'd0;
            sm_prev1 <= 14'd0;
            first_found <= 1'b0;
            first_lag <= 8'd23;
            selected_lag <= 8'd23;
            normalized_hold <= 8'd0;
            octave_hold <= 2'd0;
            semitone_idx <= 4'd0;
            previous_note <= 6'h3f;
            pitch_code <= 6'd0;
            pitch_valid <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (seen_completed != completed_toggle) begin
                        seen_completed <= completed_toggle;
                        analysis_bank <= completed_bank;
                        analysis_gate <= energy_ok;
                        if (!energy_ok) begin
                            pitch_valid <= 1'b0;
                            previous_note <= 6'h3f;
                            // Do not scan background noise unnecessarily.
                        end else begin
                            pass <= 1'b0;
                            lag <= 8'd23;
                            pair_index <= 7'd0;
                            ref_index <= 9'd511;
                            delay_index <= 9'd488;  // 511-23
                            accumulated <= 18'd0;
                            best_cost <= 14'hffff;
                            best_lag <= 8'd23;
                            baseline_sum <= 16'd0;
                            first_found <= 1'b0;
                            state <= S_READ_A;
                        end
                    end
                end

                S_READ_A: begin
                    read_addr <= {analysis_bank,ref_index};
                    state <= S_WAIT_A;
                end
                S_WAIT_A: state <= S_READ_B;
                S_READ_B: begin
                    sample_a <= read_pcm;
                    read_addr <= {analysis_bank,delay_index};
                    state <= S_WAIT_B;
                end
                S_WAIT_B: state <= S_DIFF;
                S_DIFF: begin
                    accumulated <= accumulated_next;
                    if (&pair_index) begin
                        lag_cost <= accumulated_next[17:6];
                        state <= S_FIN_LAG;
                    end else begin
                        pair_index <= pair_index + 7'd1;
                        ref_index <= ref_index - 9'd1;
                        delay_index <= delay_index - 9'd1;
                        state <= S_READ_A;
                    end
                end

                S_FIN_LAG: begin
                    if (lag == 8'd23) begin
                        raw_prev2 <= lag_cost;
                        if (!pass) begin
                            best_cost <= edge_cost;
                            best_lag <= 8'd23;
                        end else begin
                            sm_prev2 <= edge_cost;
                        end
                    end else if (lag == 8'd24) begin
                        raw_prev1 <= lag_cost;
                    end else begin
                        // Interior lag score is centered on lag-1.
                        raw_prev2 <= raw_prev1;
                        raw_prev1 <= lag_cost;
                        if (!pass) begin
                            if (smoothed_cost < best_cost) begin
                                best_cost <= smoothed_cost;
                                best_lag <= lag - 8'd1;
                            end
                            // Four reference lag costs spread across C3..C6
                            // distinguish periodic notes from broadband noise.
                            if (lag == 8'd44 || lag == 8'd84 ||
                                lag == 8'd124 || lag == 8'd164)
                                baseline_sum <= baseline_sum +
                                                {2'b00,smoothed_cost};
                        end else if (lag == 8'd25) begin
                            if ((sm_prev2 <= smoothed_cost) &&
                                ({1'b0,sm_prev2} <= valley_limit)) begin
                                first_found <= 1'b1;
                                first_lag <= 8'd23;
                            end
                            sm_prev1 <= smoothed_cost;
                        end else begin
                            // First sufficiently good local minimum wins,
                            // rather than the minimum at 2x the period.
                            if (!first_found &&
                                sm_prev1 <= sm_prev2 &&
                                sm_prev1 <= smoothed_cost &&
                                {1'b0,sm_prev1} <= valley_limit) begin
                                first_found <= 1'b1;
                                first_lag <= lag - 8'd2;
                            end
                            sm_prev2 <= sm_prev1;
                            sm_prev1 <= smoothed_cost;
                        end
                    end

                    if (lag == 8'd184) begin
                        state <= S_LAST_LAG;
                    end else begin
                        lag <= lag + 8'd1;
                        pair_index <= 7'd0;
                        ref_index <= 9'd511;
                        delay_index <= 9'd510 - {1'b0,lag};
                        accumulated <= 18'd0;
                        state <= S_READ_A;
                    end
                end

                S_LAST_LAG: begin
                    if (!pass) begin
                        // Endpoint lag=184 is not an interior smoothed score.
                        if (edge_cost < best_cost) begin
                            best_cost <= edge_cost;
                            best_lag <= 8'd184;
                        end
                        state <= S_GATE;
                    end else begin
                        if (first_found)
                            selected_lag <= first_lag;
                        else if (sm_prev1 <= sm_prev2 &&
                                 sm_prev1 <= edge_cost &&
                                 {1'b0,sm_prev1} <= valley_limit)
                            selected_lag <= 8'd183;
                        else if (edge_cost <= sm_prev1 &&
                                 {1'b0,edge_cost} <= valley_limit)
                            selected_lag <= 8'd184;
                        else
                            selected_lag <= best_lag;
                        state <= S_NOTE;
                    end
                end

                S_GATE: begin
                    if (!confident) begin
                        pitch_valid <= 1'b0;
                        previous_note <= 6'h3f;
                        state <= S_IDLE;
                    end else begin
                        // Second pass reuses exactly the same AMDF engine;
                        // total latency ~17ms < 21.3ms ping-pong frame time.
                        pass <= 1'b1;
                        lag <= 8'd23;
                        pair_index <= 7'd0;
                        ref_index <= 9'd511;
                        delay_index <= 9'd488;
                        accumulated <= 18'd0;
                        first_found <= 1'b0;
                        state <= S_READ_A;
                    end
                end

                S_NOTE: begin
                    octave_hold <= selected_octave;
                    normalized_hold <= normalized_period;
                    semitone_idx <= 4'd0;
                    state <= S_SEMITONE;
                end
                S_SEMITONE: begin
                    if (normalized_hold >= threshold ||
                        semitone_idx == 4'd11) begin
                        // Two consecutive valid 21ms frames are necessary.
                        // A new note remains blank until confirmed.
                        if (previous_note == current_candidate) begin
                            pitch_code <= current_candidate;
                            pitch_valid <= 1'b1;
                        end else begin
                            previous_note <= current_candidate;
                            pitch_valid <= 1'b0;
                        end
                        state <= S_IDLE;
                    end else
                        semitone_idx <= semitone_idx + 4'd1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

// ================================================================
// E962: FULL-SCREEN 640x480 zoom with only ONE 320-pixel line cache.
// Original video_delay passthrough behavior is preserved for 1x.
//
// Data contract (identical to original video_delay):
//   read_en sampled while de_d[18], async FIFO data ready one cycle
//   later; vout_data captures data while de_d[19], sync is de_d[20].
// 2x: each 320-source-pixel row is read once, shown in two output rows.
// 4x: each 160-source-pixel row is read once, shown in four output rows.
// Never stores a new 640x480 framebuffer or re-issues an SDRAM request.
// ================================================================
module e962_full_screen_zoom_video (
    input wire video_clk, rst,
    input wire hs, vs, de,
    input wire [23:0] read_data,
    input wire [1:0] zoom_level, // frame latched: 0=1x,1=2x,2=4x
    output wire read_en,
    output wire hs_r, vs_r, de_r,
    output reg [23:0] vout_data
);
    reg [20:0] hs_d, vs_d, de_d;
    assign hs_r=hs_d[20];
    assign vs_r=vs_d[20];
    assign de_r=de_d[20];

    reg [9:0] source_display_x;
    reg [8:0] source_display_y;
    reg was_displaying;
    reg [9:0] x_at_pixel;
    reg [8:0] y_at_pixel;
    reg [23:0] zoom_repeat_pixel;

    wire rd_stage = de_d[18];
    wire [1:0] e962_phase_x = source_display_x[1:0];
    wire [1:0] e962_phase_y = source_display_y[1:0];
    wire [1:0] e962_pixel_phase_x = x_at_pixel[1:0];
    wire [1:0] e962_pixel_phase_y = y_at_pixel[1:0];
    wire want_source_pixel =
        zoom_level==2'd0 ? 1'b1 :
        zoom_level==2'd1 ?
            ((e962_phase_x[0]==1'b0) &&
             (e962_phase_y[0]==1'b0)) :
            ((e962_phase_x==2'b00) && (e962_phase_y==2'b00));
    assign read_en = rd_stage && want_source_pixel;

    // One synchronous read port + one synchronous write port.
    // At zoom 2x 320 pixels per source row; at 4x 160 per row.
    // Only the source row is cached; the original framebuffer is SDRAM.
    (* ram_style = "block" *) reg [23:0] zoom_line[0:319];
    reg [23:0] zoom_line_q;
    wire [8:0] line_index_at_rd =
        (zoom_level==2'd2) ? {1'b0,source_display_x[9:2]} :
                             source_display_x[9:1];
    wire [8:0] line_index_at_pixel =
        (zoom_level==2'd2) ? {1'b0,x_at_pixel[9:2]} :
                             x_at_pixel[9:1];
    wire first_source_row_pixel =
        (zoom_level==2'd1) ?
           ((e962_pixel_phase_y[0]==1'b0) &&
            (e962_pixel_phase_x[0]==1'b0)) :
           ((e962_pixel_phase_y==2'b00) &&
            (e962_pixel_phase_x==2'b00));
    wire repeat_same_source_line =
        zoom_level==2'd1 ? (e962_pixel_phase_y[0]==1'b0) :
                           (e962_pixel_phase_y==2'b00);

    always @(posedge video_clk) begin
        zoom_line_q <= zoom_line[line_index_at_rd];
        if(!rst && de_d[19] && zoom_level!=2'd0 &&
            first_source_row_pixel)
            zoom_line[line_index_at_pixel] <= read_data;
    end

    always @(posedge video_clk or posedge rst) begin
        if(rst) begin
            hs_d <= 21'd0;
            vs_d <= 21'd0;
            de_d <= 21'd0;
            source_display_x <= 10'd0;
            source_display_y <= 9'd0;
            was_displaying <= 1'b0;
            x_at_pixel <= 10'd0;
            y_at_pixel <= 9'd0;
            zoom_repeat_pixel <= 24'd0;
            vout_data <= 24'd0;
        end else begin
            hs_d <= {hs_d[19:0],hs};
            vs_d <= {vs_d[19:0],vs};
            de_d <= {de_d[19:0],de};

            was_displaying <= rd_stage;
            if(!vs_d[18])
                source_display_y <= 9'd0;
            else if(was_displaying && !rd_stage &&
                    source_display_y < 9'd479)
                source_display_y <= source_display_y + 9'd1;

            if(rd_stage) begin
                if(!was_displaying)
                    source_display_x <= 10'd1;
                else if(source_display_x<10'd639)
                    source_display_x <= source_display_x + 10'd1;
            end else
                source_display_x <= 10'd0;

            x_at_pixel <= source_display_x;
            y_at_pixel <= source_display_y;

            if(de_d[19]) begin
                if(zoom_level==2'd0) begin
                    // Byte- and clock-compatible with video_delay.v.
                    vout_data <= read_data;
                end else if(first_source_row_pixel) begin
                    // FIFO returned a NEW source pixel one clock after
                    // read_en. Use it and cache it for repeated rows.
                    vout_data <= read_data;
                    zoom_repeat_pixel <= read_data;
                end else if(repeat_same_source_line) begin
                    // 2x/4x horizontal replication, no FIFO pop.
                    vout_data <= zoom_repeat_pixel;
                end else begin
                    // Vertical replication from the previous source row.
                    vout_data <= zoom_line_q;
                end
            end else
                vout_data <= 24'd0;
        end
    end
endmodule

// ================================================================
// P4.8 / E982 MIC EIGHT-FREQUENCY GOERTZEL DFT SPECTRUM
// This computes REAL bins from the actual SPH0645 I2S 48k PCM,
// not random animations and not an FFT claim.
// Inputs: 48kHz mono mic samples from existing E939; decimate /16
// -> 3kHz; 32 samples/window -> 93.75Hz frequency bins.
// k=1,2,3,4,5,6,8,11: 94..1031Hz. Fixed point Q8 coefficients.
// ONE time-shared signed 18x10 multiply, 8*(2*18) state registers,
// no new FFT sample RAM or video/audio I2S decoder.
// Re/Im calculated from Goertzel final states with the SAME
// multiplier; visual height is L1 approximation |Re|+|Im|.
// Does not synthesize a true radix-2 FFT. It IS a 32-point
// Goertzel DFT-bin analyzer, appropriate for "频谱或波形".
// ================================================================
module e982_mic_goertzel8 (
    input  wire clk, rst,
    input  wire signed [11:0] pcm12,
    input  wire signed [11:0] center12,
    input  wire pcm_strobe,
    input  wire energy_gate,
    output reg [39:0] band_levels,
    output reg frame_toggle
);
    localparam [2:0] ST_IDLE=3'd0, ST_UPDATE=3'd1,
                     ST_REAL=3'd2, ST_IMAG=3'd3,
                     ST_NEXT=3'd4;
    reg [2:0] state, bin_index;
    reg [3:0] decimate;
    reg [4:0] window_index;
    reg signed [12:0] sample_hold;
    reg signed [17:0] s1[0:7], s2[0:7];
    reg signed [19:0] real_hold;
    integer init_i;
    function signed [9:0] e982_c2;
        input [2:0] bin_idx_in;
        begin
            case(bin_idx_in)
               3'd0:e982_c2=10'sd502;
               3'd1:e982_c2=10'sd473;
               3'd2:e982_c2=10'sd426;
               3'd3:e982_c2=10'sd362;
               3'd4:e982_c2=10'sd284;
               3'd5:e982_c2=10'sd196;
               3'd6:e982_c2=10'sd0;
               3'd7:e982_c2=-10'sd284;
                default:e982_c2=10'sd0;
            endcase
        end
    endfunction
    function signed [9:0] e982_cos;
        input [2:0] bin_idx_in;
        begin
            case(bin_idx_in)
               3'd0:e982_cos=10'sd251;
               3'd1:e982_cos=10'sd237;
               3'd2:e982_cos=10'sd213;
               3'd3:e982_cos=10'sd181;
               3'd4:e982_cos=10'sd142;
               3'd5:e982_cos=10'sd98;
               3'd6:e982_cos=10'sd0;
               3'd7:e982_cos=-10'sd142;
                default:e982_cos=10'sd0;
            endcase
        end
    endfunction
    function signed [9:0] e982_sin;
        input [2:0] bin_idx_in;
        begin
            case(bin_idx_in)
               3'd0:e982_sin=10'sd50;
               3'd1:e982_sin=10'sd98;
               3'd2:e982_sin=10'sd142;
               3'd3:e982_sin=10'sd181;
               3'd4:e982_sin=10'sd213;
               3'd5:e982_sin=10'sd237;
               3'd6:e982_sin=10'sd256;
               3'd7:e982_sin=10'sd213;
                default:e982_sin=10'sd0;
            endcase
        end
    endfunction

    wire signed [12:0] centered =
        $signed({pcm12[11],pcm12}) -
        $signed({center12[11],center12});
    // Limit very loud mic transients before 32-sample resonance.
    // +/-255 keeps all 18-bit Goertzel delay states in range,
    // while weak voice/piano details are not shifted away.
    wire signed [12:0] sample_limited =
        (centered > 13'sd255) ? 13'sd255 :
        (centered < -13'sd255) ? -13'sd255 : centered;
    wire signed [17:0] a = s1[bin_index];
    wire signed [17:0] b = s2[bin_index];
    wire signed [9:0] mult_coef =
        (state==ST_UPDATE) ? e982_c2(bin_index) :
        (state==ST_REAL) ? e982_cos(bin_index) :
                           e982_sin(bin_index);
    wire signed [17:0] mult_operand =
        (state==ST_UPDATE) ? a : b;
    // One physical multiplier time-shared across all 8 bins & phases.
    wire signed [27:0] multiplied =
        $signed(mult_operand) * $signed(mult_coef);
    wire signed [19:0] product_q8 = $signed(multiplied) >>> 8;
    wire signed [19:0] sample_extended =
        {{7{sample_hold[12]}},sample_hold};
    wire signed [19:0] state_next =
        sample_extended + product_q8 - {{2{b[17]}},b};
    wire signed [19:0] real_next =
        {{2{a[17]}},a} - product_q8;
    wire signed [19:0] im_next = product_q8;
    wire [19:0] abs_real =
        real_hold[19] ? -real_hold : real_hold;
    wire [19:0] abs_im =
        im_next[19] ? -im_next : im_next;
    wire [20:0] mag_l1 =
        {1'b0,abs_real} + {1'b0,abs_im};
    // Exact DFT bin position, but intentionally L1 magnitude estimate
    // to avoid sqrt and two extra wide multipliers.
    wire [4:0] bin_height =
        (mag_l1 >= 21'd496) ? 5'd31 : mag_l1[8:4];
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=ST_IDLE;
            bin_index<=3'd0;
            decimate<=4'd0;
            window_index<=5'd0;
            sample_hold<=13'sd0;
            band_levels<=40'd0;
            frame_toggle<=1'b0;
            real_hold<=20'sd0;
            for(init_i=0;init_i<8;init_i=init_i+1) begin
                s1[init_i]<=18'sd0;
                s2[init_i]<=18'sd0;
            end
        end else begin
            // 48k -> 3k input. One computation uses <40 MCLK
            // cycles, while there are >4000 cycles per 3k sample.
            if(pcm_strobe) begin
                decimate<=decimate+4'd1;
                if(decimate==4'd0 && state==ST_IDLE) begin
                    sample_hold <= sample_limited;
                    bin_index <= 3'd0;
                    state <= ST_UPDATE;
                end
            end
            case(state)
                ST_IDLE: ;
                ST_UPDATE: begin
                    s2[bin_index] <= a;
                    s1[bin_index] <= state_next[17:0];
                    state <= (window_index==5'd31) ?
                             ST_REAL : ST_NEXT;
                end
                ST_REAL: begin
                    // s1=s(n-1), s2=s(n-2) *after* last update.
                    real_hold <= real_next;
                    state <= ST_IMAG;
                end
                ST_IMAG: begin
                    // L1 magnitude estimate of complex Goertzel
                    // DFT bin: real=s1-cos*s2, imag=sin*s2.
                    // Constant slices keep the 40bit output map a
                    // set of 8 tiny 5-bit registers (no 40-bit shifter).
                    case(bin_index)
                        3'd0:band_levels[ 4: 0] <= energy_gate ? bin_height : 5'd0;
                        3'd1:band_levels[ 9: 5] <= energy_gate ? bin_height : 5'd0;
                        3'd2:band_levels[14:10] <= energy_gate ? bin_height : 5'd0;
                        3'd3:band_levels[19:15] <= energy_gate ? bin_height : 5'd0;
                        3'd4:band_levels[24:20] <= energy_gate ? bin_height : 5'd0;
                        3'd5:band_levels[29:25] <= energy_gate ? bin_height : 5'd0;
                        3'd6:band_levels[34:30] <= energy_gate ? bin_height : 5'd0;
                        3'd7:band_levels[39:35] <= energy_gate ? bin_height : 5'd0;
                    endcase
                    // One block (32 samples) is complete: both
                    // delay elements reset before the next block.
                    s1[bin_index] <= 18'sd0;
                    s2[bin_index] <= 18'sd0;
                    state<=ST_NEXT;
                end
                ST_NEXT: begin
                    if(bin_index==3'd7) begin
                        bin_index <= 3'd0;
                        if(window_index==5'd31) begin
                            window_index <= 5'd0;
                            frame_toggle <= ~frame_toggle;
                        end else
                            window_index <= window_index + 5'd1;
                        state <= ST_IDLE;
                    end else begin
                        bin_index <= bin_index + 3'd1;
                        state <= ST_UPDATE;
                    end
                end
                default:state<=ST_IDLE;
            endcase
        end
    end
endmodule

