// E9.3 FAT32 COMPATIBILITY LOADER — experimental bmp_read port-compatible
// candidate based on the readable E9.3.11 interface. DO NOT overwrite
// E9.3.30 without checking current module ports, full RTL STA and board tests.
// Filename contract: root/PIANO.BIN and root/{0..9}.BMP, optionally
// root/IMAGES/{0..9}.BMP if IMAGE_USE_FOLDER=1. A maximum of 4 images
// are advertised to the existing image scheduler.
// The 'sector' returned to the image scheduler is a VIRTUAL index (0..9).
// sd_card_bmp's existing frame write/commit protocol is left untouched.
module fat32_bmp_read #(
    parameter IMAGE_USE_FOLDER = 1'b0,
    // These are SD-domain cycles (100 MHz in E9.3.30).
    parameter [27:0] WRITER_ACK_TIMEOUT = 28'd200_000_000,
    parameter [27:0] IMAGE_FINISH_TIMEOUT = 28'd100_000
)(
    input                       clk,
    input                       rst,
    output wire                 ready,
    input                       scan_start,
    input  [31:0]               scan_start_sector,  // unused in FAT mode
    input  [31:0]               scan_max_sector,    // unused in FAT mode
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,
    output reg                  bank_found,
    output reg                  piano_c4_pcm_wr_en,
    output reg [11:0]           piano_c4_pcm_wr_addr,
    output reg [15:0]           piano_c4_pcm_wr_data,
    output reg                  piano_c4_pcm_ready,
    output wire                 e93_bank_start,
    output wire [20:0]          e93_bank_words,
    output wire                 e93_pcm_word_en,
    output wire [31:0]          e93_pcm_word,
    output wire                 e93_scan_complete,
    // P4.3: piano remains playable if optional VIOLIN.BIN is absent.
    input                       piano_dma_ready_sd,
    input                       violin_dma_ready_sd,
    input                       load_start,
    input  [31:0]               load_sector,        // virtual index 0..9
    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,          // fixed 640
    input  [15:0]               bmp_height,         // fixed 480
    output reg                  write_req,
    input                       write_req_ack,
    output wire                 sd_sec_read,
    output wire [31:0]          sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,
    output wire                 bmp_data_wr_en,
    output wire [23:0]          bmp_data,
    output reg                  image_source_ok,     // pulse only after all 307200 pixels
    output reg                  image_load_failed,   // pulse: do NOT commit this framebuffer
    output reg                  image_fatal_error,   // sticky: possible partial video writer
    output reg  [3:0]           fat_error_code      // diagnostic last FAT read error
);
    localparam [3:0] S_IDLE        = 4'd0,
                     S_BANK_WAIT   = 4'd1,
                     S_SCAN_LAUNCH = 4'd2,
                     S_SCAN_WAIT   = 4'd3,
                     S_PROBE_WAIT  = 4'd4,
                     S_REQ_WRITER  = 4'd5,
                     S_IMAGE_WAIT  = 4'd6,
                     S_IMAGE_END   = 4'd7,
                     S_VIN_LOOKUP_LAUNCH = 4'd8,
                     S_VIN_LOOKUP_WAIT   = 4'd9,
                     S_VIN_PIANO_DRAIN   = 4'd10,
                     S_VIN_FILE_WAIT     = 4'd11,
                     S_VIN_FINISH_CHECK  = 4'd12,
                     S_VIN_DMA_WAIT      = 4'd13;
    reg [3:0] state;
    reg [27:0] wait_watchdog;
    reg [3:0] scan_idx, selected_idx;
    reg       fs_start, fs_lookup_only, fs_use_folder, fs_busy_seen;
    reg [31:0] fs_read_limit_bytes;
    reg [87:0] fs_file_name;
    wire fs_busy, fs_done, fs_found;
    wire [3:0] fs_error;
    wire [31:0] fs_file_size;
    wire [7:0] fs_byte;
    wire fs_valid, fs_last;
    wire [31:0] fs_offset;

    // E948: insert a one-cycle registered boundary after FAT32's
    // data_byte/data_offset/data_valid outputs. This directly addresses
    // the reported sd_card_clk path from data_byte_reg[0] into BMP/PCM.
    // Only matched PIANO.BIN (~3.38MB) and BMP (~0.92MB) streams reach
    // this adapter, so 24-bit byte offsets are sufficient.
    reg [7:0] fs_byte_q;
    reg [23:0] fs_offset_q;
    reg fs_valid_q, fs_done_q;
    reg fs_piano_flow_q, fs_image_flow_q, fs_full_image_flow_q;
    reg fs_violin_flow_q;
    // E969: FAT32 'found' pulse precedes its 'done' pulse.
    // Hold the earlier result until the lookup completes.
    reg violin_lookup_found_seen;
    reg violin_stream_start;
    reg bank_flow_violin;
    wire violin_header_ok, violin_scan_complete;
    wire violin_bank_start, violin_pcm_word_en;
    wire [20:0] violin_bank_words;
    wire [31:0] violin_pcm_word;
    wire [2:0] violin_error;
    wire [31:0] fs_pipelined_offset = {8'd0,fs_offset_q};
    // Data itself has no reset mux; fs_valid_q gates every consumption.
    always @(posedge clk) begin
        fs_byte_q <= fs_byte;
        fs_offset_q <= fs_offset[23:0];
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fs_valid_q <= 1'b0;
            fs_done_q <= 1'b0;
            fs_piano_flow_q <= 1'b0;
            fs_violin_flow_q <= 1'b0;
            fs_image_flow_q <= 1'b0;
            fs_full_image_flow_q <= 1'b0;
        end else begin
            fs_valid_q <= fs_valid;
            fs_done_q <= fs_done;
            fs_piano_flow_q <= is_piano_flow;
            fs_violin_flow_q <= is_violin_flow;
            fs_image_flow_q <= is_image_flow;
            fs_full_image_flow_q <= is_full_image_flow;
        end
    end

    reg pcm_stream_start, bmp_stream_start;
    wire pcm_generic_seen, pcm_hdr_ok, bmp_hdr_ok, bmp_finished;
    wire [2:0] pcm_error;
    wire [3:0] bmp_error;
    reg [7:0] c4_low_byte;
    reg [11:0] c4_sample_index;
    wire is_piano_flow = (state == S_BANK_WAIT);
    wire is_violin_flow = (state == S_VIN_FILE_WAIT ||
                           state == S_VIN_FINISH_CHECK);
    wire is_image_flow = (state == S_PROBE_WAIT ||
                          state == S_IMAGE_WAIT || state == S_IMAGE_END);
    wire is_full_image_flow = (state == S_IMAGE_WAIT || state == S_IMAGE_END);
    assign ready = (state == S_IDLE);
    function [87:0] bmp_name;
        input [3:0] index;
        begin
            // "0       BMP" etc, exactly 8+3 ASCII bytes.
            bmp_name = {8'h30 + {4'd0,index}, 56'h20202020202020, "BMP"};
        end
    endfunction
    fat32_file_stream u_fat (
        .clk(clk), .rst(rst), .sd_init_done(sd_init_done),
        .start(fs_start), .file_name_83(fs_file_name),
        .use_folder(fs_use_folder), .folder_name_83("IMAGES     "),
        .lookup_only(fs_lookup_only), .read_limit_bytes(fs_read_limit_bytes),
        .busy(fs_busy), .done(fs_done), .found(fs_found),
        .error(fs_error), .file_size(fs_file_size),
        .data_byte(fs_byte), .data_valid(fs_valid),
        .data_last(fs_last), .data_offset(fs_offset),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end)
    );
    wire piano_bank_start, piano_pcm_word_en, piano_scan_complete;
    wire [20:0] piano_bank_words;
    wire [31:0] piano_pcm_word;
    e93_fat32_piano_payload u_pcm (
        .clk(clk), .rst(rst),
        .stream_start(pcm_stream_start),
        .stream_done(fs_done_q && fs_piano_flow_q),
        .stream_file_size(fs_file_size),
        .stream_offset(fs_pipelined_offset), .stream_data(fs_byte_q),
        .stream_valid(fs_valid_q && fs_piano_flow_q),
        .generic_sbnk_seen(pcm_generic_seen),
        .header_ok(pcm_hdr_ok),
        .e93_bank_start(piano_bank_start),
        .e93_bank_words(piano_bank_words),
        .e93_pcm_word_en(piano_pcm_word_en),
        .e93_pcm_word(piano_pcm_word),
        .e93_scan_complete(piano_scan_complete),
        .error_code(pcm_error)
    );
    // No duplicate SD SPI/FAT module: the same u_fat handles both PCM
    // files and every BMP sequentially.  The existing piano payload
    // remains unmodified, and the bank writer sees one active stream.
    e964_fat32_violin_payload u_violin (
        .clk(clk), .rst(rst),
        .stream_start(violin_stream_start),
        .stream_done(fs_done_q && fs_violin_flow_q),
        .stream_file_size(fs_file_size),
        .stream_offset(fs_pipelined_offset),
        .stream_data(fs_byte_q),
        .stream_valid(fs_valid_q && fs_violin_flow_q),
        .header_ok(violin_header_ok),
        .violin_bank_start(violin_bank_start),
        .violin_bank_words(violin_bank_words),
        .violin_pcm_word_en(violin_pcm_word_en),
        .violin_pcm_word(violin_pcm_word),
        .violin_scan_complete(violin_scan_complete),
        .error_code(violin_error)
    );
    // The old piano 'complete' signal is sticky.  Select, NEVER OR,
    // the currently loading bank to prevent a premature violin finish.
    assign e93_bank_start = bank_flow_violin ?
                            violin_bank_start : piano_bank_start;
    assign e93_bank_words = bank_flow_violin ?
                            violin_bank_words : piano_bank_words;
    assign e93_pcm_word_en = bank_flow_violin ?
                             violin_pcm_word_en : piano_pcm_word_en;
    assign e93_pcm_word = bank_flow_violin ?
                          violin_pcm_word : piano_pcm_word;
    assign e93_scan_complete = bank_flow_violin ?
                               violin_scan_complete : piano_scan_complete;

    e93_fat32_bmp_payload u_bmp (
        .clk(clk), .rst(rst),
        .stream_start(bmp_stream_start),
        .stream_done(fs_done_q && fs_full_image_flow_q),
        .stream_file_size(fs_file_size),
        .stream_offset(fs_pipelined_offset), .stream_data(fs_byte_q),
        .stream_valid(fs_valid_q && fs_image_flow_q),
        .bmp_header_ok(bmp_hdr_ok),
        .bmp_data_wr_en(bmp_data_wr_en), .bmp_data(bmp_data),
        .image_source_done(bmp_finished), .error_code(bmp_error)
    );
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            wait_watchdog <= 28'd0;
            state_code <= 4'd0;
            scan_idx <= 4'd0;
            selected_idx <= 4'd0;
            fs_start <= 1'b0;
            fs_lookup_only <= 1'b0;
            fs_read_limit_bytes <= 32'd0;
            fs_use_folder <= 1'b0;
            fs_busy_seen <= 1'b0;
            fs_file_name <= 88'd0;
            pcm_stream_start <= 1'b0;
            violin_stream_start <= 1'b0;
            violin_lookup_found_seen <= 1'b0;
            bank_flow_violin <= 1'b0;
            bmp_stream_start <= 1'b0;
            scan_done <= 1'b0;
            scan_found_valid <= 1'b0;
            scan_found_sector <= 32'd0;
            scan_found_total <= 3'd0;
            bank_found <= 1'b0;
            write_req <= 1'b0;
            piano_c4_pcm_wr_en <= 1'b0;
            piano_c4_pcm_wr_addr <= 12'd0;
            piano_c4_pcm_wr_data <= 16'd0;
            piano_c4_pcm_ready <= 1'b0;
            image_source_ok <= 1'b0;
            image_load_failed <= 1'b0;
            image_fatal_error <= 1'b0;
            fat_error_code <= 4'd0;
            c4_low_byte <= 8'd0;
            c4_sample_index <= 12'd0;
        end else if (!sd_init_done) begin
            state <= S_IDLE;
            wait_watchdog <= 28'd0;
            state_code <= 4'd0;
            fs_start <= 1'b0;
            pcm_stream_start <= 1'b0;
            violin_stream_start <= 1'b0;
            violin_lookup_found_seen <= 1'b0;
            bank_flow_violin <= 1'b0;
            bmp_stream_start <= 1'b0;
            scan_done <= 1'b0;
            scan_found_valid <= 1'b0;
            bank_found <= 1'b0;
            piano_c4_pcm_wr_en <= 1'b0;
            piano_c4_pcm_ready <= 1'b0;
            write_req <= 1'b0;
            image_source_ok <= 1'b0;
            image_load_failed <= 1'b0;
            image_fatal_error <= 1'b0;
            fat_error_code <= 4'd0;
        end else begin
            fs_start <= 1'b0;
            image_source_ok <= 1'b0;
            image_load_failed <= 1'b0;
            pcm_stream_start <= 1'b0;
            violin_stream_start <= 1'b0;
            bmp_stream_start <= 1'b0;
            scan_found_valid <= 1'b0;
            piano_c4_pcm_wr_en <= 1'b0;
            if (pcm_generic_seen)
                bank_found <= 1'b1;
            // Collect just first 4096 C4 PCM16 samples to preserve existing
            // optional diagnostic ROM sample interface. Normal full piano
            // still comes from the main E93F bank writer, not these 4096 samples.
            if (fs_piano_flow_q && pcm_hdr_ok && fs_valid_q &&
                fs_pipelined_offset >= 32'd512 &&
                fs_pipelined_offset < 32'd8704) begin
                if (fs_offset_q[0] == 1'b0)
                    c4_low_byte <= fs_byte_q;
                else if (!piano_c4_pcm_ready) begin
                    piano_c4_pcm_wr_addr <= c4_sample_index;
                    piano_c4_pcm_wr_data <= {fs_byte_q,c4_low_byte};
                    piano_c4_pcm_wr_en <= 1'b1;
                    if (c4_sample_index == 12'd4095)
                        piano_c4_pcm_ready <= 1'b1;
                    else
                        c4_sample_index <= c4_sample_index + 12'd1;
                end
            end
            if (fs_busy) fs_busy_seen <= 1'b1;
            case (state)
                S_IDLE: begin
                    state_code <= 4'd1;
                    wait_watchdog <= 28'd0;
                    write_req <= 1'b0;
                    if (scan_start) begin
                        scan_idx <= 4'd0;
                        scan_done <= 1'b0;
                        scan_found_total <= 3'd0;
                        scan_found_sector <= 32'd0;
                        bank_found <= 1'b0;
                        piano_c4_pcm_ready <= 1'b0;
                        c4_sample_index <= 12'd0;
                        fs_file_name <= "PIANO   BIN";
                        fs_use_folder <= 1'b0;
                        fs_lookup_only <= 1'b0;
                        fs_read_limit_bytes <= 32'd0;
                        fs_busy_seen <= 1'b0;
                        fs_start <= 1'b1;
                        pcm_stream_start <= 1'b1;
                        bank_flow_violin <= 1'b0;
                        state <= S_BANK_WAIT;
                    end else if (load_start && !image_fatal_error) begin
                        // Open first 54 bytes to validate the actual BMP header
                        // BEFORE arming the video writer. The FAT engine still
                        // consumes the remainder of that physical SD sector.
                        selected_idx <= load_sector[3:0];
                        fs_file_name <= bmp_name(load_sector[3:0]);
                        fs_use_folder <= IMAGE_USE_FOLDER;
                        fs_lookup_only <= 1'b0;
                        fs_read_limit_bytes <= 32'd54;
                        fs_busy_seen <= 1'b0;
                        fs_start <= 1'b1;
                        bmp_stream_start <= 1'b1;
                        state <= S_PROBE_WAIT;
                    end
                end
                S_BANK_WAIT: begin
                    state_code <= 4'd2;
                    if (fs_done || (fs_busy_seen && !fs_busy && fs_error != 4'd0)) begin
                        if (fs_error != 4'd0) fat_error_code <= fs_error;
                        if (fs_done && pcm_hdr_ok && fs_error == 4'd0)
                            state <= S_VIN_LOOKUP_LAUNCH;
                        else state <= S_SCAN_LAUNCH;
                    end
                end
                S_VIN_LOOKUP_LAUNCH: begin
                    // Optional root /VIOLIN.BIN directory lookup.
                    // Reset the sticky match for each fresh FAT lookup.
                    violin_lookup_found_seen <= 1'b0;
                    state_code <= 4'd2;
                    fs_file_name <= "VIOLIN  BIN";
                    fs_use_folder <= 1'b0;
                    fs_lookup_only <= 1'b1;
                    fs_read_limit_bytes <= 32'd0;
                    fs_busy_seen <= 1'b0;
                    fs_start <= 1'b1;
                    state <= S_VIN_LOOKUP_WAIT;
                end
                S_VIN_LOOKUP_WAIT: begin
                    state_code <= 4'd2;
                    // FAT32 sets 'found' at the directory entry, then
                    // 'done' in a later ST_FILE_INIT cycle.
                    // Remember found until done rather than ANDing pulses.
                    if(fs_found)
                        violin_lookup_found_seen <= 1'b1;
                    if (fs_done || (fs_busy_seen && !fs_busy &&
                                    fs_error != 4'd0)) begin
                        if (fs_done &&
                            (violin_lookup_found_seen || fs_found) &&
                            fs_file_size == 32'd1920512 &&
                            fs_error == 4'd0) begin
                            wait_watchdog <= 28'd0;
                            state <= S_VIN_PIANO_DRAIN;
                        end else
                            // Missing / wrong-size VIOLIN keeps PIANO
                            // loaded and continues original BMP scan.
                            state <= S_SCAN_LAUNCH;
                    end
                end
                S_VIN_PIANO_DRAIN: begin
                    // Critical: don't reopen the SINGLE PCM write FIFO
                    // until the first bank DMA has completely finished.
                    state_code <= 4'd2;
                    if (piano_dma_ready_sd) begin
                        bank_flow_violin <= 1'b1;
                        fs_file_name <= "VIOLIN  BIN";
                        fs_use_folder <= 1'b0;
                        fs_lookup_only <= 1'b0;
                        fs_read_limit_bytes <= 32'd0;
                        fs_busy_seen <= 1'b0;
                        fs_start <= 1'b1;
                        violin_stream_start <= 1'b1;
                        wait_watchdog <= 28'd0;
                        state <= S_VIN_FILE_WAIT;
                    end else if(wait_watchdog >= 28'd200000000)
                        state <= S_SCAN_LAUNCH;
                    else wait_watchdog <= wait_watchdog + 28'd1;
                end
                S_VIN_FILE_WAIT: begin
                    state_code <= 4'd2;
                    if(fs_done) begin
                        wait_watchdog <= 28'd0;
                        state <= S_VIN_FINISH_CHECK;
                    end else if(fs_busy_seen && !fs_busy &&
                                fs_error != 4'd0)
                        state <= S_SCAN_LAUNCH;
                end
                S_VIN_FINISH_CHECK: begin
                    // E948 FAT->payload byte pipeline may need one cycle
                    // after raw fs_done before violin_scan_complete rises.
                    if(violin_scan_complete) begin
                        wait_watchdog <= 28'd0;
                        state <= S_VIN_DMA_WAIT;
                    end else if(wait_watchdog >= 28'd8)
                        state <= S_SCAN_LAUNCH;
                    else wait_watchdog <= wait_watchdog + 28'd1;
                end
                S_VIN_DMA_WAIT: begin
                    state_code <= 4'd2;
                    if(violin_dma_ready_sd)
                        state <= S_SCAN_LAUNCH;
                    else if(wait_watchdog >= 28'd200000000)
                        state <= S_SCAN_LAUNCH;
                    else wait_watchdog <= wait_watchdog + 28'd1;
                end
                S_SCAN_LAUNCH: begin
                    state_code <= 4'd2;
                    fs_file_name <= bmp_name(scan_idx);
                    fs_use_folder <= IMAGE_USE_FOLDER;
                    fs_lookup_only <= 1'b1;
                    fs_read_limit_bytes <= 32'd0;
                    fs_busy_seen <= 1'b0;
                    fs_start <= 1'b1;
                    state <= S_SCAN_WAIT;
                end
                S_SCAN_WAIT: begin
                    state_code <= 4'd2;
                    if (fs_done || (fs_busy_seen && !fs_busy && fs_error != 4'd0)) begin
                        if (fs_error != 4'd0) fat_error_code <= fs_error;
                        if (fs_done && fs_file_size == 32'd921654) begin
                            scan_found_valid <= 1'b1;
                            scan_found_sector <= {28'd0,scan_idx};
                            if (scan_found_total < 3'd4)
                                scan_found_total <= scan_found_total + 3'd1;
                        end
                        if (scan_idx == 4'd9 ||
                            (fs_done && (scan_found_total + 3'd1 >= scan_target_count) &&
                             fs_file_size == 32'd921654)) begin
                            scan_done <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            scan_idx <= scan_idx + 4'd1;
                            state <= S_SCAN_LAUNCH;
                        end
                    end
                end
                S_PROBE_WAIT: begin
                    state_code <= 4'd3;
                    if (fs_done) begin
                        if (bmp_hdr_ok && bmp_error == 4'd0 &&
                            fs_file_size == 32'd921654) begin
                            write_req <= 1'b1;
                            state <= S_REQ_WRITER;
                        end else begin
                            image_load_failed <= 1'b1;
                            fat_error_code <= 4'd10; // bad BMP header
                            state <= S_IDLE;
                        end
                    end else if (fs_busy_seen && !fs_busy && fs_error != 4'd0) begin
                        image_load_failed <= 1'b1;
                        fat_error_code <= fs_error;
                        state <= S_IDLE;
                    end
                end
                S_REQ_WRITER: begin
                    state_code <= 4'd3;
                    if (write_req_ack) begin
                        wait_watchdog <= 28'd0;
                        write_req <= 1'b0;
                        fs_file_name <= bmp_name(selected_idx);
                        fs_use_folder <= IMAGE_USE_FOLDER;
                        fs_lookup_only <= 1'b0;
                        fs_read_limit_bytes <= 32'd0;
                        fs_busy_seen <= 1'b0;
                        fs_start <= 1'b1;
                        bmp_stream_start <= 1'b1;
                        state <= S_IMAGE_WAIT;
                    end else if (wait_watchdog >= WRITER_ACK_TIMEOUT) begin
                        // No writer ACK. Never open the image stream.
                        write_req <= 1'b0;
                        image_load_failed <= 1'b1;
                        fat_error_code <= 4'd12;
                        state <= S_IDLE;
                    end else begin
                        wait_watchdog <= wait_watchdog + 28'd1;
                    end
                end
                S_IMAGE_WAIT: begin
                    wait_watchdog <= 28'd0;
                    state_code <= 4'd4;
                    if (fs_done) state <= S_IMAGE_END;
                    else if (fs_busy_seen && !fs_busy && fs_error != 4'd0) begin
                        // Once the writer has accepted the frame, a short file
                        // can strand its FIFO. Never reuse that buffer without
                        // a system reset/recovery of the video writer.
                        image_load_failed <= 1'b1;
                        image_fatal_error <= 1'b1;
                        fat_error_code <= fs_error;
                        state <= S_IDLE;
                    end
                end
                S_IMAGE_END: begin
                    state_code <= 4'd4;
                    if (bmp_finished) begin
                        image_source_ok <= 1'b1;
                        state <= S_IDLE;
                    end else if (bmp_error != 4'd0) begin
                        image_load_failed <= 1'b1;
                        image_fatal_error <= 1'b1;
                        fat_error_code <= 4'd11; // bad BMP body/pixel count
                        state <= S_IDLE;
                    end else if (wait_watchdog >= IMAGE_FINISH_TIMEOUT) begin
                        // Decoder's completion pulse was lost. Video writer
                        // may be partly filled, so freeze until reset.
                        image_load_failed <= 1'b1;
                        image_fatal_error <= 1'b1;
                        fat_error_code <= 4'd13;
                        state <= S_IDLE;
                    end else begin
                        wait_watchdog <= wait_watchdog + 28'd1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
