// E9.3.30 FAT32 Candidate C — AREA-FOCUSED PAYLOAD ADAPTERS
// The only supported PCM bank is the validated 25-key E93F bank.
// The only supported BMP layout is 640x480x24bpp, bottom-up, fixed 54-byte
// header and exact 921654-byte total size (the caller enforces this too).
// Instead of holding all header DWORDs in registers, inspect the header
// as SD delivers one byte per cycle. This is independent of SDRAM arbitration.
// This candidate is not a TD-compiled or board-proven release.
module e93_fat32_piano_payload #(
    parameter [20:0] AUDIO_WORDS = 21'd844672,
    parameter [31:0] AUDIO_CRC = 32'h0302c286
)(
    input wire clk, rst,
    input wire stream_start,
    input wire stream_done,
    input wire [31:0] stream_file_size,
    input wire [31:0] stream_offset,
    input wire [7:0] stream_data,
    input wire stream_valid,
    output reg generic_sbnk_seen,
    output reg header_ok,
    output reg e93_bank_start,
    output reg [20:0] e93_bank_words,
    output reg e93_pcm_word_en,
    output reg [31:0] e93_pcm_word,
    output reg e93_scan_complete,
    output reg [2:0] error_code
);
    localparam [31:0] EXPECTED_SIZE = 32'd512 + ({11'd0,AUDIO_WORDS} << 2);
    // Validity of byte positions 0..19. Unexamined reserved header bytes
    // remain unexamined, just as in the previous version.
    reg header_bytes_ok;
    reg [23:0] partial_word;
    reg [20:0] received_words;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            generic_sbnk_seen <= 1'b0;
            header_ok <= 1'b0;
            e93_bank_start <= 1'b0;
            e93_bank_words <= 21'd0;
            e93_pcm_word_en <= 1'b0;
            e93_pcm_word <= 32'd0;
            e93_scan_complete <= 1'b0;
            error_code <= 3'd0;
            header_bytes_ok <= 1'b1;
            partial_word <= 24'd0;
            received_words <= 21'd0;
        end else begin
            e93_bank_start <= 1'b0;
            e93_pcm_word_en <= 1'b0;
            // Completion is deliberately sticky: existing top consumes a level.
            if (stream_start) begin
                generic_sbnk_seen <= 1'b0;
                header_ok <= 1'b0;
                e93_scan_complete <= 1'b0;
                error_code <= 3'd0;
                header_bytes_ok <= 1'b1;
                partial_word <= 24'd0;
                received_words <= 21'd0;
            end else if (stream_valid) begin
                // Decoder guard reduces full-width position comparisons for
                // fixed-position header bytes. Payload starts at byte 512.
                if (stream_offset[31:9] == 23'd0) begin
                    case (stream_offset[8:0])
                        9'd0: if (stream_data != 8'h53) header_bytes_ok <= 1'b0; // S
                        9'd1: if (stream_data != 8'h42) header_bytes_ok <= 1'b0; // B
                        9'd2: if (stream_data != 8'h4e) header_bytes_ok <= 1'b0; // N
                        9'd3: begin
                            if (header_bytes_ok && stream_data == 8'h4b)
                                generic_sbnk_seen <= 1'b1;
                            else
                                header_bytes_ok <= 1'b0;
                        end
                        9'd4: if (stream_data != 8'h45) header_bytes_ok <= 1'b0; // E
                        9'd5: if (stream_data != 8'h39) header_bytes_ok <= 1'b0; // 9
                        9'd6: if (stream_data != 8'h33) header_bytes_ok <= 1'b0; // 3
                        9'd7: if (stream_data != 8'h46) header_bytes_ok <= 1'b0; // F
                        9'd8:  if (stream_data != AUDIO_WORDS[7:0]) header_bytes_ok <= 1'b0;
                        9'd9:  if (stream_data != AUDIO_WORDS[15:8]) header_bytes_ok <= 1'b0;
                        9'd10: if (stream_data != {3'd0,AUDIO_WORDS[20:16]}) header_bytes_ok <= 1'b0;
                        9'd11: if (stream_data != 8'd0) header_bytes_ok <= 1'b0;
                        9'd12: if (stream_data != AUDIO_CRC[7:0]) header_bytes_ok <= 1'b0;
                        9'd13: if (stream_data != AUDIO_CRC[15:8]) header_bytes_ok <= 1'b0;
                        9'd14: if (stream_data != AUDIO_CRC[23:16]) header_bytes_ok <= 1'b0;
                        9'd15: if (stream_data != AUDIO_CRC[31:24]) header_bytes_ok <= 1'b0;
                        9'd16: if (stream_data != 8'h34) header_bytes_ok <= 1'b0; // 4
                        9'd17: if (stream_data != 8'h38) header_bytes_ok <= 1'b0; // 8
                        9'd18: if (stream_data != 8'h4b) header_bytes_ok <= 1'b0; // K
                        9'd19: if (stream_data != 8'h31) header_bytes_ok <= 1'b0; // 1
                        9'd511: begin
                            if (header_bytes_ok && stream_file_size == EXPECTED_SIZE) begin
                                header_ok <= 1'b1;
                                e93_bank_words <= AUDIO_WORDS;
                                e93_bank_start <= 1'b1;
                            end else begin
                                error_code <= 3'd1;
                            end
                        end
                        default: ;
                    endcase
                end
                if (stream_offset[31:9] != 23'd0 && header_ok) begin
                    case (stream_offset[1:0])
                        2'd0: partial_word[7:0] <= stream_data;
                        2'd1: partial_word[15:8] <= stream_data;
                        2'd2: partial_word[23:16] <= stream_data;
                        2'd3: begin
                            if (received_words < AUDIO_WORDS) begin
                                e93_pcm_word <= {stream_data,partial_word};
                                e93_pcm_word_en <= 1'b1;
                                received_words <= received_words + 21'd1;
                            end else error_code <= 3'd3;
                        end
                    endcase
                end
            end
            if (stream_done) begin
                if (header_ok && received_words == AUDIO_WORDS && error_code == 3'd0)
                    e93_scan_complete <= 1'b1;
                else if (header_ok && received_words != AUDIO_WORDS)
                    error_code <= 3'd2;
            end
        end
    end
endmodule

// BMP: currently only exact 921654-byte 640x480 24bpp pictures are selected
// by fat32_bmp_read. Given that exact size, the former conditions
//   54 <= pixel_offset <= file_size
//   (file_size - pixel_offset) >= 921600
// mathematically imply pixel_offset == 54. Checking each specified header
// byte against its known constant avoids 192 header-register bits plus
// the variable 32-bit subtraction, with no change to accepted 640x480 files.
module e93_fat32_bmp_payload (
    input wire clk, rst,
    input wire stream_start,
    input wire stream_done,
    input wire [31:0] stream_file_size,
    input wire [31:0] stream_offset,
    input wire [7:0] stream_data,
    input wire stream_valid,
    output reg bmp_header_ok,
    output reg bmp_data_wr_en,
    output reg [23:0] bmp_data,
    output reg image_source_done,
    output reg [3:0] error_code
);
    localparam [31:0] BMP_EXACT_SIZE = 32'd921654;
    reg header_bytes_ok;
    reg [1:0] pixel_byte_index;
    reg [18:0] pixels_issued;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bmp_header_ok <= 1'b0;
            bmp_data_wr_en <= 1'b0;
            bmp_data <= 24'd0;
            image_source_done <= 1'b0;
            error_code <= 4'd0;
            header_bytes_ok <= 1'b1;
            pixel_byte_index <= 2'd0;
            pixels_issued <= 19'd0;
        end else begin
            bmp_data_wr_en <= 1'b0;
            image_source_done <= 1'b0;
            if (stream_start) begin
                bmp_header_ok <= 1'b0;
                error_code <= 4'd0;
                header_bytes_ok <= 1'b1;
                pixel_byte_index <= 2'd0;
                pixels_issued <= 19'd0;
            end else if (stream_valid) begin
                if (stream_offset[31:6] == 26'd0) begin
                    case (stream_offset[5:0])
                        6'd0: if (stream_data != 8'h42) header_bytes_ok <= 1'b0; // B
                        6'd1: if (stream_data != 8'h4d) header_bytes_ok <= 1'b0; // M
                        // bfSize = 921654 = 0x000E1036
                        6'd2: if (stream_data != 8'h36) header_bytes_ok <= 1'b0;
                        6'd3: if (stream_data != 8'h10) header_bytes_ok <= 1'b0;
                        6'd4: if (stream_data != 8'h0e) header_bytes_ok <= 1'b0;
                        6'd5: if (stream_data != 8'h00) header_bytes_ok <= 1'b0;
                        // bfOffBits = 54 = 0x00000036
                        6'd10: if (stream_data != 8'h36) header_bytes_ok <= 1'b0;
                        6'd11,6'd12,6'd13: if (stream_data != 8'h00) header_bytes_ok <= 1'b0;
                        // biWidth = 640 = 0x00000280
                        6'd18: if (stream_data != 8'h80) header_bytes_ok <= 1'b0;
                        6'd19: if (stream_data != 8'h02) header_bytes_ok <= 1'b0;
                        6'd20,6'd21: if (stream_data != 8'h00) header_bytes_ok <= 1'b0;
                        // biHeight = 480 = 0x000001e0
                        6'd22: if (stream_data != 8'he0) header_bytes_ok <= 1'b0;
                        6'd23: if (stream_data != 8'h01) header_bytes_ok <= 1'b0;
                        6'd24,6'd25: if (stream_data != 8'h00) header_bytes_ok <= 1'b0;
                        // biBitCount = 24, biCompression = BI_RGB (0)
                        6'd28: if (stream_data != 8'h18) header_bytes_ok <= 1'b0;
                        6'd29,6'd30,6'd31,6'd32,6'd33:
                            if (stream_data != 8'h00) header_bytes_ok <= 1'b0;
                        6'd53: begin
                            if (header_bytes_ok && stream_file_size == BMP_EXACT_SIZE)
                                bmp_header_ok <= 1'b1;
                            else error_code <= 4'd1;
                        end
                        default: ;
                    endcase
                end
                if (bmp_header_ok && stream_offset >= 32'd54 &&
                    pixels_issued < 19'd307200) begin
                    case (pixel_byte_index)
                        2'd0: begin
                            bmp_data[7:0] <= stream_data;
                            pixel_byte_index <= 2'd1;
                        end
                        2'd1: begin
                            bmp_data[15:8] <= stream_data;
                            pixel_byte_index <= 2'd2;
                        end
                        2'd2: begin
                            bmp_data[23:16] <= stream_data;
                            bmp_data_wr_en <= 1'b1;
                            pixel_byte_index <= 2'd0;
                            pixels_issued <= pixels_issued + 19'd1;
                        end
                        default: pixel_byte_index <= 2'd0;
                    endcase
                end
            end
            if (stream_done) begin
                if (bmp_header_ok && pixels_issued == 19'd307200 && error_code == 4'd0)
                    image_source_done <= 1'b1;
                else error_code <= 4'd2;
            end
        end
    end
endmodule


// E964 P4.3: VIOLIN.BIN stream decoder. Shared SINGLE FAT32 sector reader,
// SINGLE 512-word PCM DMA FIFO, and existing 4-voice player.
// Requires an exact, validated "SBNKE93V" 512-byte header, the reference
// 480000 PCM32 words, and the 1,920,512-byte user file. No sample ROM.
// Every 32-bit word is packed little-endian as two signed PCM16 samples.
module e964_fat32_violin_payload(
    input wire clk, rst,
    input wire stream_start,
    input wire stream_done,
    input wire [31:0] stream_file_size,
    input wire [31:0] stream_offset,
    input wire [7:0] stream_data,
    input wire stream_valid,
    output reg header_ok,
    output reg violin_bank_start,
    output reg [20:0] violin_bank_words,
    output reg violin_pcm_word_en,
    output reg [31:0] violin_pcm_word,
    output reg violin_scan_complete,
    output reg [2:0] error_code
);
    localparam [20:0] VIOLIN_WORDS = 21'd480000;
    localparam [31:0] EXPECTED_SIZE = 32'd1920512;
    // Header byte 0..43 includes file magic, version, 48kHz,
    // MIDI60..84, sample count, payload CRC32 and packed-word count.
    reg header_bytes_ok;
    reg [23:0] partial_word;
    reg [19:0] received_words;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            header_ok <= 1'b0;
            violin_bank_start <= 1'b0;
            violin_bank_words <= 21'd0;
            violin_pcm_word_en <= 1'b0;
            violin_pcm_word <= 32'd0;
            violin_scan_complete <= 1'b0;
            error_code <= 3'd0;
            header_bytes_ok <= 1'b1;
            partial_word <= 24'd0;
            received_words <= 20'd0;
        end else begin
            violin_bank_start <= 1'b0;
            violin_pcm_word_en <= 1'b0;
            if(stream_start) begin
                header_ok <= 1'b0;
                violin_scan_complete <= 1'b0;
                violin_bank_words <= 21'd0;
                error_code <= 3'd0;
                header_bytes_ok <= 1'b1;
                partial_word <= 24'd0;
                received_words <= 20'd0;
            end else if(stream_valid) begin
                if(stream_offset[31:9]==23'd0) begin
                    case(stream_offset[8:0])
                        9'd0: if(stream_data!=8'h53) header_bytes_ok<=1'b0;
                        9'd1: if(stream_data!=8'h42) header_bytes_ok<=1'b0;
                        9'd2: if(stream_data!=8'h4e) header_bytes_ok<=1'b0;
                        9'd3: if(stream_data!=8'h4b) header_bytes_ok<=1'b0;
                        9'd4: if(stream_data!=8'h45) header_bytes_ok<=1'b0;
                        9'd5: if(stream_data!=8'h39) header_bytes_ok<=1'b0;
                        9'd6: if(stream_data!=8'h33) header_bytes_ok<=1'b0;
                        9'd7: if(stream_data!=8'h56) header_bytes_ok<=1'b0;
                        9'd8: if(stream_data!=8'h01) header_bytes_ok<=1'b0;
                        9'd9: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd10: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd11: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd12: if(stream_data!=8'h80) header_bytes_ok<=1'b0;
                        9'd13: if(stream_data!=8'hbb) header_bytes_ok<=1'b0;
                        9'd14: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd15: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd16: if(stream_data!=8'h19) header_bytes_ok<=1'b0;
                        9'd17: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd18: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd19: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd20: if(stream_data!=8'h3c) header_bytes_ok<=1'b0;
                        9'd21: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd22: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd23: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd24: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd25: if(stream_data!=8'ha6) header_bytes_ok<=1'b0;
                        9'd26: if(stream_data!=8'h0e) header_bytes_ok<=1'b0;
                        9'd27: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd28: if(stream_data!=8'h76) header_bytes_ok<=1'b0;
                        9'd29: if(stream_data!=8'h99) header_bytes_ok<=1'b0;
                        9'd30: if(stream_data!=8'h7b) header_bytes_ok<=1'b0;
                        9'd31: if(stream_data!=8'h1a) header_bytes_ok<=1'b0;
                        9'd32: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd33: if(stream_data!=8'h02) header_bytes_ok<=1'b0;
                        9'd34: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd35: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd36: if(stream_data!=8'h03) header_bytes_ok<=1'b0;
                        9'd37: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd38: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd39: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd40: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd41: if(stream_data!=8'h53) header_bytes_ok<=1'b0;
                        9'd42: if(stream_data!=8'h07) header_bytes_ok<=1'b0;
                        9'd43: if(stream_data!=8'h00) header_bytes_ok<=1'b0;
                        9'd511: begin
                            if(header_bytes_ok && stream_file_size==EXPECTED_SIZE) begin
                                header_ok<=1'b1;
                                violin_bank_words<=VIOLIN_WORDS;
                                violin_bank_start<=1'b1;
                            end else
                                error_code<=3'd1;
                        end
                        default: ;
                    endcase
                end else if(header_ok) begin
                    case(stream_offset[1:0])
                        2'd0: partial_word[7:0]<=stream_data;
                        2'd1: partial_word[15:8]<=stream_data;
                        2'd2: partial_word[23:16]<=stream_data;
                        2'd3: begin
                            if(received_words<20'd480000) begin
                                violin_pcm_word<={stream_data,partial_word};
                                violin_pcm_word_en<=1'b1;
                                received_words<=received_words+20'd1;
                            end else error_code<=3'd3;
                        end
                    endcase
                end
            end
            if(stream_done) begin
                if(header_ok && received_words==20'd480000 &&
                   error_code==3'd0)
                    violin_scan_complete<=1'b1;
                else if(header_ok && received_words!=20'd480000)
                    error_code<=3'd2;
            end
        end
    end
endmodule
