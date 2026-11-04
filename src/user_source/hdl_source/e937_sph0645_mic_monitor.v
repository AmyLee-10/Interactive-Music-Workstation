`timescale 1ns/1ps
// E9.3.37 SPH0645 microphone capture + small amplitude gate + OSD indicator.
//
// Source: Knowles SPH0645LM4H-B Rev C, I2S slave, 64 BCLK/sample frame.
// 12.288 MHz audio_mclk /4 = 3.072MHz BCLK; BCLK/64 = 48kHz LRCL.
// SEL must be tied LOW (GND) for LEFT slot with this implementation.
// NO FPGA IO pins are assigned inside RTL. See mic_pin.adc.template.
// NOT a pitch detector. Does not feed audio playback or Note RAM.
// Compile this file together with the modified top_tf_hdmi_audio.v.

module e937_sph0645_capture #(
    parameter integer PROBE_FRAMES = 4800
) (
    input  wire audio_mclk,   // PLL output: 12.288 MHz
    input  wire video_clk,   // 25 MHz existing editor domain
    input  wire rst,         // existing PLL/reset logic
    input  wire mic_dout,    // mic -> FPGA input
    output wire mic_bclk,    // FPGA -> mic: 3.072 MHz
    output wire mic_lrcl,    // FPGA -> mic: 48 kHz, LOW=left
    output reg signed [15:0] pcm16,
    output reg pcm_valid,
    output reg cdc_overrun,
    output wire frame_alive,
    output wire left_raw_active,
    output wire right_raw_active,
    // Per selected slot over last ~100ms:
    // [1]=at least one logic 1 seen, [0]=at least one logic 0 seen.
    // 2'b01=low only, 2'b10=high only, 2'b11=both levels seen.
    output reg [1:0] left_pin_levels,
    output reg [1:0] right_pin_levels
);
    reg [1:0] bclk_div;
    reg [5:0] slot;
    assign mic_bclk = bclk_div[1];
    assign mic_lrcl = slot[5];

    // Rising BCLK is the microphone's data-launch edge. Data is sampled
    // at falling BCLK, half a period (~163ns) later; no gated clock is used.
    // I2S skips 1 bit at each WS transition. LEFT bits occupy slots 1..24.
    reg [23:0] sample_shift;
    reg signed [15:0] sample_hold;
    reg req_toggle;
    reg [1:0] ack_meta;
    reg ack_toggle;
    reg [2:0] req_meta;
    reg overrun_source;
    reg [2:0] overrun_meta;

    // P2.2-Lite: lightweight independent pin-level probe.
    // 25MHz oversamples the generated 3.07MHz BCLK; use the delayed
    // BCLK FALLING edge to sample the external DOUT away from the
    // SPH0645's rising-edge data launch. This is diagnostic only;
    // the original I2S capture and CDC above are unchanged.
    reg [2:0] probe_bclk_sync;
    reg [2:0] probe_lrcl_sync;
    reg [2:0] probe_dout_sync;
    wire probe_bfall = probe_bclk_sync[2] && !probe_bclk_sync[1];
    reg probe_ws_seen;
    reg [4:0] probe_slot_index; // 0..23 actual data bits after I2S delay
    reg [1:0] left_seen_levels, right_seen_levels;
    reg [12:0] probe_frame_count; // 4800 PCM-valids -> ~100 ms
    reg [20:0] frame_heartbeat;
    assign frame_alive = (frame_heartbeat != 21'd0);
    assign left_raw_active  = (left_pin_levels == 2'b11) && frame_alive;
    assign right_raw_active = (right_pin_levels == 2'b11) && frame_alive;

    always @(posedge audio_mclk or posedge rst) begin
        if(rst) begin
            bclk_div <= 2'd0;
            slot <= 6'd0;
            sample_shift <= 24'd0;
            sample_hold <= 16'sd0;
            req_toggle <= 1'b0;
            ack_meta <= 2'd0;
            overrun_source <= 1'b0;
        end else begin
            bclk_div <= bclk_div + 2'd1;
            ack_meta <= {ack_meta[0],ack_toggle};
            if(bclk_div == 2'd3) begin
                slot <= slot + 6'd1;
                if(!slot[5] && slot[4:0] >= 5'd1 &&
                   slot[4:0] <= 5'd24) begin
                    sample_shift <= {sample_shift[22:0],mic_dout};
                    if(slot[4:0] == 5'd24) begin
                        if(req_toggle == ack_meta[1]) begin
                            // Complete 24-bit word is {sample_shift[22:0],
                            // mic_dout}. Keep signed high 16, no multiplier.
                            sample_hold <= $signed(sample_shift[22:7]);
                            req_toggle <= ~req_toggle;
                        end else
                            overrun_source <= 1'b1;
                    end
                end
            end
        end
    end

    // Toggle/ack CDC: the 16-bit bus is held stable until acknowledged.
    // Only sample_hold is transferred, so the stream is isolated from
    // arbitrary phase between video_clk and audio_mclk.
    always @(posedge video_clk or posedge rst) begin
        if(rst) begin
            req_meta <= 3'd0;
            ack_toggle <= 1'b0;
            pcm16 <= 16'sd0;
            pcm_valid <= 1'b0;
            overrun_meta <= 3'd0;
            cdc_overrun <= 1'b0;
            frame_heartbeat <= 21'd0;
            probe_bclk_sync<=3'd0;
            probe_lrcl_sync<=3'd0;
            probe_dout_sync<=3'd0;
            probe_ws_seen<=1'b0;
            probe_slot_index<=5'd0;
            left_seen_levels<=2'd0;
            right_seen_levels<=2'd0;
            left_pin_levels<=2'd0;
            right_pin_levels<=2'd0;
            probe_frame_count<=13'd0;
        end else begin
            req_meta <= {req_meta[1:0],req_toggle};
            overrun_meta <= {overrun_meta[1:0],overrun_source};
            cdc_overrun <= overrun_meta[2];
            probe_bclk_sync <= {probe_bclk_sync[1:0],mic_bclk};
            probe_lrcl_sync <= {probe_lrcl_sync[1:0],mic_lrcl};
            probe_dout_sync <= {probe_dout_sync[1:0],mic_dout};

            // The selected slot can be inspected even if the original
            // 24-bit I2S decoder has a format/alignment error.
            // Unselected half-frame may be Hi-Z, so high/low alone
            // is not evidence of valid microphone audio.
            if (probe_bfall) begin
                if (probe_lrcl_sync[1] != probe_ws_seen) begin
                    // LRCL changes at BCLK falling; next falling edge
                    // is the 1st valid I2S data bit in this half frame.
                    probe_ws_seen <= probe_lrcl_sync[1];
                    probe_slot_index <= 5'd0;
                end else begin
                    if(probe_slot_index != 5'd31)
                        probe_slot_index <= probe_slot_index + 5'd1;
                    if(probe_slot_index >= 5'd1 && probe_slot_index <= 5'd24) begin
                        if (probe_ws_seen) begin
                            if (probe_dout_sync[2])
                                right_seen_levels[1] <= 1'b1;
                            else
                                right_seen_levels[0] <= 1'b1;
                        end else begin
                            if (probe_dout_sync[2])
                                left_seen_levels[1] <= 1'b1;
                            else
                                left_seen_levels[0] <= 1'b1;
                        end
                    end
                end
            end

            // Snapshot ~10 times/s for readable and persistent colors.
            // Existing PCM CDC provides a 48kHz heartbeat.
            if(pcm_valid) begin
                if(probe_frame_count==PROBE_FRAMES-1) begin
                    probe_frame_count<=13'd0;
                    left_pin_levels<=left_seen_levels;
                    right_pin_levels<=right_seen_levels;
                    left_seen_levels<=2'd0;
                    right_seen_levels<=2'd0;
                end else
                    probe_frame_count<=probe_frame_count+13'd1;
            end
            pcm_valid <= 1'b0;
            if(req_meta[2] != ack_toggle) begin
                pcm16 <= sample_hold;
                pcm_valid <= 1'b1;
                ack_toggle <= req_meta[2];
                frame_heartbeat <= 21'd1250000; // 50ms @25MHz
            end else if(frame_heartbeat != 21'd0)
                frame_heartbeat <= frame_heartbeat - 21'd1;

            // P2.2-Lite replaces the older edge-toggle hold counters
            // with simple per-slot high/low level accumulation.
        end
    end
endmodule

// Low-cost signal-presence indicator:
// (1) remove microphone DC offset with an IIR high-pass,
// (2) average absolute samples over 256 audio frames (~5.33ms),
// (3) require >= 3 above-threshold blocks to light,
// (4) require >= 8 quiet blocks to turn off.
// The status is NOT evidence of accurate pitch or a healthy microphone.
// Holding DOUT at 0 generates silent PCM and leaves the indicator gray.
module e937_mic_activity #(
    parameter [15:0] ON_AVG  = 16'd220,
    parameter [15:0] OFF_AVG = 16'd110
) (
    input  wire clk,
    input  wire rst,
    input  wire signed [15:0] pcm16,
    input  wire pcm_valid,
    output reg loud,
    output reg [2:0] level
);
    // DC accumulator keeps 8 fractional bits; slow (~21ms) DC tracking.
    reg signed [25:0] dc_q8;
    wire signed [25:0] pcm_q8 = {{2{pcm16[15]}},pcm16,8'd0};
    wire signed [25:0] diff_q8 = pcm_q8 - dc_q8;
    wire signed [17:0] hp = diff_q8 >>> 8;
    wire [17:0] magnitude = hp[17] ? $unsigned(-hp) : $unsigned(hp);
    wire [15:0] abs16 =
        (|magnitude[17:16]) ? 16'hffff : magnitude[15:0];

    reg [23:0] energy_sum;
    reg [7:0]  n_in_block;
    wire [23:0] energy_next = energy_sum + {8'd0,abs16};
    wire [15:0] avg_abs = energy_next[23:8]; // /256, no DSP/divider
    reg [1:0]  consecutive_hot;
    reg [3:0]  consecutive_quiet;
    // Allow ~170ms after boot for microphone power-up and DC IIR settling.
    reg [13:0] warmup_samples;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            dc_q8 <= 26'sd0;
            energy_sum <= 24'd0;
            n_in_block <= 8'd0;
            consecutive_hot <= 2'd0;
            consecutive_quiet <= 4'd0;
            warmup_samples <= 14'd0;
            loud <= 1'b0;
            level <= 3'd0;
        end else if(pcm_valid) begin
            dc_q8 <= dc_q8 + (diff_q8 >>> 10);
            if(!warmup_samples[13]) begin
                // Do not treat boot-time DC settling as sound activity.
                warmup_samples <= warmup_samples + 14'd1;
                energy_sum <= 24'd0;
                n_in_block <= 8'd0;
                consecutive_hot <= 2'd0;
                consecutive_quiet <= 4'd0;
                loud <= 1'b0;
                level <= 3'd0;
            end else if(n_in_block == 8'd255) begin
                n_in_block <= 8'd0;
                energy_sum <= 24'd0;

                // Four small steps; no wide dynamic RGB interpolation.
                if      (avg_abs >= 16'd120) level <= 3'd4;
                else if (avg_abs >= 16'd60)  level <= 3'd3;
                else if (avg_abs >= ON_AVG)   level <= 3'd2;
                else if (avg_abs >= 16'd8)    level <= 3'd1;
                else                          level <= 3'd0;

                if(avg_abs >= ON_AVG) begin
                    consecutive_quiet <= 4'd0;
                    if(consecutive_hot != 2'd3)
                        consecutive_hot <= consecutive_hot + 2'd1;
                    if(consecutive_hot >= 2'd2)
                        loud <= 1'b1;
                end else if(avg_abs <= OFF_AVG) begin
                    consecutive_hot <= 2'd0;
                    if(consecutive_quiet != 4'd8)
                        consecutive_quiet <= consecutive_quiet + 4'd1;
                    if(consecutive_quiet >= 4'd7)
                        loud <= 1'b0;
                end else begin
                    // Hysteresis band: do not flicker around the threshold.
                    consecutive_hot <= 2'd0;
                    consecutive_quiet <= 4'd0;
                end
            end else begin
                n_in_block <= n_in_block + 8'd1;
                energy_sum <= energy_next;
            end
        end
    end
endmodule

// Reuses the current 640x480 video stream; purely combinational RGB mux.
// MIC label is drawn near the top, BELOW existing track/title text.
// Gray = quiet/no detectable AC input; amber = weak detected input;
// green = sustained above the voice threshold; red = CDC overrun.
// This does NOT claim that the mic is electrically connected or functioning.
module e937_mic_overlay (
    input wire clk,
    input wire rst,
    input wire vs,
    input wire de,
    input wire [23:0] rgb_in,
    output reg [23:0] rgb_out,
    input wire loud,
    input wire [2:0] level,
    input wire overrun,
    input wire frame_alive,       // C: internal PCM handshakes recently
    input wire left_raw_active,   // both 0/1 detected in left slot
    input wire right_raw_active,  // both 0/1 detected in right slot
    input wire [1:0] left_pin_levels,
    input wire [1:0] right_pin_levels,
    input wire self_test          // purple border means synthetic PCM test
);
    reg [9:0] x;
    reg [8:0] y;
    reg prev_de;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            x<=10'd0;
            y<=9'd0;
            prev_de<=1'b0;
        end else begin
            prev_de <= de;
            if(!vs) begin x<=10'd0; y<=9'd0; end
            else begin
                if(!prev_de && de) x<=10'd0;
                else if(de && x<10'd639) x<=x+10'd1;
                if(prev_de && !de && y<9'd479) y<=y+9'd1;
            end
        end
    end

    function [4:0] mic_font;
        input [1:0] ch;
        input [2:0] row;
        begin
            mic_font = 5'b00000;
            case(ch)
                2'd0: case(row) // M
                    0: mic_font=5'b10001;
                    1: mic_font=5'b11011;
                    2: mic_font=5'b10101;
                    3: mic_font=5'b10101;
                    4: mic_font=5'b10001;
                    5: mic_font=5'b10001;
                    6: mic_font=5'b10001;
                endcase
                2'd1: case(row) // I
                    0: mic_font=5'b11111;
                    1: mic_font=5'b00100;
                    2: mic_font=5'b00100;
                    3: mic_font=5'b00100;
                    4: mic_font=5'b00100;
                    5: mic_font=5'b00100;
                    6: mic_font=5'b11111;
                endcase
                2'd2: case(row) // C
                    0: mic_font=5'b01111;
                    1: mic_font=5'b10000;
                    2: mic_font=5'b10000;
                    3: mic_font=5'b10000;
                    4: mic_font=5'b10000;
                    5: mic_font=5'b10000;
                    6: mic_font=5'b01111;
                endcase
                default: mic_font=5'b00000;
            endcase
        end
    endfunction


    // C / L / R mini-glyphs: 3x5 font, one new selection each pixel.
    function [2:0] status_font;
        input [1:0] ch;
        input [2:0] row;
        begin
            status_font=3'b000;
            case (ch)
                2'd0: case(row) // C
                    0: status_font=3'b111;
                    1: status_font=3'b100;
                    2: status_font=3'b100;
                    3: status_font=3'b100;
                    4: status_font=3'b111;
                endcase
                2'd1: case(row) // L
                    0: status_font=3'b100;
                    1: status_font=3'b100;
                    2: status_font=3'b100;
                    3: status_font=3'b100;
                    4: status_font=3'b111;
                endcase
                2'd2: case(row) // R
                    0: status_font=3'b110;
                    1: status_font=3'b101;
                    2: status_font=3'b110;
                    3: status_font=3'b101;
                    4: status_font=3'b101;
                endcase
            endcase
        end
    endfunction

    reg [1:0] glyph_select;
    reg [2:0] glyph_row;
    reg [2:0] glyph_col;
    reg glyph_area;
    reg [4:0] glyph_bits;
    reg [23:0] active_color;
    reg [1:0] status_sel;
    reg [2:0] status_row;
    reg [1:0] status_col;
    reg status_area;
    reg [2:0] status_bits;
    reg [23:0] status_color;

    always @* begin
        glyph_select=2'd0;glyph_row=3'd0;glyph_col=3'd0;
        glyph_area=1'b0;
        if(x>=10'd475 && x<10'd480) begin
            glyph_select=2'd0; glyph_col=x-10'd475; glyph_area=1'b1;
        end else if(x>=10'd482 && x<10'd487) begin
            glyph_select=2'd1; glyph_col=x-10'd482; glyph_area=1'b1;
        end else if(x>=10'd489 && x<10'd494) begin
            glyph_select=2'd2; glyph_col=x-10'd489; glyph_area=1'b1;
        end
        if(y>=9'd36 && y<9'd43) glyph_row=y-9'd36;
        else glyph_area=1'b0;
        glyph_bits=mic_font(glyph_select,glyph_row);

        status_sel=2'd0;status_row=3'd0;status_col=2'd0;
        status_area=1'b0;
        status_color=24'h667b8d;
        if(x>=10'd505 && x<10'd508) begin
            status_sel=2'd0;status_col=x-10'd505;
            status_area=1'b1;
            if(frame_alive) status_color=24'h51e1ad;
        end else if(x>=10'd520 && x<10'd523) begin
            status_sel=2'd1;status_col=x-10'd520;
            status_area=1'b1;
            if(left_raw_active) status_color=24'h51e1ad;
            else if(left_pin_levels==2'b10) status_color=24'hf0bd73;
        end else if(x>=10'd535 && x<10'd538) begin
            status_sel=2'd2;status_col=x-10'd535;
            status_area=1'b1;
            if(right_raw_active) status_color=24'h51e1ad;
            else if(right_pin_levels==2'b10) status_color=24'hf0bd73;
        end
        if(y>=9'd37 && y<9'd42) status_row=y-9'd37;
        else status_area=1'b0;
        status_bits=status_font(status_sel,status_row);

        active_color=overrun ? 24'hfa7070 :
                     loud ? 24'h51e1ad :
                     (level!=3'd0) ? 24'hf0bd73 : 24'h71859b;

        rgb_out=rgb_in;
        if(de && x>=10'd468 && x<10'd596 &&
           y>=9'd31 && y<9'd48) begin
            rgb_out=24'h172b40;
            // The outer border is violet in SELF-TEST; otherwise subdued.
            if(x==10'd468 || x==10'd595 || y==9'd31 || y==9'd47)
                rgb_out=self_test ? 24'hb48dff : 24'h4a6479;

            if(glyph_area && glyph_bits[4-glyph_col])
                rgb_out=self_test ? 24'hd1a5fa : active_color;

            // Small 11px boxes behind C, L, R, with independent LED color.
            if(y>=9'd34 && y<9'd45) begin
                if(x>=10'd501 && x<10'd513)
                    rgb_out=frame_alive ? 24'h1f554a : 24'h304556;
                else if(x>=10'd516 && x<10'd528)
                    rgb_out=left_raw_active ? 24'h1f554a :
                            (left_pin_levels==2'b10) ? 24'h71532c : 24'h304556;
                else if(x>=10'd531 && x<10'd543)
                    rgb_out=right_raw_active ? 24'h1f554a :
                            (right_pin_levels==2'b10) ? 24'h71532c : 24'h304556;
            end
            if(status_area && status_bits[2-status_col])
                rgb_out=status_color;

            // Four PCM-volume blocks; lower bring-up thresholds (8/30/60/120).
            if(y>=9'd36 && y<9'd44) begin
                if(x>=10'd548 && x<10'd555)
                    rgb_out=(level>=3'd1) ? active_color : 24'h364a5b;
                else if(x>=10'd558 && x<10'd565)
                    rgb_out=(level>=3'd2) ? active_color : 24'h364a5b;
                else if(x>=10'd568 && x<10'd575)
                    rgb_out=(level>=3'd3) ? active_color : 24'h364a5b;
                else if(x>=10'd578 && x<10'd585)
                    rgb_out=(level>=3'd4) ? active_color : 24'h364a5b;
                // Red means CDC overrun; violet means synthetic self-test.
                if(x>=10'd588 && x<10'd592)
                    rgb_out=overrun ? 24'hff5367 :
                            self_test ? 24'had83f9 : 24'h41687d;
            end
        end
    end
endmodule
