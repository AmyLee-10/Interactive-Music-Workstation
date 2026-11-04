// lab_ex5_i2s_20261027_e9_3_39 — compact 4-voice mixer, natural PCM tails.
// Based on the working-image E9.3.36 player. Only this Verilog source changes.
// Four sample voices share a single SDRAM request channel and one sequential
// mixer. Each voice owns 16 PCM32 words in a 64-word circular cache.
// Every Note-On triggers one complete piano sample; Note-Off does NOT shorten it.
// A held key does not retrigger when the sample finishes.
// No changes to FAT32, image double-buffer, PLL, TF SPI or SDRAM arbitration.
// NOTE: E9.3.35 top passes DUO_TEST=1. Kept as a legacy compatibility flag;
// for THIS version, it selects the FOUR-voice mode, not two voices.
// EXPERIMENTAL until TD synthesis, timing and board playback are verified.
module e93_piano_8voice #(
    parameter [20:0] AUDIO_BASE = 21'd655360,
    parameter [20:0] VIOLIN_BASE = 21'd1500032,
    parameter MONO_TEST = 1'b0,
    parameter DUO_TEST = 1'b0,
    parameter FOUR_VOICE = DUO_TEST
)(
    input wire clk, rst,
    input wire audio_valid,
    input wire bank_loaded,
    input wire violin_loaded,
    input wire        note_evt_valid,
    output wire       note_evt_ready,
    input wire [29:0] note_evt_data,
    input wire [3:0] track_mute_mask,
    output reg [23:0] pcm_out,
    output reg mem_req_toggle,
    output reg [20:0] mem_req_addr,
    input wire mem_rsp_toggle,
    input wire [31:0] mem_rsp_word,
    output reg underrun
);

    function [20:0] sample_base_addr;
        input [4:0] p;
        begin
            case(p)
                5'd0: sample_base_addr = AUDIO_BASE + 21'd0;
                5'd1: sample_base_addr = AUDIO_BASE + 21'd35072;
                5'd2: sample_base_addr = AUDIO_BASE + 21'd72064;
                5'd3: sample_base_addr = AUDIO_BASE + 21'd111360;
                5'd4: sample_base_addr = AUDIO_BASE + 21'd148224;
                5'd5: sample_base_addr = AUDIO_BASE + 21'd183424;
                5'd6: sample_base_addr = AUDIO_BASE + 21'd219136;
                5'd7: sample_base_addr = AUDIO_BASE + 21'd254848;
                5'd8: sample_base_addr = AUDIO_BASE + 21'd288512;
                5'd9: sample_base_addr = AUDIO_BASE + 21'd324096;
                5'd10: sample_base_addr = AUDIO_BASE + 21'd358528;
                5'd11: sample_base_addr = AUDIO_BASE + 21'd394752;
                5'd12: sample_base_addr = AUDIO_BASE + 21'd428288;
                5'd13: sample_base_addr = AUDIO_BASE + 21'd462208;
                5'd14: sample_base_addr = AUDIO_BASE + 21'd494720;
                5'd15: sample_base_addr = AUDIO_BASE + 21'd527872;
                5'd16: sample_base_addr = AUDIO_BASE + 21'd559616;
                5'd17: sample_base_addr = AUDIO_BASE + 21'd594048;
                5'd18: sample_base_addr = AUDIO_BASE + 21'd630272;
                5'd19: sample_base_addr = AUDIO_BASE + 21'd664832;
                5'd20: sample_base_addr = AUDIO_BASE + 21'd703616;
                5'd21: sample_base_addr = AUDIO_BASE + 21'd741248;
                5'd22: sample_base_addr = AUDIO_BASE + 21'd764416;
                5'd23: sample_base_addr = AUDIO_BASE + 21'd788864;
                5'd24: sample_base_addr = AUDIO_BASE + 21'd816384;
                default: sample_base_addr = AUDIO_BASE;
            endcase
        end
    endfunction
    function [16:0] frame_count;
        input [4:0] p;
        begin
            case(p)
                5'd0: frame_count = 17'd69984;
                5'd1: frame_count = 17'd73937;
                5'd2: frame_count = 17'd78521;
                5'd3: frame_count = 17'd73649;
                5'd4: frame_count = 17'd70176;
                5'd5: frame_count = 17'd71189;
                5'd6: frame_count = 17'd71237;
                5'd7: frame_count = 17'd67236;
                5'd8: frame_count = 17'd71141;
                5'd9: frame_count = 17'd68778;
                5'd10: frame_count = 17'd72395;
                5'd11: frame_count = 17'd66996;
                5'd12: frame_count = 17'd67620;
                5'd13: frame_count = 17'd65016;
                5'd14: frame_count = 17'd66270;
                5'd15: frame_count = 17'd63474;
                5'd16: frame_count = 17'd68634;
                5'd17: frame_count = 17'd72443;
                5'd18: frame_count = 17'd68922;
                5'd19: frame_count = 17'd77507;
                5'd20: frame_count = 17'd75095;
                5'd21: frame_count = 17'd46160;
                5'd22: frame_count = 17'd48668;
                5'd23: frame_count = 17'd54889;
                5'd24: frame_count = 17'd56335;
                default: frame_count = 17'd0;
            endcase
        end
    endfunction


    // E964: a 0.8s (38400-frame) 48kHz soundfont note occupies
    // 19200 consecutive PCM32 words. Actual loop starts from user file,
    // no runtime multiplier, no extra waveform memory.
    // P4.3.2: mathematically identical to the former 25-entry ROM.
    // offset = pitch * 19200 = (pitch * 75) << 8.
    // 24 * 75 = 1800, fitting 11 bits; invalid pitches retain zero offset.
    function [20:0] violin_base_offset;
        input [4:0] p;
        reg [10:0] times_75;
        begin
            times_75 = ({6'd0,p} << 6)
                     + ({6'd0,p} << 3)
                     + ({6'd0,p} << 1)
                     + {6'd0,p};
            violin_base_offset = (p < 5'd25)
                                ? {2'd0,times_75,8'd0} : 21'd0;
        end
    endfunction

    function [15:0] violin_loop_start;
        input [4:0] p;
        begin
            case(p)
                5'd0: violin_loop_start = 16'd20109;
                5'd1: violin_loop_start = 16'd19066;
                5'd2: violin_loop_start = 16'd19664;
                5'd3: violin_loop_start = 16'd14100;
                5'd4: violin_loop_start = 16'd20404;
                5'd5: violin_loop_start = 16'd20581;
                5'd6: violin_loop_start = 16'd20042;
                5'd7: violin_loop_start = 16'd20587;
                5'd8: violin_loop_start = 16'd19858;
                5'd9: violin_loop_start = 16'd19715;
                5'd10: violin_loop_start = 16'd20557;
                5'd11: violin_loop_start = 16'd13052;
                5'd12: violin_loop_start = 16'd15668;
                5'd13: violin_loop_start = 16'd20482;
                5'd14: violin_loop_start = 16'd15288;
                5'd15: violin_loop_start = 16'd19133;
                5'd16: violin_loop_start = 16'd13153;
                5'd17: violin_loop_start = 16'd13641;
                5'd18: violin_loop_start = 16'd17820;
                5'd19: violin_loop_start = 16'd18917;
                5'd20: violin_loop_start = 16'd13206;
                5'd21: violin_loop_start = 16'd20364;
                5'd22: violin_loop_start = 16'd15786;
                5'd23: violin_loop_start = 16'd17172;
                5'd24: violin_loop_start = 16'd20119;
                default:violin_loop_start=16'd20109;
            endcase
        end
    endfunction

    // The currently deployed top config MONO_TEST=0, DUO_TEST=1 selects
    // LAST_VOICE=3, allowing up to 4 simultaneously audible piano notes.
    // Optional fallback mode selection is compile-time only.
    localparam [1:0] LAST_VOICE = MONO_TEST ? 2'd0 :
                                  FOUR_VOICE ? 2'd3 : 2'd1;
    localparam [15:0] WINDOW_WORDS = 16'd16;
    localparam IDLE=4'd0, VOICE=4'd2,
               READ_WAIT=4'd3, GAIN=4'd4, SUM=4'd5,
               NEXT=4'd6, DROP_WAIT=4'd7,
               PREFETCH_DRAIN=4'd8;

    // Each PCM voice owns its note source identity; same pitch in distinct
    // tracks has distinct NOTE-ONs and does not share a 25-bit global gate.
    reg [1:0]  v_track [0:3];
    reg [8:0]  v_note_id [0:3];
    reg [4:0]  v_pitch [0:3];
    reg [17:0] v_start_tick [0:3];
    reg        v_violin [0:3];
    reg [15:0] v_sample_position [0:3];
    reg        v_releasing [0:3];
    reg [6:0]  v_gain [0:3];
    reg        v_active [0:3];

    reg [17:0] audio_tick_count;
    reg [1:0]  vi, steal, pre_slot;
    reg [3:0]  fsm;

    reg [2:0]  ack_sync;
    reg        ack_seen;
    reg        pending;
    reg        req_prefetch;
    reg [1:0]  req_slot;
    reg [15:0] req_word;
    reg [7:0]  req_epoch;
    reg        ignore_drain_data;
    reg        bank_was_loaded;

    // 4 x 16 PCM32 words = 64 PCM32 words (2048 bits of sample storage).
    // Not four replicated mixers or four replicated SDRAM readers.
    // The unreset PCM ring has ONE synchronous write port.
    // Index = {voice[1:0], word[3:0]}.
    reg [31:0] pcm_ring [0:63];
    reg [15:0] filled [0:3];
    reg [7:0]  epoch [0:3];

    reg [31:0] pcm_word;
    reg        high_half;
    reg signed [22:0] gain_product;
    reg signed [15:0] accum;
    reg        mix_ready;

    // Share note-age subtraction, table lookup and address generation across
    // foreground mixing and background PCM prefetch (one computation path).
    wire [1:0] selected_slot = (fsm == VOICE) ? vi : pre_slot;
    wire [17:0] selected_age = v_violin[selected_slot] ?
        {2'b00,v_sample_position[selected_slot]} :
        (audio_tick_count - v_start_tick[selected_slot]);
    wire [15:0] selected_word = selected_age[16:1];
    wire [15:0] selected_filled = filled[selected_slot];
    // Signed distance, valid across the whole 16-bit producer/index range.
    // One subtractor serves the prefetch limit and the ring hit test.
    wire signed [16:0] window_lead =
        $signed({1'b0,selected_filled}) -
        $signed({1'b0,selected_word});
    // The same selected frame-count ROM serves prefetch and VOICE.
    // In VOICE state selected_slot == vi, so this is bit-exact.
    wire [16:0] selected_frames = v_violin[selected_slot] ?
        17'd38400 : frame_count(v_pitch[selected_slot]);
    wire [15:0] selected_total_words =
        selected_frames[16:1] + {15'd0,selected_frames[0]};

    wire read_ahead_room = bank_loaded &&
        (selected_slot <= LAST_VOICE) && v_active[selected_slot] &&
        (selected_age < {1'b0,selected_frames}) &&
        (selected_filled < selected_total_words) &&
        (window_lead < 17'sd16);

    wire ring_hit = (window_lead > 17'sd0) &&
                    (window_lead <= 17'sd16);
    wire [31:0] ring_data =
        pcm_ring[{selected_slot,selected_word[3:0]}];

    wire [15:0] selected_req_word =
        (fsm == VOICE) ? selected_word : selected_filled;
    wire [20:0] selected_base =
        v_violin[selected_slot] ?
          (VIOLIN_BASE + violin_base_offset(v_pitch[selected_slot])) :
          sample_base_addr(v_pitch[selected_slot]);
    wire [20:0] selected_req_addr =
        selected_base + {5'd0,selected_req_word};

    // P1 shared event handshake, synchronous to clk. Piano uses complete
    // PCM sample even after Note-Off; square events belong to the oscillator.
    assign note_evt_ready = (fsm == IDLE) && !audio_valid;
    wire event_accept = note_evt_valid && note_evt_ready;
    wire event_on = note_evt_data[29];
    wire [1:0] event_instrument = note_evt_data[28:27];
    wire [1:0] event_track = note_evt_data[26:25];
    wire [8:0] event_note_id = note_evt_data[24:16];
    wire [4:0] event_pitch = note_evt_data[15:11];
    wire [6:0] event_gain = note_evt_data[6:0];
    // Same note-event FIFO/mixer/SDRAM ring serves BOTH PCM timbres.
    wire start_pcm_voice = event_accept && event_on && bank_loaded &&
        (((event_instrument==2'd0)) ||
         ((event_instrument==2'd2) && violin_loaded)) &&
        (event_pitch<5'd25) && (event_gain!=7'd0);
    wire violin_note_off = event_accept && !event_on &&
        (event_instrument==2'd2);

    // Prefer an unused voice; if all four are playing, steal round-robin.
    // A fifth Note-On can necessarily terminate an existing voice.
    wire [1:0] chosen_slot = MONO_TEST ? 2'd0 :
        (!v_active[0]) ? 2'd0 :
        (!v_active[1]) ? 2'd1 :
        ((LAST_VOICE == 2'd3) && !v_active[2]) ? 2'd2 :
        ((LAST_VOICE == 2'd3) && !v_active[3]) ? 2'd3 : steal;

    // One 25-entry loop-point ROM evaluated for the voice chosen by
    // the existing mix FSM, NOT four parallel 25-entry ROMs.
    wire [15:0] e964_voice_loop_start =
        violin_loop_start(v_pitch[vi]);
    wire signed [15:0] pcm16 = high_half ?
        $signed(pcm_word[31:16]) : $signed(pcm_word[15:0]);

    // The piano PCM already contains its true decay. Never apply the old
    // key-up Release envelope: this also removes the second multiply stage.
    // Shift 9 instead of 8 to leave headroom for four simultaneous notes;
    // all single-note outputs are correspondingly 1/2 the former peak level.
    // 16-bit signed PCM * [0,127] fits 23 signed bits.
    // After >>> 9 each voice is in [-8128,+8127].
    // Four voices sum to [-32512,+32508], fitting signed 16 bits.
    wire signed [15:0] gain_term = gain_product >>> 9;
    wire signed [15:0] accumulated = accum + gain_term;
    wire signed [15:0] held_pcm16 = $signed(pcm_out[23:8]);
    wire signed [15:0] faded_pcm16 = held_pcm16 - (held_pcm16 >>> 4);
    // No saturation circuit needed: the four-voice sum cannot clip.
    wire signed [15:0] clipped = accum;

    integer init_i;

    // One outstanding request owns the only cache write port. The generation
    // tag and monotone producer count prevent stale responses being installed.
    wire rsp_ready = pending && (ack_sync[2] != ack_seen);
    wire foreground_accept =
        !req_prefetch && !audio_valid &&
        (fsm == READ_WAIT ||
         (fsm == DROP_WAIT && !ignore_drain_data)) &&
        (req_word >= filled[req_slot]);
    wire prefetch_accept =
        req_prefetch && (req_word == filled[req_slot]);
    wire pcm_ring_write =
        !rst && bank_loaded && rsp_ready &&
        (req_epoch == epoch[req_slot]) &&
        (prefetch_accept || foreground_accept);

    always @(posedge clk) begin
        if (pcm_ring_write)
            pcm_ring[{req_slot,req_word[3:0]}] <= mem_rsp_word;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ack_sync <= 3'b0;
            ack_seen <= 1'b0;
            mem_req_toggle <= 1'b0;
            mem_req_addr <= AUDIO_BASE;
            pcm_out <= 24'd0;
            underrun <= 1'b0;

            audio_tick_count <= 18'd0;
            vi <= 2'd0;
            steal <= 2'd0;
            pre_slot <= 2'd0;
            fsm <= IDLE;

            pending <= 1'b0;
            req_prefetch <= 1'b0;
            req_slot <= 2'd0;
            req_word <= 16'd0;
            req_epoch <= 8'd0;
            ignore_drain_data <= 1'b0;
            bank_was_loaded <= 1'b0;
            pcm_word <= 32'd0;
            high_half <= 1'b0;
            gain_product <= 23'sd0;
            accum <= 16'sd0;
            mix_ready <= 1'b0;

            for (init_i=0; init_i<4; init_i=init_i+1) begin
                v_track[init_i] <= 2'd0;
                v_note_id[init_i] <= 9'd0;
                v_pitch[init_i] <= 5'd0;
                v_start_tick[init_i] <= 18'd0;
                v_violin[init_i] <= 1'b0;
                v_sample_position[init_i] <= 16'd0;
                v_releasing[init_i] <= 1'b0;
                v_gain[init_i] <= 7'd0;
                v_active[init_i] <= 1'b0;
                filled[init_i] <= 16'd0;
                epoch[init_i] <= 8'd0;
            end
        end else begin
            ack_sync <= {ack_sync[1:0],mem_rsp_toggle};
            bank_was_loaded <= bank_loaded;
            if (!pending)
                pre_slot <= pre_slot + 2'd1;
            if (audio_valid)
                audio_tick_count <= audio_tick_count + 18'd1;

            if (!bank_loaded && bank_was_loaded) begin
                for (init_i=0; init_i<4; init_i=init_i+1)
                    epoch[init_i] <= epoch[init_i] + 8'd1;
            end

            if (pending && req_prefetch && rsp_ready) begin
                pending <= 1'b0;
                ack_seen <= ack_sync[2];
                if (bank_loaded &&
                    req_epoch == epoch[req_slot] &&
                    req_word == filled[req_slot])
                    filled[req_slot] <= filled[req_slot] + 16'd1;
            end

            if (!bank_loaded) begin
                pcm_out <= 24'd0;
                mix_ready <= 1'b0;
                accum <= 16'sd0;
                for (init_i=0; init_i<4; init_i=init_i+1) begin
                    filled[init_i] <= 16'd0;
                    v_active[init_i] <= 1'b0;
                end

                if (pending && !req_prefetch) begin
                    if (ack_sync[2] != ack_seen) begin
                        ack_seen <= ack_sync[2];
                        pending <= 1'b0;
                        ignore_drain_data <= 1'b0;
                        fsm <= IDLE;
                    end else begin
                        ignore_drain_data <= 1'b1;
                        fsm <= DROP_WAIT;
                    end
                end else begin
                    fsm <= IDLE;
                end
            end else begin
                // Don't change the existing video/SDRAM priority. Audio
                // only requests when one previous request is fully drained.
                if (fsm == IDLE && !audio_valid && !event_accept &&
                    !pending && (mem_req_toggle == ack_seen) &&
                    read_ahead_room) begin
                    mem_req_addr <= selected_req_addr;
                    mem_req_toggle <= ~mem_req_toggle;
                    req_prefetch <= 1'b1;
                    req_slot <= selected_slot;
                    req_word <= selected_filled;
                    req_epoch <= epoch[selected_slot];
                    pending <= 1'b1;
                end

                // PCM Note-Ons share the existing 4 physical voices.
                // Piano is one-shot; violin holds until matching Note-Off.
                if (start_pcm_voice) begin
                    v_active[chosen_slot] <= 1'b1;
                    v_violin[chosen_slot] <=
                        (event_instrument==2'd2);
                    v_sample_position[chosen_slot] <= 16'd0;
                    v_releasing[chosen_slot] <= 1'b0;
                    v_pitch[chosen_slot] <= event_pitch;
                    v_track[chosen_slot] <= event_track;
                    v_note_id[chosen_slot] <= event_note_id;
                    v_start_tick[chosen_slot] <= audio_tick_count;
                    v_gain[chosen_slot] <= event_gain;
                    filled[chosen_slot] <= 16'd0;
                    epoch[chosen_slot] <= epoch[chosen_slot] + 8'd1;
                    steal <= (chosen_slot == LAST_VOICE) ?
                             2'd0 : chosen_slot + 2'd1;
                end
                // A change of this note's instrument releases any
                // previous sustained violin voice on the same identity.
                // (When the new voice steals that exact slot, don't
                // accidentally mark the newly initialized voice off.)
                if(event_accept && event_on) begin
                    for(init_i=0;init_i<4;init_i=init_i+1)
                        if(v_active[init_i] && v_violin[init_i] &&
                           v_track[init_i]==event_track &&
                           v_note_id[init_i]==event_note_id &&
                           (!start_pcm_voice || init_i != chosen_slot))
                            v_releasing[init_i] <= 1'b1;
                end
                // Voice-specific Note-Off, never cuts a piano's
                // natural tail and never releases a different track.
                if(violin_note_off) begin
                    for(init_i=0;init_i<4;init_i=init_i+1)
                        if(v_active[init_i] && v_violin[init_i] &&
                           v_track[init_i]==event_track &&
                           v_note_id[init_i]==event_note_id)
                            v_releasing[init_i] <= 1'b1;
                end

                if (audio_valid) begin
                    if (fsm == IDLE && mix_ready)
                        pcm_out <= {clipped,8'd0};
                    else
                        pcm_out <= {faded_pcm16,8'd0};
                end

                if (audio_valid && fsm != IDLE) begin
                    underrun <= 1'b1;
                    mix_ready <= 1'b0;
                    accum <= 16'sd0;
                    if (fsm == READ_WAIT || fsm == DROP_WAIT)
                        fsm <= DROP_WAIT;
                    else
                        fsm <= IDLE;
                end else begin
                    case (fsm)
                    IDLE: if (audio_valid) begin
                        mix_ready <= 1'b0;
                        accum <= 16'sd0;
                        vi <= 2'd0;
                        fsm <= VOICE;
                    end

                    VOICE: begin
                        if (!v_active[vi]) begin
                            fsm <= NEXT;
                        end else if (selected_age >=
                            {1'b0,selected_frames}) begin
                            v_active[vi] <= 1'b0;
                            fsm <= NEXT;
                        end else begin
                            high_half <= selected_age[0];
                            if (ring_hit) begin
                                pcm_word <= ring_data;
                                fsm <= GAIN;
                            end else if (pending) begin
                                fsm <= PREFETCH_DRAIN;
                            end else begin
                                mem_req_addr <= selected_req_addr;
                                mem_req_toggle <= ~mem_req_toggle;
                                req_prefetch <= 1'b0;
                                req_slot <= vi;
                                req_word <= selected_word;
                                req_epoch <= epoch[vi];
                                pending <= 1'b1;
                                ignore_drain_data <= 1'b0;
                                fsm <= READ_WAIT;
                            end
                        end
                    end

                    PREFETCH_DRAIN: if (!pending) fsm <= VOICE;

                    READ_WAIT: if (pending && !req_prefetch && rsp_ready) begin
                        ack_seen <= ack_sync[2];
                        pending <= 1'b0;
                        if (req_epoch == epoch[req_slot]) begin
                            pcm_word <= mem_rsp_word;
                            if (req_word >= filled[req_slot])
                                filled[req_slot] <= req_word + 16'd1;
                            fsm <= GAIN;
                        end else begin
                            fsm <= VOICE;
                        end
                    end

                    DROP_WAIT: if (pending && !req_prefetch && rsp_ready) begin
                        ack_seen <= ack_sync[2];
                        pending <= 1'b0;
                        if (!ignore_drain_data &&
                            req_epoch == epoch[req_slot] &&
                            req_word >= filled[req_slot])
                            filled[req_slot] <= req_word + 16'd1;
                        ignore_drain_data <= 1'b0;
                        fsm <= IDLE;
                    end

                    GAIN: begin
                        // Muted voices continue playing to their natural end.
                        gain_product <= track_mute_mask[v_track[vi]] ?
                               23'sd0 : pcm16 * $signed({1'b0,v_gain[vi]});
                        fsm <= SUM;
                    end

                    SUM: begin
                        accum <= accumulated;
                        fsm <= NEXT;
                    end

                    NEXT: begin
                        // Process one slot at a time, same 4-voice mixer.
                        // This is the ONLY violin position/loop update
                        // circuit, so 25x loop ROM is NOT instantiated
                        // independently for each simultaneous voice.
                        if(v_active[vi] && v_violin[vi]) begin
                            if(!violin_loaded) begin
                                v_active[vi] <= 1'b0;
                            end else begin
                                if(v_sample_position[vi]==16'd38399) begin
                                    v_sample_position[vi] <=
                                        e964_voice_loop_start;
                                    filled[vi] <= e964_voice_loop_start >> 1;
                                    epoch[vi] <= epoch[vi] + 8'd1;
                                end else
                                    v_sample_position[vi] <=
                                        v_sample_position[vi] + 16'd1;

                                // Reuse existing PCM16 x gain7 multiplier:
                                // fade is a decrement of v_gain, not a
                                // second independent envelope multiplier.
                                if(v_releasing[vi] &&
                                   audio_tick_count[5:0]==6'd0) begin
                                    if(v_gain[vi]<=7'd1) begin
                                        v_gain[vi] <= 7'd0;
                                        v_active[vi] <= 1'b0;
                                    end else
                                        v_gain[vi] <= v_gain[vi]-7'd1;
                                end
                            end
                        end
                        if(vi==LAST_VOICE) begin
                            mix_ready <= 1'b1;
                            fsm <= IDLE;
                        end else begin
                            vi <= vi + 2'd1;
                            fsm <= VOICE;
                        end
                    end
                    default: fsm <= IDLE;
                    endcase
                end
            end
        end
    end
endmodule
