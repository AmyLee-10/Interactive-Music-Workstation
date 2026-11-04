// E9.3.24 OWNER-GUARD - shared VIDEO return ownership protection.
// Based on E9.3.19 P-TRACE; all PCM reads, caches, diagnostics unchanged.
// This is a transaction-boundary fix, NOT an audio-noise cure.
// E9.3.19 P-TRACE - staged PASSIVE PCM latency telemetry.
// Based on the VERIFIED K 4-word read protocol.  8 contiguous 32-bit
// commands = two 4-word aligned groups.  There is NO background prefetch.
//
// Advantages relative to L:
//  * No speculative SDRAM transaction when the piano did not request one.
//  * No ahead_block -> 128-bit cache copy mux (L's reported critical paths).
//  * On a sequential voice, one SDRAM visit can serve 16 PCM16 frames.
//  * Video remains higher priority; preserve exactly 8 returned word tags.
//
// NO diagnostic SDRAM transactions. NO speculative or timed probes.
// Only observes NORMAL piano accesses; no additional SDRAM commands.
// Retains P-PASSIVE's exact 8-word command/response and arbiter behavior.
// The existing N diagnostic top-level and its port connections are unchanged.
module e93_sdram_reader (
    input wire rst, video_clk, mem_clk,
    input wire req_toggle_video,
    input wire [20:0] req_addr_video,
    output reg rsp_toggle_mem,
    output reg [31:0] rsp_word_mem,
    input wire video_App_rd_en,
    input wire [9:0] video_fifo_words_mem,
    input wire image_priority_busy,
    input wire pcm_writer_busy,
    input wire bank_loaded_mem,    // N top interface; for status only
    input wire sdram_init_done,
    input wire sdram_ref_vld,
    input wire sdram_busy,
    input wire Sdr_rd_en,
    input wire [31:0] Sdr_rd_dout,
    output reg audio_read_lock,
    // E9.3.24: export ONE existing video-command/response scoreboard.
    // True while the video port is issuing or still awaiting real responses.
    output wire video_returns_pending,
    output wire video_Sdr_rd_en,
    output wire audio_App_rd_en,
    output wire [20:0] audio_App_rd_addr,
    output wire [31:0] diag_status,
    output wire [20:0] diag_first_bad_addr,
    output wire [31:0] diag_first_bad_expected,
    output wire [31:0] diag_first_bad_observed
);
    localparam [9:0] VIDEO_SAFE_WORDS = 10'd384;
    localparam [5:0] VIDEO_QUIET_CLKS = 6'd32;

    localparam [3:0]
        S_IDLE         = 4'd0,
        S_LOOKUP       = 4'd1,
        S_CACHE_SELECT = 4'd2,
        S_CACHE_OUTPUT = 4'd3,
        S_WAIT         = 4'd4,
        S_GUARD        = 4'd5,
        S_ISSUE8       = 4'd6,
        S_RECV8        = 4'd7,
        S_FINISH       = 4'd8;

    reg [2:0] req_sync_mem;
    reg [3:0] state;
    reg [20:0] req_addr_mem;

    reg [9:0] video_pending;
    reg [5:0] video_quiet_cnt;
    reg pcm_burst_owns_responses;

    reg [20:0] issue_addr;
    reg [2:0] issue_lane;
    reg [2:0] response_lane;
    reg [2:0] chosen_lane;
    reg [17:0] requested_block;

    // Eight-word cache, packed as two consecutive FOUR-word SDRAM groups.
    // Data registers do not need reset; cache_valid gates their use.
    reg cache_valid;
    reg [17:0] cache_block;
    reg [31:0] cache_word0, cache_word1, cache_word2, cache_word3;
    reg [31:0] cache_word4, cache_word5, cache_word6, cache_word7;

    // Two-stage 8:1 lookup removes block-tag comparison from the
    // 32-bit sample-data D path, unlike L's 128-bit block promotion.
    reg [31:0] selected_lo, selected_hi;

    // The verified C4 PCM32 first word, only tested on a genuine PCM cache
    // miss for the aligned C4 starting block. No background reads.
    localparam [17:0] C4_FIRST_BLOCK = 18'd81920; // 655360 / 8
    localparam [31:0] C4_FIRST_WORD = 32'hFFF6FFF3;
    assign diag_first_bad_expected = C4_FIRST_WORD;

    reg [7:0] c4_real_reads;
    reg [7:0] c4_bad_reads;
    reg [7:0] pcm_slow_count;
    reg c4_seen, c4_bad_seen, wait_1ms_seen, any_pcm_completed;

    // Memory clock is 125 MHz: 2500 cycles = 20 us.
    // This timer is observation-only and does not affect the read FSM.
    reg [16:0] pcm_wait_ticks;
    reg slow_counted;

    // P-TRACE: two separate latency phases, measured in 125 MHz clocks.
    // GRANT: S_WAIT + S_GUARD, before the first PCM command.
    // BURST: S_ISSUE8 + S_RECV8, until the eighth PCM response.
    // These counters DO NOT alter any SDRAM request, lock, or reply.
    reg [15:0] grant_ticks, burst_ticks;
    reg [15:0] max_grant_ticks, max_burst_ticks;
    reg [7:0] grant_slow_count, burst_slow_count;
    reg grant_slow_counted, burst_slow_counted;
    reg [3:0] grant_blockers_seen;
    reg early_return_seen;
    reg [31:0] first_bad_c4_word;

    // Existing HDMI row 120 RIGHT (originally first bad address):
    //   If BB != 00: 000A0000 = first known C4 error address.
    //   If BB == 00: 0FPPQQ packed PASSIVE telemetry:
    //      F[4] = [20] early Sdr_rd_en DURING S_ISSUE8 (unexpected)
    //      F[3:0] = [19:16] wait>=20us blocker flags:
    //         [19] outstanding video reads
    //         [18] video FIFO below 384 words
    //         [17] image priority or PCM writer busy
    //         [16] SDRAM not ready/busy/refresh
    //      PP = [15:8] grant-phase waits >=20us (saturates at FF)
    //      QQ = [7:0] burst-phase waits >=20us (saturates at FF)
    // Existing HDMI row 130 RIGHT:
    //   If BB != 00: FIRST incorrect C4 word.
    //   If BB == 00: MMMMNNNN; max completed grant clocks and burst clocks.
    //   One clock = 8 ns. Each 16-bit maximum saturates at FFFF.
    // Row 130 LEFT remains the C4 expected word FFF6FFF3.
    assign diag_first_bad_addr =
        c4_bad_seen ? 21'd655360 :
        {early_return_seen, grant_blockers_seen,
         grant_slow_count, burst_slow_count};
    assign diag_first_bad_observed =
        c4_bad_seen ? first_bad_c4_word :
        {max_grant_ticks, max_burst_ticks};

    // HDMI row 120: RR BB SS FF
    // RR = C4 block's actual SDRAM read count (not a minute counter)
    // BB = C4 first-word mismatch count (expected FFF6FFF3)
    // SS = cache-miss waits over 20 us
    // FF bits 7..0:
    //   7: any PCM request completed; 6: cache valid; 5: PCM reply owner
    //   4: audio read lock (live); 3: any wait over 1 ms
    //   2: C4 mismatch ever; 1: C4 first word observed; 0: bank loaded
    assign diag_status = {
        c4_real_reads, c4_bad_reads, pcm_slow_count,
        any_pcm_completed, cache_valid, pcm_burst_owns_responses,
        audio_read_lock, wait_1ms_seen, c4_bad_seen, c4_seen,
        bank_loaded_mem
    };

    // E9.3.24 OWNER-GUARD:
    // This counter already tracked accepted VIDEO App_rd_en commands against
    // video-owned SDRAM return-valid words in E9.3.19. Expose the EXISTING
    // outstanding state to the image writer and PCM bank writer as an
    // admission interlock. This does not change read ownership, the
    // 8-word PCM burst, video FIFO thresholds, or return-data routing.
    //
    // Never time out and silently clear this flag: doing so could let
    // a late video response be mistaken for a PCM response.
    wire video_outstanding =
        (video_pending != 10'd0) || video_App_rd_en || video_Sdr_rd_en;
    assign video_returns_pending = video_outstanding;
    wire can_start_pcm =
        sdram_init_done && !sdram_ref_vld && !sdram_busy &&
        !image_priority_busy && !pcm_writer_busy && !video_outstanding &&
        (video_fifo_words_mem >= VIDEO_SAFE_WORDS) &&
        (video_quiet_cnt == VIDEO_QUIET_CLKS);

    // Only issue during one approved non-interruptible eight-word burst.
    // Video read/write FSMs already see audio_read_lock via ext_mem_busy.
    assign audio_App_rd_en = (state == S_ISSUE8);
    assign audio_App_rd_addr = issue_addr;
    assign video_Sdr_rd_en = Sdr_rd_en && !pcm_burst_owns_responses;

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            req_sync_mem <= 3'd0;
            req_addr_mem <= 21'd0;
            rsp_toggle_mem <= 1'b0;
            rsp_word_mem <= 32'd0;
            audio_read_lock <= 1'b0;
            state <= S_IDLE;
            video_pending <= 10'd0;
            video_quiet_cnt <= 6'd0;
            pcm_burst_owns_responses <= 1'b0;
            issue_addr <= 21'd0;
            issue_lane <= 3'd0;
            response_lane <= 3'd0;
            chosen_lane <= 3'd0;
            requested_block <= 18'd0;
            cache_valid <= 1'b0;
            cache_block <= 18'd0;
            selected_lo <= 32'd0;
            selected_hi <= 32'd0;
            c4_real_reads <= 8'd0;
            c4_bad_reads <= 8'd0;
            pcm_slow_count <= 8'd0;
            c4_seen <= 1'b0;
            c4_bad_seen <= 1'b0;
            wait_1ms_seen <= 1'b0;
            any_pcm_completed <= 1'b0;
            pcm_wait_ticks <= 17'd0;
            slow_counted <= 1'b0;
            grant_ticks <= 16'd0;
            burst_ticks <= 16'd0;
            max_grant_ticks <= 16'd0;
            max_burst_ticks <= 16'd0;
            grant_slow_count <= 8'd0;
            burst_slow_count <= 8'd0;
            grant_slow_counted <= 1'b0;
            burst_slow_counted <= 1'b0;
            grant_blockers_seen <= 4'd0;
            early_return_seen <= 1'b0;
            first_bad_c4_word <= 32'd0;
        end else begin
            req_sync_mem <= {req_sync_mem[1:0], req_toggle_video};

            // Entirely passive cache-miss wait measurement. A lost SDRAM
            // reply still requires RESET; never release ownership early and
            // misroute late PCM data into the video FIFO.
            if (state == S_LOOKUP &&
                !(cache_valid && cache_block == req_addr_mem[20:3])) begin
                pcm_wait_ticks <= 17'd0;
                slow_counted <= 1'b0;
            end else if (state == S_WAIT || state == S_GUARD ||
                         state == S_ISSUE8 || state == S_RECV8) begin
                if (pcm_wait_ticks < 17'd125000) begin
                    pcm_wait_ticks <= pcm_wait_ticks + 17'd1;
                    if (pcm_wait_ticks == 17'd2499 && !slow_counted) begin
                        slow_counted <= 1'b1;
                        if (pcm_slow_count != 8'hff)
                            pcm_slow_count <= pcm_slow_count + 8'd1;
                    end
                    if (pcm_wait_ticks == 17'd124999)
                        wait_1ms_seen <= 1'b1;
                end
            end

            // P-TRACE: passive phase timing.  Independent of the original
            // total-time SS/FF diagnostics, which are retained unchanged.
            // Reset only for a real cache MISS, never for a cache HIT.
            if (state == S_LOOKUP &&
                !(cache_valid && cache_block == req_addr_mem[20:3])) begin
                grant_ticks <= 16'd0;
                burst_ticks <= 16'd0;
                grant_slow_counted <= 1'b0;
                burst_slow_counted <= 1'b0;
            end else begin
                if (state == S_WAIT || state == S_GUARD) begin
                    if (grant_ticks != 16'hffff)
                        grant_ticks <= grant_ticks + 16'd1;
                    if (grant_ticks == 16'd2499 && !grant_slow_counted) begin
                        grant_slow_counted <= 1'b1;
                        if (grant_slow_count != 8'hff)
                            grant_slow_count <= grant_slow_count + 8'd1;
                        // A single snapshot of the blockers present at
                        // the 20-us threshold; flags remain sticky.
                        grant_blockers_seen <= grant_blockers_seen |
                            {video_outstanding,
                             (video_fifo_words_mem < VIDEO_SAFE_WORDS),
                             (image_priority_busy || pcm_writer_busy),
                             (!sdram_init_done || sdram_busy || sdram_ref_vld)};
                    end
                end
                if (state == S_ISSUE8 || state == S_RECV8) begin
                    if (burst_ticks != 16'hffff)
                        burst_ticks <= burst_ticks + 16'd1;
                    if (burst_ticks == 16'd2499 && !burst_slow_counted) begin
                        burst_slow_counted <= 1'b1;
                        if (burst_slow_count != 8'hff)
                            burst_slow_count <= burst_slow_count + 8'd1;
                    end
                end
            end

            // If the SDRAM returns a word while the eight commands are
            // STILL being issued, the existing P-PASSIVE FSM does not
            // consume it.  Flag that case without changing ownership.
            if (state == S_ISSUE8 && Sdr_rd_en)
                early_return_seen <= 1'b1;

            // The counter records VIDEO commands only.  PCM returns are
            // filtered out by the owner tag, never sent to the video FIFO.
            case ({video_App_rd_en, video_Sdr_rd_en})
                2'b10: if (video_pending != 10'd1023)
                           video_pending <= video_pending + 10'd1;
                2'b01: if (video_pending != 10'd0)
                           video_pending <= video_pending - 10'd1;
                default: ;
            endcase

            if (video_outstanding || image_priority_busy)
                video_quiet_cnt <= 6'd0;
            else if (video_quiet_cnt < VIDEO_QUIET_CLKS)
                video_quiet_cnt <= video_quiet_cnt + 6'd1;

            case (state)
                S_IDLE: begin
                    // NO look-ahead transaction is allowed in IDLE.
                    if (req_sync_mem[2] != rsp_toggle_mem) begin
                        req_addr_mem <= req_addr_video;
                        state <= S_LOOKUP;
                    end
                end
                S_LOOKUP: begin
                    chosen_lane <= req_addr_mem[2:0];
                    if (cache_valid && cache_block == req_addr_mem[20:3])
                        state <= S_CACHE_SELECT;
                    else
                        state <= S_WAIT;
                end
                S_CACHE_SELECT: begin
                    // Register one selected pair (4:1, not 8:1 + tag).
                    case (chosen_lane[2:1])
                        2'd0: begin
                            selected_lo <= cache_word0;
                            selected_hi <= cache_word1;
                        end
                        2'd1: begin
                            selected_lo <= cache_word2;
                            selected_hi <= cache_word3;
                        end
                        2'd2: begin
                            selected_lo <= cache_word4;
                            selected_hi <= cache_word5;
                        end
                        default: begin
                            selected_lo <= cache_word6;
                            selected_hi <= cache_word7;
                        end
                    endcase
                    state <= S_CACHE_OUTPUT;
                end
                S_CACHE_OUTPUT: begin
                    rsp_word_mem <= chosen_lane[0] ? selected_hi : selected_lo;
                    state <= S_FINISH;
                end
                S_WAIT: begin
                    if (can_start_pcm) begin
                        // 8-word aligned base; eight contiguous words are
                        // two complete 4-word SDRAM physical groups.
                        requested_block <= req_addr_mem[20:3];
                        issue_addr <= {req_addr_mem[20:3], 3'b000};
                        issue_lane <= 3'd0;
                        response_lane <= 3'd0;
                        audio_read_lock <= 1'b1;
                        state <= S_GUARD;
                    end
                end
                S_GUARD: begin
                    // Abort ONLY before the first physical PCM command.
                    // The video writer/reader gets its priority back.
                    if (can_start_pcm && !video_App_rd_en) begin
                        if (grant_ticks > max_grant_ticks)
                            max_grant_ticks <= grant_ticks;
                        state <= S_ISSUE8;
                    end else begin
                        audio_read_lock <= 1'b0;
                        state <= S_WAIT;
                    end
                end
                S_ISSUE8: begin
                    // 8 consecutive App_rd_en cycles, no gaps or arbitration
                    // mid-burst. The SDRAM captures the current address
                    // before this nonblocking increment takes effect.
                    pcm_burst_owns_responses <= 1'b1;
                    issue_addr <= issue_addr + 21'd1;
                    if (issue_lane == 3'd7) begin
                        response_lane <= 3'd0;
                        state <= S_RECV8;
                    end else
                        issue_lane <= issue_lane + 3'd1;
                end
                S_RECV8: begin
                    // Return order matches 8 consecutive addresses.
                    // Never release VIDEO access after only 4 replies.
                    if (Sdr_rd_en && pcm_burst_owns_responses) begin
                        // Observe the first C4 word only on EXISTING piano
                        // replies. No extra SDRAM command and no change to
                        // cache, FIFO, or response routing.
                        if (response_lane == 3'd0 &&
                            requested_block == C4_FIRST_BLOCK) begin
                            c4_seen <= 1'b1;
                            if (c4_real_reads != 8'hff)
                                c4_real_reads <= c4_real_reads + 8'd1;
                            if (Sdr_rd_dout != C4_FIRST_WORD) begin
                                if (c4_bad_reads != 8'hff)
                                    c4_bad_reads <= c4_bad_reads + 8'd1;
                                if (!c4_bad_seen) begin
                                    c4_bad_seen <= 1'b1;
                                    first_bad_c4_word <= Sdr_rd_dout;
                                end
                            end
                        end
                        case (response_lane)
                            3'd0: cache_word0 <= Sdr_rd_dout;
                            3'd1: cache_word1 <= Sdr_rd_dout;
                            3'd2: cache_word2 <= Sdr_rd_dout;
                            3'd3: cache_word3 <= Sdr_rd_dout;
                            3'd4: cache_word4 <= Sdr_rd_dout;
                            3'd5: cache_word5 <= Sdr_rd_dout;
                            3'd6: cache_word6 <= Sdr_rd_dout;
                            default: cache_word7 <= Sdr_rd_dout;
                        endcase
                        if (response_lane == chosen_lane)
                            rsp_word_mem <= Sdr_rd_dout;
                        if (response_lane == 3'd7) begin
                            if (burst_ticks > max_burst_ticks)
                                max_burst_ticks <= burst_ticks;
                            cache_block <= requested_block;
                            cache_valid <= 1'b1;
                            pcm_burst_owns_responses <= 1'b0;
                            state <= S_FINISH;
                        end else
                            response_lane <= response_lane + 3'd1;
                    end
                end
                S_FINISH: begin
                    any_pcm_completed <= 1'b1;
                    rsp_toggle_mem <= ~rsp_toggle_mem;
                    audio_read_lock <= 1'b0;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
