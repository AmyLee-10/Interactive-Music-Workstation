// E9.3: SD (100 MHz) -> SDRAM (user mem_clk) PCM16 bulk transfer.
// The existing video frame writer retains priority between 128-word bursts.
// A missing/overflowed bank never asserts bank_loaded; no partial bank plays.
module e93_sdram_bank_writer #(
    parameter [20:0] AUDIO_BASE = 21'd655360,
    parameter [20:0] AUDIO_WORDS = 21'd844672,
    parameter [20:0] VIOLIN_BASE = 21'd1500032,
    parameter [20:0] VIOLIN_WORDS = 21'd480000
)(
    input wire rst,
    input wire sd_clk, mem_clk,
    input wire sd_bank_start,
    input wire [20:0] sd_bank_words,
    input wire sd_pcm_en,
    input wire [31:0] sd_pcm_word,
    input wire sd_scan_complete,
    input wire sdram_init_done,
    input wire sdram_ref_vld,
    input wire sdram_busy,
    input wire higher_priority_busy,
    input wire audio_read_lock,
    output wire bank_write_busy,
    output wire bank_loaded_mem,
    output wire violin_loaded_mem,
    output reg fifo_overflow_sd,
    output wire App_wr_en,
    output wire [20:0] App_wr_addr,
    output wire [31:0] App_wr_data
);
    reg request_sd;
    reg capture_sd;
    wire write_ack_mem, write_finish_mem;
    reg [2:0] ack_sync_sd;
    reg [2:0] fin_sync_sd;
    reg [2:0] done_sync_mem;
    reg [2:0] overflow_sync_mem;
    reg [1:0] start_sync_mem;
    reg bank_started_mem;
    reg bank_loaded_reg;
    reg violin_loaded_reg;
    reg target_violin_sd;
    reg [1:0] target_violin_mem_sync;
    reg active_violin_mem;
    wire target_violin_mem = target_violin_mem_sync[1];
    wire [20:0] target_base_mem = target_violin_mem ?
                                 VIOLIN_BASE : AUDIO_BASE;
    wire [20:0] target_words_mem = target_violin_mem ?
                                  VIOLIN_WORDS : AUDIO_WORDS;
    // E9.3.9: source and DMA completion are asynchronous in arrival order.
    // Latch both events; never require the pulses to coincide.
    reg source_done_seen_sd;
    reg start_prev_mem;
    reg dma_done_seen_mem;
    wire fifo_full;
    wire fifo_clear;
    wire [9:0] fifo_rdusedw;
    wire [31:0] fifo_out;
    wire writer_pending;

    // Do not feed this FIFO unless the exact header has been recognized.
    wire valid_input = sd_pcm_en && capture_sd;
    wfifo_32_32_512 pcm_write_fifo (
        .clkr(mem_clk),
        .clkw(sd_clk),
        .rst(fifo_clear | rst),
        .we(valid_input && !fifo_full),
        .re(App_wr_en),
        .di(sd_pcm_word),
        .empty_flag(),
        .full_flag(fifo_full),
        .wrusedw(),
        .rdusedw(fifo_rdusedw),
        .dout(fifo_out)
    );
    assign App_wr_data = fifo_out;

    always @(posedge sd_clk or posedge rst) begin
        if (rst) begin
            request_sd <= 1'b0;
            capture_sd <= 1'b0;
            ack_sync_sd <= 3'b0;
            fin_sync_sd <= 3'b0;
            fifo_overflow_sd <= 1'b0;
            source_done_seen_sd <= 1'b0;
            target_violin_sd <= 1'b0;
        end else begin
            ack_sync_sd <= {ack_sync_sd[1:0], write_ack_mem};
            fin_sync_sd <= {fin_sync_sd[1:0], write_finish_mem};
            if (sd_bank_start &&
                (sd_bank_words == AUDIO_WORDS ||
                 sd_bank_words == VIOLIN_WORDS)) begin
                request_sd <= 1'b1;
                capture_sd <= 1'b1;
                target_violin_sd <= (sd_bank_words == VIOLIN_WORDS);
                fifo_overflow_sd <= 1'b0;
                source_done_seen_sd <= 1'b0;
            end else if (ack_sync_sd[2])
                request_sd <= 1'b0;
            // sd_scan_complete may be a single sd_clk pulse.
            // Make it a level before synchronizing into the memory clock.
            if (sd_scan_complete) begin
                capture_sd <= 1'b0;
                source_done_seen_sd <= 1'b1;
            end
            if (valid_input && fifo_full)
                fifo_overflow_sd <= 1'b1;
        end
    end

    frame_fifo_write #(
        .MEM_DATA_BITS(32),
        .ADDR_BITS(21),
        .BURST_BITS(9),
        .BURST_SIZE(128),
        .WRITE_V_FLIP(0)
    ) pcm_dma (
        .rst(rst), .mem_clk(mem_clk),
        .Sdr_init_done(sdram_init_done),
        .Sdr_init_ref_vld(sdram_ref_vld),
        .Sdr_busy(sdram_busy),
        // E9.3.28: PCM DMA must not START a burst during refresh/busy.
        .App_rd_busy(higher_priority_busy | audio_read_lock |
                     sdram_ref_vld | sdram_busy),
        .O_wr_busy(bank_write_busy),
        .O_wr_pending(writer_pending),
        .App_wr_en(App_wr_en),
        .App_wr_addr(App_wr_addr),
        .write_req(request_sd),
        .write_req_ack(write_ack_mem),
        .write_finish(write_finish_mem),
        // SDRAM base and word count are stable for an entire DMA burst.
        // Both banks use the ONE existing FIFO and frame_fifo_write.
        .write_addr_0(target_base_mem),
        .write_addr_1(21'd0), .write_addr_2(21'd0),
        .write_addr_3(21'd0),
        .write_addr_index(2'd0),
        .write_len(target_words_mem),
        .fifo_aclr(fifo_clear),
        .rdusedw(fifo_rdusedw)
    );

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            done_sync_mem <= 3'b0;
            overflow_sync_mem <= 3'b0;
            start_sync_mem <= 2'b0;
            start_prev_mem <= 1'b0;
            dma_done_seen_mem <= 1'b0;
            bank_started_mem <= 1'b0;
            bank_loaded_reg <= 1'b0;
            violin_loaded_reg <= 1'b0;
            target_violin_mem_sync <= 2'b00;
            active_violin_mem <= 1'b0;
        end else begin
            target_violin_mem_sync <= {target_violin_mem_sync[0],
                                      target_violin_sd};
            done_sync_mem <= {done_sync_mem[1:0], source_done_seen_sd};
            overflow_sync_mem <= {overflow_sync_mem[1:0], fifo_overflow_sd};
            start_sync_mem <= {start_sync_mem[0], request_sd};
            start_prev_mem <= start_sync_mem[1];
            // Reset one-bank completion accounting on the request rising edge.
            if (start_sync_mem[1] && !start_prev_mem) begin
                bank_started_mem <= 1'b1;
                active_violin_mem <= target_violin_mem;
                // Loading VIOLIN must NEVER invalidate good PIANO audio.
                if(target_violin_mem)
                    violin_loaded_reg <= 1'b0;
                else begin
                    bank_loaded_reg <= 1'b0;
                    violin_loaded_reg <= 1'b0;
                end
                dma_done_seen_mem <= 1'b0;
            end else if (write_finish_mem) begin
                dma_done_seen_mem <= 1'b1;
            end
            // Neither completion order can now lose the bank_loaded event.
            // Overflow always vetoes bank_loaded.
            if (bank_started_mem && dma_done_seen_mem &&
                done_sync_mem[2] && !overflow_sync_mem[2]) begin
                if(active_violin_mem)
                    violin_loaded_reg <= 1'b1;
                else
                    bank_loaded_reg <= 1'b1;
            end
            if (overflow_sync_mem[2]) begin
                if(active_violin_mem)
                    violin_loaded_reg <= 1'b0;
                else
                    bank_loaded_reg <= 1'b0;
            end
        end
    end
    assign bank_loaded_mem = bank_loaded_reg;
    assign violin_loaded_mem = violin_loaded_reg;
endmodule
