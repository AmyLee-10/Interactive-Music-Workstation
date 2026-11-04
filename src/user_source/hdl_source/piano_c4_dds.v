
// ============================================================================
// Step7: 单音C4电子琴最小链路
// 48kHz采样域DDS音源
// 后续可替换为PCM钢琴采样
// ============================================================================
module piano_c4_dds(
    input  wire        clk_audio,
    input  wire        rst,
    input  wire        note_on,
    output reg signed [23:0] pcm_out
);

parameter integer PHASE_BITS = 24;
parameter integer PHASE_STEP = 91590; // 261.63Hz @ 48kHz

reg [PHASE_BITS-1:0] phase;
reg [15:0] volume;

always @(posedge clk_audio or posedge rst) begin
    if(rst) begin
        phase <= 0;
        volume <= 0;
        pcm_out <= 0;
    end
    else begin
        phase <= phase + PHASE_STEP;

        if(note_on)
            volume <= 16'd32768;
        else if(volume > 16'd32)
            volume <= volume - 16'd32;
        else
            volume <= 0;

        // 简化钢琴感：基础波形+衰减包络
        pcm_out <= ($signed({phase[23],phase[22:8],8'd0}) *
                    $signed({1'b0,volume[15:1]})) >>> 15;
    end
end

endmodule
