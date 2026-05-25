module lif_bank_debug (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        clear_state,
    input  wire        weight_write,
    input  wire        weight_commit,
    input  wire        weight_clear,
    input  wire [6:0]  weight_addr,
    input  wire signed [15:0] weight_data,
    input  wire signed [15:0] measurement_in,
    input  wire [3:0]  input_override,
    output reg         busy,
    output reg         done,
    output reg         model_ready,
    output reg  [1:0]  phase,
    output reg  [15:0] cycles,
    output reg signed [15:0] position_out,
    output reg signed [15:0] velocity_out,
    output reg signed [15:0] debug_measurement_raw,
    output reg signed [15:0] debug_delta_raw,
    output reg signed [15:0] debug_measurement_feature,
    output reg signed [15:0] debug_delta_feature,
    output reg signed [15:0] debug_membrane0,
    output reg signed [15:0] debug_membrane1,
    output reg signed [15:0] debug_membrane2,
    output reg signed [15:0] debug_membrane3,
    output reg signed [15:0] debug_membrane4,
    output reg        [3:0]  debug_input_spikes,
    output reg signed [15:0] debug_beta_product,
    output reg signed [15:0] debug_input_sum,
    output reg signed [15:0] debug_recurrent_sum,
    output reg signed [15:0] debug_membrane_clip,
    output reg        [31:0] debug_packed
);
    localparam signed [15:0] MEM_CLIP = 16'sd16380;
    localparam signed [15:0] NEG_MEM_CLIP = -16'sd16380;
    localparam signed [15:0] THRESHOLD = 16'sd4096;
    localparam signed [15:0] FEATURE_CLIP = 16'sd8188;      // ~1.999 in Q4.12
    localparam signed [15:0] NEG_FEATURE_CLIP = -16'sd8188;
    localparam signed [15:0] MEAS_SCALE = 16'sd1638;        // ~1/2.5 in Q4.12
    localparam signed [15:0] DELTA_SCALE = 16'sd819;        // ~1/5.0 in Q4.12

    reg signed [15:0] membrane [0:7];
    reg signed [15:0] membrane_next [0:7];
    reg signed [15:0] beta_v [0:7];
    reg signed [15:0] isum_v [0:7];
    reg signed [15:0] rsum_lo_v [0:7];
    reg signed [15:0] rsum_hi_v [0:7];
    reg signed [15:0] rsum_v [0:7];
    // Pipeline registers between stage A (carry-chain heavy partial sums)
    // and stage B (combine + clip + threshold). Splits the long
    // prev_spikes -> rsum -> clip -> next_spikes critical path in two.
    reg signed [15:0] beta_r [0:7];
    reg signed [15:0] isum_r [0:7];
    reg signed [15:0] rsum_lo_r [0:7];
    reg signed [15:0] rsum_hi_r [0:7];
    reg               phase1_step;
    reg signed [15:0] clip_v [0:7];
    reg signed [15:0] feature_v [0:10];
    reg signed [15:0] feature_reg [0:10];
    reg signed [15:0] meas_feature_v;
    reg signed [15:0] delta_feature_v;
    reg signed [15:0] prev_measurement;
    reg signed [15:0] latched_measurement;
    reg signed [15:0] latched_delta;
    reg [3:0] latched_spikes;
    reg [7:0] prev_spikes;
    reg [7:0] next_spikes_v;
    reg signed [23:0] acc;
    reg signed [31:0] feature_acc;
    reg signed [31:0] readout0_acc;
    reg signed [31:0] readout1_acc;
    reg [3:0] readout_idx;
    reg signed [31:0] readout0_term;
    reg signed [31:0] readout1_term;
    reg signed [15:0] input_weight [0:31];
    reg signed [15:0] recurrent_weight [0:63];
    reg signed [15:0] readout_weight [0:21];
    integer i;
    integer j;

    function signed [15:0] input_weight_at;
        input integer n;
        input integer j;
        begin
            input_weight_at = input_weight[(n * 4) + j];
        end
    endfunction

    function signed [15:0] readout_weight_at;
        input integer out_idx;
        input integer idx;
        begin
            readout_weight_at = readout_weight[(out_idx * 11) + idx];
        end
    endfunction

    function signed [15:0] recurrent_weight_at;
        input integer n;
        input integer j;
        begin
            recurrent_weight_at = recurrent_weight[(n * 8) + j];
        end
    endfunction

    // Stage A: short carry chains from prev_spikes/latched_spikes/membrane.
    // rsum is computed as two parallel half-sums so neither cascade is
    // longer than 4 conditional adds.
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            beta_v[i] = membrane[i] - (membrane[i] >>> 3);
            isum_v[i] = 16'sd0;
            if (latched_spikes[0]) isum_v[i] = isum_v[i] + input_weight_at(i, 0);
            if (latched_spikes[1]) isum_v[i] = isum_v[i] + input_weight_at(i, 1);
            if (latched_spikes[2]) isum_v[i] = isum_v[i] + input_weight_at(i, 2);
            if (latched_spikes[3]) isum_v[i] = isum_v[i] + input_weight_at(i, 3);
            rsum_lo_v[i] = 16'sd0;
            if (prev_spikes[0]) rsum_lo_v[i] = rsum_lo_v[i] + recurrent_weight_at(i, 0);
            if (prev_spikes[1]) rsum_lo_v[i] = rsum_lo_v[i] + recurrent_weight_at(i, 1);
            if (prev_spikes[2]) rsum_lo_v[i] = rsum_lo_v[i] + recurrent_weight_at(i, 2);
            if (prev_spikes[3]) rsum_lo_v[i] = rsum_lo_v[i] + recurrent_weight_at(i, 3);
            rsum_hi_v[i] = 16'sd0;
            if (prev_spikes[4]) rsum_hi_v[i] = rsum_hi_v[i] + recurrent_weight_at(i, 4);
            if (prev_spikes[5]) rsum_hi_v[i] = rsum_hi_v[i] + recurrent_weight_at(i, 5);
            if (prev_spikes[6]) rsum_hi_v[i] = rsum_hi_v[i] + recurrent_weight_at(i, 6);
            if (prev_spikes[7]) rsum_hi_v[i] = rsum_hi_v[i] + recurrent_weight_at(i, 7);
        end
    end

    // Stage B: combine pipelined partial sums and finish clip/threshold.
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            rsum_v[i] = rsum_lo_r[i] + rsum_hi_r[i];
            acc = beta_r[i] + isum_r[i] + rsum_v[i];
            if (acc > MEM_CLIP) clip_v[i] = MEM_CLIP;
            else if (acc < NEG_MEM_CLIP) clip_v[i] = NEG_MEM_CLIP;
            else clip_v[i] = acc[15:0];
            if (clip_v[i] >= THRESHOLD) begin
                membrane_next[i] = clip_v[i] - THRESHOLD;
                next_spikes_v[i] = 1'b1;
            end else begin
                membrane_next[i] = clip_v[i];
                next_spikes_v[i] = 1'b0;
            end
        end

        feature_acc = (latched_measurement * MEAS_SCALE) >>> 12;
        if (feature_acc > FEATURE_CLIP)
            meas_feature_v = FEATURE_CLIP;
        else if (feature_acc < NEG_FEATURE_CLIP)
            meas_feature_v = NEG_FEATURE_CLIP;
        else
            meas_feature_v = feature_acc[15:0];

        feature_acc = (latched_delta * DELTA_SCALE) >>> 12;
        if (feature_acc > FEATURE_CLIP)
            delta_feature_v = FEATURE_CLIP;
        else if (feature_acc < NEG_FEATURE_CLIP)
            delta_feature_v = NEG_FEATURE_CLIP;
        else
            delta_feature_v = feature_acc[15:0];

        for (i = 0; i < 8; i = i + 1)
            feature_v[i] = membrane_next[i];
        feature_v[8] = meas_feature_v;
        feature_v[9] = delta_feature_v;
        feature_v[10] = 16'sd4096;

        readout0_term = (feature_reg[readout_idx] * readout_weight_at(0, readout_idx)) >>> 12;
        readout1_term = (feature_reg[readout_idx] * readout_weight_at(1, readout_idx)) >>> 12;
    end

    always @(posedge clk) begin
        if (rst || clear_state) begin
            busy <= 0; done <= 0; phase <= 0; cycles <= 0;
            phase1_step <= 0;
            prev_measurement <= 0; latched_measurement <= 0; latched_delta <= 0; latched_spikes <= 0;
            prev_spikes <= 0;
            readout0_acc <= 0; readout1_acc <= 0; readout_idx <= 0;
            position_out <= 0; velocity_out <= 0; debug_measurement_raw <= 0; debug_delta_raw <= 0;
            debug_measurement_feature <= 0; debug_delta_feature <= 0; debug_membrane0 <= 0; debug_membrane1 <= 0;
            debug_membrane2 <= 0; debug_membrane3 <= 0; debug_membrane4 <= 0; debug_input_spikes <= 0; debug_beta_product <= 0;
            debug_input_sum <= 0; debug_recurrent_sum <= 0; debug_membrane_clip <= 0; debug_packed <= 0;
            for (i = 0; i < 8; i = i + 1) membrane[i] <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                beta_r[i] <= 0; isum_r[i] <= 0;
                rsum_lo_r[i] <= 0; rsum_hi_r[i] <= 0;
            end
            for (i = 0; i < 11; i = i + 1) feature_reg[i] <= 0;
            if (rst) begin
                model_ready <= 0;
                for (i = 0; i < 32; i = i + 1) input_weight[i] <= 0;
                for (i = 0; i < 64; i = i + 1) recurrent_weight[i] <= 0;
                for (i = 0; i < 22; i = i + 1) readout_weight[i] <= 0;
            end
        end else if (weight_clear && !busy) begin
            model_ready <= 0;
            done <= 0;
            for (i = 0; i < 32; i = i + 1) input_weight[i] <= 0;
            for (i = 0; i < 64; i = i + 1) recurrent_weight[i] <= 0;
            for (i = 0; i < 22; i = i + 1) readout_weight[i] <= 0;
        end else if (weight_write && !busy) begin
            model_ready <= 0;
            done <= 0;
            if (weight_addr < 7'd32)
                input_weight[weight_addr] <= weight_data;
            else if (weight_addr < 7'd96)
                recurrent_weight[weight_addr - 7'd32] <= weight_data;
            else if (weight_addr < 7'd118)
                readout_weight[weight_addr - 7'd96] <= weight_data;
        end else if (weight_commit && !busy) begin
            model_ready <= 1;
            done <= 0;
        end else if (start && !busy && model_ready) begin
            busy <= 1; done <= 0; phase <= 1; cycles <= 0;
            phase1_step <= 1'b0;
            latched_measurement <= measurement_in;
            latched_delta <= measurement_in - prev_measurement;
            prev_measurement <= measurement_in;
            latched_spikes <= input_override;
        end else if (busy) begin
            cycles <= cycles + 16'd1;
            if (phase == 1) begin
                if (phase1_step == 1'b0) begin
                    // Stage A: capture short-chain partial sums.
                    for (i = 0; i < 8; i = i + 1) begin
                        beta_r[i]    <= beta_v[i];
                        isum_r[i]    <= isum_v[i];
                        rsum_lo_r[i] <= rsum_lo_v[i];
                        rsum_hi_r[i] <= rsum_hi_v[i];
                    end
                    phase1_step <= 1'b1;
                end else begin
                    // Stage B: combine, clip, threshold, register state.
                    for (i = 0; i < 8; i = i + 1) membrane[i] <= membrane_next[i];
                    prev_spikes <= next_spikes_v;
                    for (i = 0; i < 11; i = i + 1) feature_reg[i] <= feature_v[i];
                    readout0_acc <= 0;
                    readout1_acc <= 0;
                    readout_idx <= 0;
                    phase <= 2;
                    phase1_step <= 1'b0;
                    debug_measurement_raw <= latched_measurement;
                    debug_delta_raw <= latched_delta;
                    debug_measurement_feature <= meas_feature_v;
                    debug_delta_feature <= delta_feature_v;
                    debug_membrane0 <= membrane_next[0];
                    debug_membrane1 <= membrane_next[1];
                    debug_membrane2 <= membrane_next[2];
                    debug_membrane3 <= membrane_next[3];
                    debug_membrane4 <= membrane_next[4];
                    debug_input_spikes <= latched_spikes;
                    debug_beta_product <= beta_r[0];
                    debug_input_sum <= isum_r[0];
                    debug_recurrent_sum <= rsum_v[0];
                    debug_membrane_clip <= clip_v[0];
                    debug_packed <= {20'd0, 1'b0, 1'b1, 2'd1, latched_spikes, 4'd0};
                end
            end else if (phase == 2) begin
                readout0_acc <= readout0_acc + readout0_term;
                readout1_acc <= readout1_acc + readout1_term;
                if (readout_idx == 4'd10) begin
                    phase <= 3;
                end else begin
                    readout_idx <= readout_idx + 4'd1;
                end
            end else begin
                if (readout0_acc > MEM_CLIP) position_out <= MEM_CLIP;
                else if (readout0_acc < NEG_MEM_CLIP) position_out <= NEG_MEM_CLIP;
                else position_out <= readout0_acc[15:0];

                if (readout1_acc > MEM_CLIP) velocity_out <= MEM_CLIP;
                else if (readout1_acc < NEG_MEM_CLIP) velocity_out <= NEG_MEM_CLIP;
                else velocity_out <= readout1_acc[15:0];

                busy <= 0;
                done <= 1;
                phase <= 0;
                debug_packed <= {20'd0, 1'b1, 1'b0, 2'd0, latched_spikes, 4'd0};
            end
        end
    end
endmodule
