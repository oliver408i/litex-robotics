module lif_bank_debug (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        clear_state,
    input  wire signed [15:0] measurement_in,
    input  wire [3:0]  input_override,
    output reg         busy,
    output reg         done,
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
    reg signed [15:0] clip_v [0:7];
    reg signed [15:0] feature_v [0:10];
    reg signed [15:0] feature_reg [0:10];
    reg signed [15:0] meas_feature_v;
    reg signed [15:0] delta_feature_v;
    reg signed [15:0] prev_measurement;
    reg signed [15:0] latched_measurement;
    reg signed [15:0] latched_delta;
    reg [3:0] latched_spikes;
    reg signed [23:0] acc;
    reg signed [31:0] feature_acc;
    reg signed [31:0] readout0_acc;
    reg signed [31:0] readout1_acc;
    reg [3:0] readout_idx;
    reg signed [31:0] readout0_term;
    reg signed [31:0] readout1_term;
    integer i;
    integer j;

    function signed [15:0] weight;
        input integer n;
        input integer j;
        begin
            case (n)
                0: case (j) 0: weight=-16'sd866; 1: weight=-16'sd1716; 2: weight=16'sd742; 3: weight=-16'sd2102; endcase
                1: case (j) 0: weight=16'sd176; 1: weight=-16'sd660; 2: weight=-16'sd2173; 3: weight=16'sd37; endcase
                2: case (j) 0: weight=-16'sd2273; 1: weight=-16'sd326; 2: weight=-16'sd2114; 3: weight=-16'sd2012; endcase
                3: case (j) 0: weight=-16'sd371; 1: weight=16'sd1607; 2: weight=-16'sd1849; 3: weight=-16'sd1360; endcase
                4: case (j) 0: weight=16'sd626; 1: weight=16'sd2201; 2: weight=16'sd379; 3: weight=-16'sd508; endcase
                5: case (j) 0: weight=16'sd2341; 1: weight=-16'sd2229; 2: weight=16'sd1762; 3: weight=-16'sd1034; endcase
                6: case (j) 0: weight=-16'sd1749; 1: weight=-16'sd1879; 2: weight=-16'sd941; 3: weight=16'sd1554; endcase
                default: case (j) 0: weight=-16'sd1569; 1: weight=16'sd401; 2: weight=16'sd683; 3: weight=-16'sd627; endcase
            endcase
        end
    endfunction

    function signed [15:0] readout_weight;
        input integer out_idx;
        input integer idx;
        begin
            if (out_idx == 0) begin
                case (idx)
                    0: readout_weight = -16'sd1012;
                    1: readout_weight =  16'sd1021;
                    2: readout_weight = -16'sd62;
                    3: readout_weight = -16'sd232;
                    4: readout_weight =  16'sd20;
                    5: readout_weight =  16'sd441;
                    6: readout_weight = -16'sd12;
                    7: readout_weight =  16'sd1298;
                    8: readout_weight =  16'sd12219;
                    9: readout_weight = -16'sd9146;
                    default: readout_weight = 16'sd22;
                endcase
            end else begin
                case (idx)
                    0: readout_weight = 16'sd71;
                    1: readout_weight = -16'sd2223;
                    2: readout_weight = 16'sd427;
                    3: readout_weight = 16'sd201;
                    4: readout_weight = -16'sd1;
                    5: readout_weight = 16'sd746;
                    6: readout_weight = -16'sd470;
                    7: readout_weight = -16'sd361;
                    8: readout_weight = 16'sd3016;
                    9: readout_weight = 16'sd681;
                    default: readout_weight = -16'sd690;
                endcase
            end
        end
    endfunction

    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            beta_v[i] = membrane[i] - (membrane[i] >>> 3);
            isum_v[i] = 16'sd0;
            if (latched_spikes[0]) isum_v[i] = isum_v[i] + weight(i, 0);
            if (latched_spikes[1]) isum_v[i] = isum_v[i] + weight(i, 1);
            if (latched_spikes[2]) isum_v[i] = isum_v[i] + weight(i, 2);
            if (latched_spikes[3]) isum_v[i] = isum_v[i] + weight(i, 3);
            acc = beta_v[i] + isum_v[i];
            if (acc > MEM_CLIP) clip_v[i] = MEM_CLIP;
            else if (acc < NEG_MEM_CLIP) clip_v[i] = NEG_MEM_CLIP;
            else clip_v[i] = acc[15:0];
            if (clip_v[i] >= THRESHOLD) membrane_next[i] = clip_v[i] - THRESHOLD;
            else membrane_next[i] = clip_v[i];
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

        readout0_term = (feature_reg[readout_idx] * readout_weight(0, readout_idx)) >>> 12;
        readout1_term = (feature_reg[readout_idx] * readout_weight(1, readout_idx)) >>> 12;
    end

    always @(posedge clk) begin
        if (rst || clear_state) begin
            busy <= 0; done <= 0; phase <= 0; cycles <= 0;
            prev_measurement <= 0; latched_measurement <= 0; latched_delta <= 0; latched_spikes <= 0;
            readout0_acc <= 0; readout1_acc <= 0; readout_idx <= 0;
            position_out <= 0; velocity_out <= 0; debug_measurement_raw <= 0; debug_delta_raw <= 0;
            debug_measurement_feature <= 0; debug_delta_feature <= 0; debug_membrane0 <= 0; debug_membrane1 <= 0;
            debug_membrane2 <= 0; debug_membrane3 <= 0; debug_input_spikes <= 0; debug_beta_product <= 0;
            debug_input_sum <= 0; debug_recurrent_sum <= 0; debug_membrane_clip <= 0; debug_packed <= 0;
            for (i = 0; i < 8; i = i + 1) membrane[i] <= 0;
            for (i = 0; i < 11; i = i + 1) feature_reg[i] <= 0;
        end else if (start && !busy) begin
            busy <= 1; done <= 0; phase <= 1; cycles <= 0;
            latched_measurement <= measurement_in;
            latched_delta <= measurement_in - prev_measurement;
            prev_measurement <= measurement_in;
            latched_spikes <= input_override;
        end else if (busy) begin
            cycles <= cycles + 16'd1;
            if (phase == 1) begin
                for (i = 0; i < 8; i = i + 1) membrane[i] <= membrane_next[i];
                for (i = 0; i < 11; i = i + 1) feature_reg[i] <= feature_v[i];
                readout0_acc <= 0;
                readout1_acc <= 0;
                readout_idx <= 0;
                phase <= 2;
                debug_measurement_raw <= latched_measurement;
                debug_delta_raw <= latched_delta;
                debug_measurement_feature <= meas_feature_v;
                debug_delta_feature <= delta_feature_v;
                debug_membrane0 <= membrane_next[0];
                debug_membrane1 <= membrane_next[1];
                debug_membrane2 <= membrane_next[2];
                debug_membrane3 <= membrane_next[3];
                debug_input_spikes <= latched_spikes;
                debug_beta_product <= beta_v[0];
                debug_input_sum <= isum_v[0];
                debug_recurrent_sum <= 0;
                debug_membrane_clip <= clip_v[0];
                debug_packed <= {20'd0, 1'b0, 1'b1, 2'd1, latched_spikes, 4'd0};
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
