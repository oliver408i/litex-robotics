// SNN-MLP inference core for the layered LIF MNIST classifier.
//
// Parameterizable in network shape (IN_SIZE -> HIDDEN -> OUT_SIZE), time
// horizon (TIMESTEPS), and MAC parallelism (N_MAC). Numerics mirror
// sim/snn_mlp.py and the snntorch QAT pipeline exactly: Q4.12 fixed-point,
// beta = 1 - 2^-BETA_SHIFT (0.875 by default), membrane clipped to +/-3.999,
// threshold 1.0 with subtract reset.
//
// Schedule (image-major):
//   1. Host writes pixels into pixel_mem (CSR write port).
//   2. Host writes biases into bias_mem (CSR write port).
//   3. start pulse -> for t in [0, TIMESTEPS):
//        for each layer-1 tile of N_MAC neurons:
//          IN_SIZE cycles of N_MAC-parallel MACs (weights streamed in)
//          1 cycle: shift + bias + leak + clip + threshold per neuron
//        same for layer 2 over the HIDDEN spike vector (no shift after MAC
//        since spike inputs are unitless 0/1).
//      After all timesteps, argmax(spk_count) -> classification.
//
// Weights are consumed via a ready/valid stream so the core is agnostic to
// the producer: in cocotb the testbench drives it directly, in the SoC a
// future SDRAM/Wishbone loader will fill it.
`default_nettype none

module snn_mlp_core #(
    parameter integer IN_SIZE    = 784,
    parameter integer HIDDEN     = 64,
    parameter integer OUT_SIZE   = 10,
    parameter integer TIMESTEPS  = 25,
    parameter integer N_MAC      = 1,
    parameter integer DATA_WIDTH = 16,
    parameter integer FRAC_BITS  = 12,
    parameter integer BETA_SHIFT = 3,
    parameter integer SPK_WIDTH  = 8
)(
    input  wire                                clk,
    input  wire                                rst,

    input  wire                                start,
    input  wire                                clear_state,
    output reg                                 busy,
    output reg                                 done,

    // Pixel buffer write port (host fills before start).
    input  wire                                pixel_we,
    input  wire [15:0]                         pixel_addr,
    input  wire signed [DATA_WIDTH-1:0]        pixel_data,

    // Bias buffer write port. First HIDDEN entries are layer-1 biases,
    // next OUT_SIZE entries are layer-2 biases.
    input  wire                                bias_we,
    input  wire [15:0]                         bias_addr,
    input  wire signed [DATA_WIDTH-1:0]        bias_data,

    // Weight stream: N_MAC Q4.12 words per beat.
    // Expected order, per timestep:
    //   for each layer-1 tile (0..L1_TILES-1):
    //     for each input i (0..IN_SIZE-1): beat[m]=W1[tile*N_MAC+m][i]  (pad 0 for partial tile)
    //   for each layer-2 tile (0..L2_TILES-1):
    //     for each hidden h (0..HIDDEN-1): beat[m]=W2[tile*N_MAC+m][h]  (pad 0 for partial tile)
    input  wire                                w_valid,
    output wire                                w_ready,
    input  wire signed [N_MAC*DATA_WIDTH-1:0]  w_data,

    // Result
    output reg  [3:0]                          classification,
    output reg                                 classification_valid,
    output reg  [OUT_SIZE*SPK_WIDTH-1:0]       spike_counts_packed
);

    // --- Derived sizes ---
    localparam integer L1_TILES  = (HIDDEN   + N_MAC - 1) / N_MAC;
    localparam integer L2_TILES  = (OUT_SIZE + N_MAC - 1) / N_MAC;
    localparam integer ACC_WIDTH = 48;

    // --- Q4.12 constants ---
    localparam signed [DATA_WIDTH-1:0] MEM_CLIP_Q   = 16'sd16380;  // ~3.999
    localparam signed [DATA_WIDTH-1:0] NEG_MEM_CLIP = -16'sd16380;
    localparam signed [DATA_WIDTH-1:0] THR_Q        = 16'sd4096;   // 1.0

    // --- Memories ---
    reg signed [DATA_WIDTH-1:0] pixel_mem [0:IN_SIZE-1];
    reg signed [DATA_WIDTH-1:0] bias_mem  [0:HIDDEN+OUT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] mem1      [0:HIDDEN-1];
    reg signed [DATA_WIDTH-1:0] mem2      [0:OUT_SIZE-1];
    reg                          spk1_buf [0:HIDDEN-1];
    reg        [SPK_WIDTH-1:0]   spk_count [0:OUT_SIZE-1];

    integer init_i;

    // BRAM write ports (host-side).
    always @(posedge clk) begin
        if (pixel_we && pixel_addr < IN_SIZE)
            pixel_mem[pixel_addr] <= pixel_data;
        if (bias_we && bias_addr < (HIDDEN + OUT_SIZE))
            bias_mem[bias_addr] <= bias_data;
    end

    // --- FSM ---
    // FIN is split into two stages so the leak+shift+bias ACC_WIDTH add and
    // the subsequent clip+threshold each get their own cycle. Without this
    // the lif_step combinational chain blew the 50 MHz period on ECP5.
    localparam [3:0]
        S_IDLE     = 4'd0,
        S_L1_MAC   = 4'd1,
        S_L1_FIN_A = 4'd2,
        S_L1_FIN_B = 4'd3,
        S_L2_MAC   = 4'd4,
        S_L2_FIN_A = 4'd5,
        S_L2_FIN_B = 4'd6,
        S_NEXT_T   = 4'd7,
        S_ARGMAX   = 4'd8,
        S_DONE     = 4'd9;

    reg [3:0]  state;
    reg [15:0] t_step;
    reg [15:0] tile_idx;
    reg [15:0] in_idx;
    reg [15:0] argmax_idx;
    reg [SPK_WIDTH-1:0] argmax_val;

    // MAC accumulator bank
    reg signed [ACC_WIDTH-1:0] mac_acc [0:N_MAC-1];

    // Stream handshake: we want a beat whenever we're in a MAC phase.
    assign w_ready = (state == S_L1_MAC) || (state == S_L2_MAC);

    // Per-MAC weight slice. Read combinationally; only consumed when w_valid.
    function automatic signed [DATA_WIDTH-1:0] w_slice;
        input integer idx;
        begin
            w_slice = w_data[idx*DATA_WIDTH +: DATA_WIDTH];
        end
    endfunction

    // Sign-extend Q4.12 (DATA_WIDTH) to ACC_WIDTH.
    function automatic signed [ACC_WIDTH-1:0] sxt;
        input signed [DATA_WIDTH-1:0] v;
        begin
            sxt = {{(ACC_WIDTH-DATA_WIDTH){v[DATA_WIDTH-1]}}, v};
        end
    endfunction

    // Pipelined LIF post-processing.
    // Stage A (combinational): produce pre = leak(prev_mem) + shifted(mac) + bias.
    //   This is the heavy carry chain (two stacked ACC_WIDTH adds). Register
    //   its output between cycles to break the 21 ns combinational path that
    //   was failing 50 MHz timing on ECP5.
    function automatic signed [ACC_WIDTH-1:0] lif_stage_a;
        input signed [DATA_WIDTH-1:0]   prev_mem;
        input signed [ACC_WIDTH-1:0]    mac_in;
        input signed [DATA_WIDTH-1:0]   bias;
        input                            apply_shift;
        reg   signed [ACC_WIDTH-1:0]    shifted;
        reg   signed [ACC_WIDTH-1:0]    leak;
        begin
            shifted     = apply_shift ? (mac_in >>> FRAC_BITS) : mac_in;
            leak        = sxt(prev_mem) - sxt(prev_mem >>> BETA_SHIFT);
            lif_stage_a = leak + shifted + sxt(bias);
        end
    endfunction

    // Stage B (combinational): clip + threshold reset on a registered `pre`.
    function automatic [DATA_WIDTH:0] lif_stage_b;
        input signed [ACC_WIDTH-1:0] pre;
        reg   signed [DATA_WIDTH-1:0] clipped;
        reg   signed [DATA_WIDTH-1:0] new_mem;
        reg                            spk;
        begin
            if (pre > sxt(MEM_CLIP_Q))        clipped = MEM_CLIP_Q;
            else if (pre < sxt(NEG_MEM_CLIP)) clipped = NEG_MEM_CLIP;
            else                              clipped = pre[DATA_WIDTH-1:0];
            if (clipped >= THR_Q) begin
                spk     = 1'b1;
                new_mem = clipped - THR_Q;
            end else begin
                spk     = 1'b0;
                new_mem = clipped;
            end
            lif_stage_b = {spk, new_mem};
        end
    endfunction

    // Pipeline register between FIN_A and FIN_B.
    reg signed [ACC_WIDTH-1:0] pre_reg [0:N_MAC-1];

    integer m;
    reg [15:0] neuron;
    reg [DATA_WIDTH:0] lif_result;

    always @(posedge clk) begin
        if (rst) begin
            state                <= S_IDLE;
            busy                 <= 1'b0;
            done                 <= 1'b0;
            classification       <= 4'd0;
            classification_valid <= 1'b0;
            spike_counts_packed  <= {OUT_SIZE*SPK_WIDTH{1'b0}};
            t_step               <= 16'd0;
            tile_idx             <= 16'd0;
            in_idx               <= 16'd0;
            argmax_idx           <= 16'd0;
            argmax_val           <= {SPK_WIDTH{1'b0}};
            for (init_i = 0; init_i < N_MAC;    init_i = init_i + 1) begin
                mac_acc[init_i] <= {ACC_WIDTH{1'b0}};
                pre_reg[init_i] <= {ACC_WIDTH{1'b0}};
            end
            for (init_i = 0; init_i < HIDDEN;   init_i = init_i + 1) begin mem1[init_i] <= 0; spk1_buf[init_i] <= 1'b0; end
            for (init_i = 0; init_i < OUT_SIZE; init_i = init_i + 1) begin mem2[init_i] <= 0; spk_count[init_i] <= 0; end
        end else if (clear_state && state == S_IDLE) begin
            classification_valid <= 1'b0;
            done                 <= 1'b0;
            for (init_i = 0; init_i < HIDDEN;   init_i = init_i + 1) begin mem1[init_i] <= 0; spk1_buf[init_i] <= 1'b0; end
            for (init_i = 0; init_i < OUT_SIZE; init_i = init_i + 1) begin mem2[init_i] <= 0; spk_count[init_i] <= 0; end
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy                 <= 1'b1;
                        done                 <= 1'b0;
                        classification       <= 4'd0;
                        classification_valid <= 1'b0;
                        t_step               <= 16'd0;
                        tile_idx             <= 16'd0;
                        in_idx               <= 16'd0;
                        argmax_idx           <= 16'd0;
                        argmax_val           <= {SPK_WIDTH{1'b0}};
                        for (m = 0; m < N_MAC;    m = m + 1) mac_acc[m]   <= {ACC_WIDTH{1'b0}};
                        for (m = 0; m < OUT_SIZE; m = m + 1) spk_count[m] <= 0;
                        state <= S_L1_MAC;
                    end
                end

                // Layer-1 MAC phase. One pixel per beat, N_MAC products in parallel.
                S_L1_MAC: begin
                    if (w_valid) begin
                        for (m = 0; m < N_MAC; m = m + 1) begin
                            mac_acc[m] <= mac_acc[m]
                                + $signed(pixel_mem[in_idx]) * w_slice(m);
                        end
                        if (in_idx == IN_SIZE - 1) begin
                            in_idx <= 16'd0;
                            state  <= S_L1_FIN_A;
                        end else begin
                            in_idx <= in_idx + 16'd1;
                        end
                    end
                end

                // Stage A: compute pre = leak + shifted + bias, register it.
                // Also clear the MAC accumulator for the next tile/layer.
                S_L1_FIN_A: begin
                    for (m = 0; m < N_MAC; m = m + 1) begin
                        neuron = tile_idx * N_MAC + m;
                        if (neuron < HIDDEN) begin
                            pre_reg[m] <= lif_stage_a(
                                mem1[neuron],
                                mac_acc[m],
                                bias_mem[neuron],
                                1'b1                 // apply >> FRAC_BITS for layer 1
                            );
                        end
                        mac_acc[m] <= {ACC_WIDTH{1'b0}};
                    end
                    state <= S_L1_FIN_B;
                end

                // Stage B: clip + threshold pre_reg, write back mem1/spk1_buf,
                // advance tile.
                S_L1_FIN_B: begin
                    for (m = 0; m < N_MAC; m = m + 1) begin
                        neuron = tile_idx * N_MAC + m;
                        if (neuron < HIDDEN) begin
                            lif_result = lif_stage_b(pre_reg[m]);
                            mem1[neuron]     <= lif_result[DATA_WIDTH-1:0];
                            spk1_buf[neuron] <= lif_result[DATA_WIDTH];
                        end
                    end
                    if (tile_idx == L1_TILES - 1) begin
                        tile_idx <= 16'd0;
                        in_idx   <= 16'd0;
                        state    <= S_L2_MAC;
                    end else begin
                        tile_idx <= tile_idx + 16'd1;
                        in_idx   <= 16'd0;
                        state    <= S_L1_MAC;
                    end
                end

                // Layer-2 MAC: spike-driven. Add weight only when spike present.
                // No post-shift since spike is unitless 0/1 and weight is already Q4.12.
                S_L2_MAC: begin
                    if (w_valid) begin
                        for (m = 0; m < N_MAC; m = m + 1) begin
                            if (spk1_buf[in_idx]) begin
                                mac_acc[m] <= mac_acc[m] + sxt(w_slice(m));
                            end
                        end
                        if (in_idx == HIDDEN - 1) begin
                            in_idx <= 16'd0;
                            state  <= S_L2_FIN_A;
                        end else begin
                            in_idx <= in_idx + 16'd1;
                        end
                    end
                end

                S_L2_FIN_A: begin
                    for (m = 0; m < N_MAC; m = m + 1) begin
                        neuron = tile_idx * N_MAC + m;
                        if (neuron < OUT_SIZE) begin
                            pre_reg[m] <= lif_stage_a(
                                mem2[neuron],
                                mac_acc[m],
                                bias_mem[HIDDEN + neuron],
                                1'b0                 // no post-shift for layer 2
                            );
                        end
                        mac_acc[m] <= {ACC_WIDTH{1'b0}};
                    end
                    state <= S_L2_FIN_B;
                end

                S_L2_FIN_B: begin
                    for (m = 0; m < N_MAC; m = m + 1) begin
                        neuron = tile_idx * N_MAC + m;
                        if (neuron < OUT_SIZE) begin
                            lif_result = lif_stage_b(pre_reg[m]);
                            mem2[neuron] <= lif_result[DATA_WIDTH-1:0];
                            if (lif_result[DATA_WIDTH]) begin
                                spk_count[neuron] <= spk_count[neuron] + 1'b1;
                            end
                        end
                    end
                    if (tile_idx == L2_TILES - 1) begin
                        tile_idx <= 16'd0;
                        in_idx   <= 16'd0;
                        state    <= S_NEXT_T;
                    end else begin
                        tile_idx <= tile_idx + 16'd1;
                        in_idx   <= 16'd0;
                        state    <= S_L2_MAC;
                    end
                end

                S_NEXT_T: begin
                    if (t_step == TIMESTEPS - 1) begin
                        argmax_idx <= 16'd0;
                        argmax_val <= {SPK_WIDTH{1'b0}};
                        state      <= S_ARGMAX;
                    end else begin
                        t_step <= t_step + 16'd1;
                        state  <= S_L1_MAC;
                    end
                end

                // Sequential argmax over OUT_SIZE counters.
                S_ARGMAX: begin
                    if (argmax_idx < OUT_SIZE) begin
                        if (spk_count[argmax_idx] > argmax_val) begin
                            argmax_val     <= spk_count[argmax_idx];
                            classification <= argmax_idx[3:0];
                        end
                        argmax_idx <= argmax_idx + 16'd1;
                    end else begin
                        for (m = 0; m < OUT_SIZE; m = m + 1) begin
                            spike_counts_packed[m*SPK_WIDTH +: SPK_WIDTH] <= spk_count[m];
                        end
                        classification_valid <= 1'b1;
                        state                <= S_DONE;
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
