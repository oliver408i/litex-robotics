// Wishbone-master weight stream loader for snn_mlp_core.
//
// Reads weight words sequentially from SDRAM (via a Wishbone-classic master
// port) and emits them on a ready/valid stream the core consumes.
//
// Two-segment schedule (matches the core hoisting layer-1 out of the timestep
// loop, see verilog/snn_mlp_core.v):
//   1. Preamble: stream `preamble_beats` beats from `base_addr` exactly ONCE
//      (the layer-1 weights W1; the core's constant layer-1 current is computed
//      from these a single time).
//   2. Cycles: stream `beats_per_cycle` beats `num_cycles` times (the layer-2
//      weights W2, replayed once per timestep). These live immediately after
//      the preamble in SDRAM, so the cycle base is derived as
//      base_addr + preamble_beats*WB_BYTES — no separate base CSR needed.
//
// Setting preamble_beats=0 collapses to the old single-segment behavior
// (replay `beats_per_cycle` from `base_addr` `num_cycles` times).
//
// Per beat: one Wishbone read; data captured in `w_data` and held until the
// downstream `w_ready` accepts it. WB_DATA_WIDTH must be >= N_MAC*DATA_WIDTH;
// the low bits feed the stream and the high bits (if any) are ignored.
//
// Simple Wishbone classic master: assert cyc/stb until ack, capture data,
// deassert, advance address. No bursts, no pipelining — bandwidth is
// dominated by the per-beat stall on the downstream core anyway (the core
// is ~13 cycles per L1 input cycle when N_MAC=1, so SDRAM has plenty of
// headroom).
`default_nettype none

module snn_weight_loader #(
    parameter integer WB_DATA_WIDTH = 32,
    parameter integer WB_ADDR_WIDTH = 32,
    parameter integer N_MAC         = 1,
    parameter integer DATA_WIDTH    = 16
)(
    input  wire                              clk,
    input  wire                              rst,

    // Control
    input  wire                              start,
    input  wire [WB_ADDR_WIDTH-1:0]          base_addr,    // SDRAM byte address
    input  wire [31:0]                       preamble_beats,  // beats streamed ONCE up front (W1); 0 = none
    input  wire [31:0]                       beats_per_cycle,
    input  wire [31:0]                       num_cycles,
    output reg                               busy,
    output reg                               done,

    // Wishbone classic master (read only)
    output reg                               wb_cyc,
    output reg                               wb_stb,
    output wire                              wb_we,
    output reg  [WB_ADDR_WIDTH-1:0]          wb_adr,
    output wire [WB_DATA_WIDTH-1:0]          wb_dat_w,
    output wire [WB_DATA_WIDTH/8-1:0]        wb_sel,
    input  wire                              wb_ack,
    input  wire [WB_DATA_WIDTH-1:0]          wb_dat_r,

    // Stream out to snn_mlp_core
    output reg                               w_valid,
    input  wire                              w_ready,
    output reg  [N_MAC*DATA_WIDTH-1:0]       w_data
);
    localparam integer WB_BYTES = WB_DATA_WIDTH / 8;

    // Read-only master.
    assign wb_we    = 1'b0;
    assign wb_dat_w = {WB_DATA_WIDTH{1'b0}};
    assign wb_sel   = {(WB_DATA_WIDTH/8){1'b1}};

    localparam [2:0]
        S_IDLE = 3'd0,
        S_READ = 3'd1,   // wb_cyc/wb_stb asserted, waiting for wb_ack
        S_HOLD = 3'd2,   // w_valid asserted, waiting for w_ready
        S_DONE = 3'd3;

    reg [2:0]  state;
    reg [31:0] beat_idx;    // beat within current segment block
    reg [31:0] cycle_idx;   // outer iteration (timestep)
    reg        in_preamble; // 1 while streaming the one-shot W1 prefix
    reg [WB_ADDR_WIDTH-1:0] cyc_base;  // base of the repeated (W2) block

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            wb_cyc    <= 1'b0;
            wb_stb    <= 1'b0;
            wb_adr    <= {WB_ADDR_WIDTH{1'b0}};
            w_valid     <= 1'b0;
            w_data      <= {N_MAC*DATA_WIDTH{1'b0}};
            beat_idx    <= 32'd0;
            cycle_idx   <= 32'd0;
            in_preamble <= 1'b0;
            cyc_base    <= {WB_ADDR_WIDTH{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy        <= 1'b1;
                        done        <= 1'b0;
                        beat_idx    <= 32'd0;
                        cycle_idx   <= 32'd0;
                        // Repeated (W2) block sits right after the preamble.
                        // With preamble_beats=0 this is just base_addr.
                        cyc_base    <= base_addr + preamble_beats * WB_BYTES;
                        in_preamble <= (preamble_beats != 32'd0);
                        wb_adr      <= base_addr;
                        wb_cyc      <= 1'b1;
                        wb_stb      <= 1'b1;
                        state       <= S_READ;
                    end
                end

                S_READ: begin
                    if (wb_ack) begin
                        wb_cyc  <= 1'b0;
                        wb_stb  <= 1'b0;
                        w_data  <= wb_dat_r[N_MAC*DATA_WIDTH-1:0];
                        w_valid <= 1'b1;
                        state   <= S_HOLD;
                    end
                end

                S_HOLD: begin
                    if (w_ready) begin
                        w_valid <= 1'b0;
                        if (in_preamble) begin
                            // One-shot W1 prefix. When done, fall into the
                            // repeated (W2) block starting at cyc_base.
                            if (beat_idx == preamble_beats - 32'd1) begin
                                in_preamble <= 1'b0;
                                beat_idx    <= 32'd0;
                                cycle_idx   <= 32'd0;
                                wb_adr      <= cyc_base;
                                wb_cyc      <= 1'b1;
                                wb_stb      <= 1'b1;
                                state       <= S_READ;
                            end else begin
                                beat_idx <= beat_idx + 32'd1;
                                wb_adr   <= wb_adr + WB_BYTES;
                                wb_cyc   <= 1'b1;
                                wb_stb   <= 1'b1;
                                state    <= S_READ;
                            end
                        end else if (beat_idx == beats_per_cycle - 32'd1) begin
                            if (cycle_idx == num_cycles - 32'd1) begin
                                state <= S_DONE;
                            end else begin
                                cycle_idx <= cycle_idx + 32'd1;
                                beat_idx  <= 32'd0;
                                wb_adr    <= cyc_base;   // wrap to repeated-block base
                                wb_cyc    <= 1'b1;
                                wb_stb    <= 1'b1;
                                state     <= S_READ;
                            end
                        end else begin
                            beat_idx <= beat_idx + 32'd1;
                            wb_adr   <= wb_adr + WB_BYTES;
                            wb_cyc   <= 1'b1;
                            wb_stb   <= 1'b1;
                            state    <= S_READ;
                        end
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
