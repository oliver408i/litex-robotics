// ring_monitor.v -- tiny self-contained on-die frequency/PVT monitor.
//
// Drop into any SoC: wire clk/rstn/enable, then read `count` whenever you like.
// A ring oscillator's speed tracks voltage/temperature/process, so a drop in
// `count` means the die got hotter (or the supply sagged). Reconstruct the
// frequency however/whenever you want, off the hot path:
//
//   f_ring ~= count * (2**PRESCALE_BITS) * f_clk / GATE_CYCLES
//
// Cost: ~5 LUTs + PRESCALE_BITS FFs in the ring/osc domain, plus a handful of
// FFs in the clk domain. CPU cost is zero -- the measurement runs entirely in
// hardware and software just reads `count` (and optionally `seq`) at its
// leisure. No interrupts, no polling loop required.
//
// Clock domains: exactly TWO -- your `clk`, and the internal free-running
// `osc`. The prescaler is a *synchronous* counter (not a ripple chain), so it
// adds a single clock domain rather than one per divider bit, and the clk side
// samples only the slow prescaler MSB as ordinary data (no divided clock net).
//
// The ring is an intentional combinational loop, so PnR must be told to allow
// it (nextpnr-ecp5: --ignore-loops). The LUT4 primitives stop yosys collapsing
// the inverter chain into one LUT. Porting to another FPGA: swap the two LUT4
// instances for that family's LUT primitive (or `(* keep *)` ~ inverters) and
// keep the rest as-is.
//
// PRESCALE_BITS sets the divider (2**PRESCALE_BITS). The prescaler MSB toggles
// at f_osc / 2**PRESCALE_BITS; keep that well below f_clk/4 so the clk-domain
// edge detector never misses a transition (default /256 puts a ~300 MHz ring
// at ~1.2 MHz). Want finer resolution? Widen GATE_CYCLES, don't shrink the
// divider.
module ring_monitor #(
  parameter integer N_STAGES      = 5,      // odd number of inverter stages
  parameter integer PRESCALE_BITS = 8,      // divider = 2**PRESCALE_BITS
  parameter integer GATE_CYCLES   = 500000  // measurement window, in clk cycles
)(
  input  wire        clk,
  input  wire        rstn,                  // active-low, synchronous to clk
  input  wire        enable,                // gate the ring (0 => zero dynamic power)
  output reg  [31:0] count = 32'd0,         // prescaled ring edges in the last window
  output reg         valid = 1'b0,          // high once the first window has completed
  output reg  [7:0]  seq   = 8'd0           // bumps every time `count` updates
);

  // ---- ring oscillator (osc domain) ----------------------------------------
  reg run = 1'b0;
  (* keep *) wire [N_STAGES-1:0] stage;
  // NAND gate on the first stage: the ring only oscillates while run = 1.
  (* keep *) LUT4 #(.INIT(16'h7777)) s0 (
    .A(stage[N_STAGES-1]), .B(run), .C(1'b0), .D(1'b0), .Z(stage[0]));
  genvar i;
  generate
    for (i = 1; i < N_STAGES; i = i + 1) begin : inv
      (* keep *) LUT4 #(.INIT(16'h5555)) g (
        .A(stage[i-1]), .B(1'b0), .C(1'b0), .D(1'b0), .Z(stage[i]));
    end
  endgenerate
  wire osc = stage[N_STAGES-1];

  // ---- synchronous prescaler (osc domain, the ONLY extra clock domain) ------
  // A synchronous up-counter, not a ripple chain, so PnR sees one clock (osc).
  // Keep PRESCALE_BITS small enough that this counter meets timing at the real
  // (unconstrained) ring frequency -- 8 bits clears a few hundred MHz easily.
  reg [PRESCALE_BITS-1:0] presc = {PRESCALE_BITS{1'b0}};
  always @(posedge osc) presc <= presc + 1'b1;
  wire presc_msb = presc[PRESCALE_BITS-1];   // toggles at f_osc / 2**PRESCALE_BITS

  // ---- clk domain: sample the slow MSB, count its edges over a window -------
  reg [2:0] msb_sync = 3'd0;                 // [0] = metastability capture
  always @(posedge clk) msb_sync <= {msb_sync[1:0], presc_msb};
  wire msb_rise = msb_sync[1] & ~msb_sync[2];

  reg [31:0] acc  = 32'd0;
  reg [31:0] gate = 32'd0;
  reg        busy = 1'b0;

  always @(posedge clk) begin
    if (!rstn) begin
      run <= 1'b0; busy <= 1'b0; acc <= 32'd0; gate <= 32'd0;
      count <= 32'd0; valid <= 1'b0; seq <= 8'd0;
    end else if (!enable) begin
      run <= 1'b0; busy <= 1'b0; acc <= 32'd0; gate <= 32'd0;
    end else begin
      run <= 1'b1;                           // ring free-runs while enabled
      if (!busy) begin
        acc  <= 32'd0;
        gate <= GATE_CYCLES;
        busy <= 1'b1;
      end else if (gate == 32'd0) begin
        count <= acc;                        // publish the finished window
        valid <= 1'b1;
        seq   <= seq + 8'd1;
        busy  <= 1'b0;                        // a fresh window starts next cycle
      end else begin
        if (msb_rise) acc <= acc + 32'd1;
        gate <= gate - 32'd1;
      end
    end
  end
endmodule
