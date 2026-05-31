// On-die heater: a bank of enable-gated ring oscillators whose only job is to
// burn dynamic power and warm the die, so you can watch a monitored ring (see
// ring_osc_meas.v) slow down without any external heat source.
//
// Each ring is the same NAND-gated odd LUT4 chain as the measurement ring, so
// `enable`=0 leaves every ring static (zero dynamic power). The last node of
// every ring is XOR-reduced into `alive`, which MUST be driven somewhere by the
// instantiator -- it gives the rings a real fanout so yosys/nextpnr can't prune
// the whole bank away as dead logic. (It also makes a fine noise source.)
//
// Like ring_osc_meas, this is an intentional combinational loop: build with
// --nextpnr-ignoreloops.
module ring_heater #(
  parameter integer N_RINGS  = 32,   // scale this UP gradually while watching the monitor
  parameter integer N_STAGES = 5     // odd
)(
  input  wire enable,
  output wire alive
);
  wire [N_RINGS-1:0] tap;
  genvar r, k;
  generate
    for (r = 0; r < N_RINGS; r = r + 1) begin : ring
      (* keep *) wire [N_STAGES-1:0] s;
      // NAND enable on the first stage: ring runs only while `enable` is high.
      (* keep *) LUT4 #(.INIT(16'h7777)) s0 (
        .A(s[N_STAGES-1]), .B(enable), .C(1'b0), .D(1'b0), .Z(s[0])
      );
      for (k = 1; k < N_STAGES; k = k + 1) begin : inv
        (* keep *) LUT4 #(.INIT(16'h5555)) g (
          .A(s[k-1]), .B(1'b0), .C(1'b0), .D(1'b0), .Z(s[k])
        );
      end
      assign tap[r] = s[N_STAGES-1];
    end
  endgenerate

  assign alive = ^tap;   // real sink so the bank survives synthesis
endmodule
