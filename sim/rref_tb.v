// Whole-system bench for the RREF demo: the real PSC18SR core running
// software/psc18sr_rref/rref.s, driving the real rref_engine over its ports.
// sim/test_rref.py generates the inputs, runs this, and checks the output
// against Python's fractions.Fraction.
//
//   +prog=<hex>    program, from tools/psc18sr_asm.py --hex --words 1024
//   +mat=<hex>     32-bit words: rows, cols, then num,den per element (row-major)
//
// Prints "RANK n", "CYCLES n", "FLAGS ovf err", then one "M r c num den" line
// per element.
`default_nettype none
`timescale 1ns/1ps

module rref_tb;
    localparam integer W = 32;
    reg clk = 0, rst = 1, run = 0;
    always #5 clk = ~clk;

    // ---- PSC18SR: 6 out ports (4 CMD, 5 CTL), 5 in ports (4 = STS) --------
    reg         imem_we = 0;
    reg [15:0]  imem_addr = 0;
    reg [17:0]  imem_data = 0;
    wire        running, halted, aborted;
    wire [15:0] result;
    wire [16*6-1:0] io_out;
    wire [15:0] sts;
    wire [16*5-1:0] io_in = {sts, 64'd0};

    psc18sr #(.NUM_OUT(6), .NUM_IN(5), .DELAY_PRESCALE(4)) cpu (
        .clk(clk), .rst(rst),
        .run(run), .abort(1'b0), .start_pc(10'd256),
        .running(running), .halted(halted), .aborted(aborted), .result(result),
        .imem_we(imem_we), .imem_addr(imem_addr), .imem_data(imem_data),
        .io_out(io_out), .io_in(io_in),
        .wb_cyc(), .wb_stb(), .wb_we(), .wb_adr(), .wb_dat_w(), .wb_sel(),
        .wb_dat_r(32'd0), .wb_ack(1'b0),
        .wdt_fired(), .retire(), .dbg_pc()
    );

    // ---- engine ---------------------------------------------------------
    reg  [3:0]   rows = 0, cols = 0;
    reg  [5:0]   h_addr = 0;
    reg          h_we = 0;
    reg  [W-1:0] h_wn = 0, h_wd = 1;
    wire [W-1:0] h_rn, h_rd;
    wire [31:0]  ops_done;

    rref_engine #(.W(W)) eng (
        .clk(clk), .rst(rst),
        .cmd(io_out[16*4 +: 16]), .go(io_out[16*5]), .sts(sts),
        .rows(rows), .cols(cols), .clr_flags(1'b0),
        .h_addr(h_addr), .h_we(h_we), .h_wn(h_wn), .h_wd(h_wd),
        .h_rn(h_rn), .h_rd(h_rd), .ops_done(ops_done)
    );

    reg [17:0]  prog [0:1023];
    reg [31:0]  mat  [0:2+2*64-1];
    reg [1023:0] progfile, matfile;
    integer k, r, c, cycles;

    initial begin
        if (!$value$plusargs("prog=%s", progfile) || !$value$plusargs("mat=%s", matfile)) begin
            $display("need +prog= and +mat="); $fatal(1);
        end
        $readmemh(progfile, prog);
        $readmemh(matfile, mat);
        rows = mat[0]; cols = mat[1];

        repeat (4) @(posedge clk); rst <= 0; @(posedge clk);

        for (k = 256; k < 1024; k = k + 1) begin          // load program
            imem_addr <= k; imem_data <= prog[k]; imem_we <= 1; @(posedge clk);
        end
        imem_we <= 0;

        for (r = 0; r < rows; r = r + 1)                   // load matrix
            for (c = 0; c < cols; c = c + 1) begin
                h_addr <= r * 8 + c;
                h_wn <= mat[2 + 2 * (r * cols + c)];
                h_wd <= mat[3 + 2 * (r * cols + c)];
                h_we <= 1; @(posedge clk);
            end
        h_we <= 0;

        @(posedge clk); run <= 1; @(posedge clk); run <= 0;
        for (cycles = 0; cycles < 20000000 && !halted; cycles = cycles + 1) @(posedge clk);
        if (!halted) begin $display("TIMEOUT pc=%0d", cpu.pc); $fatal(1); end

        $display("RANK %0d", result);
        $display("CYCLES %0d", cycles);
        $display("OPS %0d", ops_done);
        $display("FLAGS %0d %0d", sts[2], sts[3]);
        for (r = 0; r < rows; r = r + 1)
            for (c = 0; c < cols; c = c + 1) begin
                h_addr <= r * 8 + c; @(posedge clk); @(posedge clk);
                $display("M %0d %0d %0d %0d", r, c, $signed(h_rn), h_rd);
            end
        $finish;
    end
endmodule

`default_nettype wire
