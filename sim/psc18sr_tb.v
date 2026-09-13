// PSC18SR self-checking testbench -- NOT the golden model.
//
// `sim/psc18sr_model.py` is deliberately still absent: the ISA is a draft, and
// a model written against a moving encoding would only be rewritten. This is
// the smaller thing that is useful before the freeze -- it runs an assembled
// program on the RTL and compares `result` against what the program's comments
// say it should be. It proves instructions EXECUTE; it proves nothing about
// the encoding being right, which is the model's job.
//
//   python3 tools/psc18sr_asm.py software/psc18sr_test/shift.s --hex /tmp/p.hex \
//           --words 1024
//   iverilog -g2012 -o /tmp/tb.vvp sim/psc18sr_tb.v verilog/psc18sr.v
//   vvp /tmp/tb.vvp +hex=/tmp/p.hex +expect=0x5a
//
// `+in=<word>` with `+in_after=<cycles>` stands in for the host poking an
// input port while the core runs -- what seq.s parks in WAIT for:
//
//   vvp /tmp/tb.vvp +hex=/tmp/seq.hex +expect=0xa5 +in=1 +in_after=2000
//
// The program is loaded through the HOST write port at `start` (default 256 =
// the default ro_words), which is the post-boot path -- the same one the CPU
// uses.  +autostart=1 instead loads it as the bitstream-resident image and
// enters at pc=0, which is the stage-0 path; the program must then be
// assembled with `.org 0`.
`default_nettype none
`timescale 1ns/1ps

module psc18sr_tb;
    localparam integer IMEM_WORDS = 1024;
    localparam integer RO_WORDS   = 256;
    localparam integer NUM_OUT    = 4;
    localparam integer NUM_IN     = 4;
    localparam integer TIMEOUT    = 200000;

    reg clk = 0, rst = 1, run = 0;
    always #5 clk = ~clk;

    reg [17:0] img [0:IMEM_WORDS-1];
    reg [1023:0] hexfile;
    integer expect_val, start, autostart, words, k;
    integer in_val, in_after;

    reg         imem_we   = 0;
    reg [15:0]  imem_addr = 0;
    reg [17:0]  imem_data = 0;
    reg [16*NUM_IN-1:0] io_in = 0;

    wire running, halted, aborted, wdt_fired;
    wire [15:0] result;
    wire [16*NUM_OUT-1:0] io_out;

    // Held-off Wishbone: BUS accesses never ack, so a program using BUS needs
    // a real SoC, not this bench.  seq.s/sum.s/shift.s do not.
    wire wb_cyc, wb_stb, wb_we;
    wire [29:0] wb_adr;
    wire [31:0] wb_dat_w;
    wire [3:0]  wb_sel;

    psc18sr #(
        .IMEM_WORDS(IMEM_WORDS), .RO_WORDS(RO_WORDS),
        .NUM_OUT(NUM_OUT), .NUM_IN(NUM_IN), .DELAY_PRESCALE(4)
    ) dut (
        .clk(clk), .rst(rst),
        .run(run), .abort(1'b0), .start_pc(start[$clog2(IMEM_WORDS)-1:0]),
        .running(running), .halted(halted), .aborted(aborted), .result(result),
        .imem_we(imem_we), .imem_addr(imem_addr), .imem_data(imem_data),
        .io_out(io_out), .io_in(io_in),
        .wb_cyc(wb_cyc), .wb_stb(wb_stb), .wb_we(wb_we), .wb_adr(wb_adr),
        .wb_dat_w(wb_dat_w), .wb_sel(wb_sel),
        .wb_dat_r(32'd0), .wb_ack(1'b0),
        .wdt_fired(wdt_fired), .retire(), .dbg_pc()
    );

    initial begin
        if (!$value$plusargs("hex=%s", hexfile)) begin
            $display("FAIL: need +hex=<file> from tools/psc18sr_asm.py --hex");
            $fatal(1);
        end
        if (!$value$plusargs("expect=%d", expect_val)) expect_val = -1;
        if (!$value$plusargs("start=%d", start))       start     = RO_WORDS;
        if (!$value$plusargs("autostart=%d", autostart)) autostart = 0;
        if (!$value$plusargs("words=%d", words))       words     = IMEM_WORDS;
        if (!$value$plusargs("in=%d", in_val))         in_val    = 0;
        if (!$value$plusargs("in_after=%d", in_after)) in_after  = 0;

        for (k = 0; k < IMEM_WORDS; k = k + 1) img[k] = 18'd0;
        $readmemh(hexfile, img);

        if (autostart != 0) begin
            // Stage-0 path: the image IS the bitstream-resident region, so it
            // goes straight into the memory array rather than through the host
            // port, which refuses writes below ro_words.
            for (k = 0; k < words; k = k + 1) dut.mem[k] = img[k];
            repeat (4) @(posedge clk);
            rst <= 0;
        end else begin
            repeat (4) @(posedge clk);
            rst <= 0;
            @(posedge clk);
            for (k = start; k < words; k = k + 1) begin
                imem_addr <= k[15:0];
                imem_data <= img[k];
                imem_we   <= 1'b1;
                @(posedge clk);
            end
            imem_we <= 1'b0;
            @(posedge clk);
            run <= 1'b1; @(posedge clk); run <= 1'b0;
        end

        for (k = 0; k < TIMEOUT && !halted; k = k + 1) begin
            @(posedge clk);
            if (in_val != 0 && k == in_after) io_in <= in_val[16*NUM_IN-1:0];
        end

        if (!halted) begin
            $display("FAIL: no HLT within %0d cycles (pc=%0d)", TIMEOUT, dut.pc);
            $fatal(1);
        end
        $display("halted after %0d cycles: result=0x%04x (%0d)%s",
                 k, result, result, aborted ? " ABORTED" : "");
        if (expect_val >= 0 && result !== expect_val[15:0]) begin
            $display("FAIL: expected 0x%04x", expect_val[15:0]);
            $fatal(1);
        end
        $display("PASS");
        $finish;
    end
endmodule

`default_nettype wire
