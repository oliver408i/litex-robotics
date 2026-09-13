// PSC18SR -- PISC v2 sequencer core.  PROTOTYPE, against a DRAFT ISA.
//
// 18-bit instructions / 16-bit data.  See docs/psc18sr_isa_draft.md, which is
// NOT frozen -- unlike verilog/pisc.v, this module has no golden model to be
// diffed against yet, so it is the only executable statement of the encoding
// and it is expected to change.  Do not treat it as a contract.
//
// Two entry paths (draft "Autostart, HLT, and restart"):
//   - AUTOSTART=1: begins at pc=0 out of reset, no host involvement.  This is
//     the stage-0 boot role, where the core holds the CPU in reset.
//   - AUTOSTART=0: v1's peripheral model -- host loads imem, writes start_pc,
//     pulses run, polls halted.  This is what the bring-up top uses, so a
//     broken program cannot wedge the SoC that is debugging it.
//
// imem is one unified 18-bit memory.  Words [0, RO_WORDS) are loaded from the
// bitstream (INIT_FILE -> INITVAL) and are NOT host-writable: autostart always
// enters at pc=0, so a host-loaded program must never be able to land there.
// Words [RO_WORDS, IMEM_WORDS) take host writes exactly as v1 does.
//
// Microarchitecture: multicycle FSM, same shape as pisc.v.
//   FETCH -> DECODE -> EXEC -> {LOAD | DLY | BUSRD -> BUSWR} -> FETCH
//
// Timing invariant the whole design serves: every instruction retires in
// exactly 3 cycles, branches taken AND not-taken included, because they share
// the EXEC path.  LD is the lone 4-cycle outlier; BUS, WAIT and DELAY are
// variable by design.  Anything added to EXEC must resolve combinationally or
// it costs more than it looks -- the shifts are the worked example.
//
// BUS is read-modify-write on a half-word store (see S_BUSWR): LiteX's
// Wishbone2CSR ignores wb_sel, so a 16-bit write must read the 32-bit word
// back first.  PROTOTYPE CAVEAT: this is wrong for any CSR with a read side
// effect.  Every CSR stage-0 touches today is a plain CSRStorage, which reads
// back what it holds, so it is safe there and nowhere promised beyond that.
`default_nettype none

module psc18sr #(
    parameter integer IMEM_WORDS     = 1024, // unified program/data words (pow2)
    parameter integer RO_WORDS       = 256,  // low words: bitstream-resident, RO
    parameter integer NUM_OUT        = 4,
    parameter integer NUM_IN         = 4,
    parameter integer DELAY_PRESCALE = 1024, // core cycles per DELAY tick (>=1)
    parameter integer AUTOSTART      = 0,    // 1 = run at pc=0 out of reset
    parameter [31:0]  BUS_BASE       = 32'hf0000000, // 64K window base (byte addr)
    parameter integer WDT_CYCLES     = 0,    // 0 = watchdog disabled
    parameter integer CPU_RST_PORT   = 0,    // which out port holds cpu_rst_n
    parameter integer CPU_RST_BIT    = 0,
    parameter         INIT_FILE      = "",   // $readmemh source for the RO region
    // Derived; do not override.
    parameter integer ADDR_BITS      = $clog2(IMEM_WORDS)
)(
    input  wire                        clk,
    input  wire                        rst,

    // Host control (AUTOSTART=0 path, and abort in both).
    input  wire                        run,      // pulse: start at start_pc
    input  wire                        abort,    // pulse: stop now, go idle
    input  wire [ADDR_BITS-1:0]        start_pc,
    output reg                         running,
    output reg                         halted,
    output reg                         aborted,
    output reg  [15:0]                 result,   // r7 latched at HLT

    // Host instruction-memory write port.  Honored only while !running AND
    // addr >= RO_WORDS.
    input  wire                        imem_we,
    input  wire [15:0]                 imem_addr,
    input  wire [17:0]                 imem_data,

    // I/O ports, packed: port p occupies bits [16*p +: 16].
    output reg  [16*NUM_OUT-1:0]       io_out,
    input  wire [16*NUM_IN-1:0]        io_in,

    // Wishbone master for BUS (classic, 32-bit, word-addressed).
    output reg                         wb_cyc,
    output reg                         wb_stb,
    output reg                         wb_we,
    output wire [29:0]                 wb_adr,
    output reg  [31:0]                 wb_dat_w,
    output wire [3:0]                  wb_sel,
    input  wire [31:0]                 wb_dat_r,
    input  wire                        wb_ack,

    // Debug hooks.
    output reg                         wdt_fired,
    output reg                         retire,
    output wire [ADDR_BITS-1:0]        dbg_pc
);
    localparam [3:0] S_IDLE  = 4'd0, S_FETCH = 4'd1, S_DECODE = 4'd2,
                     S_EXEC  = 4'd3, S_LOAD  = 4'd4, S_DLY    = 4'd5,
                     S_BUSRD = 4'd6, S_BUSWR = 4'd7;

    // Opcodes (draft "Encoding").
    localparam [3:0] OP_JMP=4'h0, OP_ALU=4'h1, OP_ADDI=4'h2, OP_LI  =4'h3,
                     OP_LD =4'h4, OP_ST =4'h5, OP_BEQ =4'h6, OP_BNE =4'h7,
                     OP_JAL=4'h8, OP_JALR=4'h9,OP_PORT=4'hA, OP_BITOP=4'hB,
                     OP_WAIT=4'hC,OP_BUS=4'hD, OP_DELAY=4'hE,OP_HLT =4'hF;

    // ALU funct (draft: 0x08-0x1F reserved, deliberately unassigned).
    localparam [4:0] F_ADD=5'h00, F_SUB=5'h01, F_AND=5'h02, F_OR=5'h03,
                     F_XOR=5'h04, F_SLL=5'h05, F_SRL=5'h06, F_SRA=5'h07;

    localparam integer PSCW = (DELAY_PRESCALE <= 1) ? 1 : $clog2(DELAY_PRESCALE+1);

    reg [3:0]           state;
    reg [ADDR_BITS-1:0] pc;
    reg [17:0]          ir;
    reg [15:0]          regs [0:7];
    reg [13:0]          dcount;
    reg [PSCW-1:0]      psc;
    reg [31:0]          wdt;
    reg                 wdt_armed;
    reg [15:0]          bus_addr;   // byte offset within the window
    reg [15:0]          bus_wdata;  // half-word to merge on a BUS write

    assign dbg_pc = pc;

    // --- unified 18-bit memory -------------------------------------------
    reg [17:0] mem [0:IMEM_WORDS-1];
    reg [17:0] mem_rdata;
    integer j;
    initial begin
        for (j = 0; j < IMEM_WORDS; j = j + 1) mem[j] = 18'd0;
        // 18'h00000 decodes as JMP +0 -- a blank imem parks the core rather
        // than running garbage.  Matters more here than in v1: this core can
        // hold the CPU reset line.
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
        mem_rdata = 18'd0;
    end
    reg                 mem_we;
    reg [ADDR_BITS-1:0] mem_waddr;
    reg [17:0]          mem_wdata;
    reg [ADDR_BITS-1:0] mem_raddr;

    // --- field decode -----------------------------------------------------
    wire [3:0] op    = ir[17:14];
    wire [2:0] rd    = ir[13:11];
    wire [2:0] rs    = ir[10:8];
    wire [2:0] rs2   = ir[7:5];
    wire [4:0] funct = ir[4:0];
    wire [5:0] port  = ir[5:0];
    wire [3:0] bsel  = ir[11:8];   // 4 bits: reaches all 16 port bits (draft Q10)
    wire       lvl   = ir[7];
    wire       bdir  = ir[7];      // PORT/BUS direction: 1 = write/out
    wire       bhalf = ir[6];      // BUS half: 1 = upper 16 bits

    wire signed [15:0] imm6  = {{10{ir[5]}},  ir[5:0]};
    wire signed [15:0] imm8  = {{8{ir[7]}},   ir[7:0]};
    wire signed [15:0] imm11 = {{5{ir[10]}},  ir[10:0]};
    wire signed [15:0] imm14 = {{2{ir[13]}},  ir[13:0]};

    wire [15:0] a  = regs[rs];
    wire [15:0] b  = regs[rs2];
    wire [15:0] d  = regs[rd];
    wire [ADDR_BITS-1:0] mem_addr = (a + imm8);   // truncate = wrap (pow2)

    wire [5:0] in_idx   = (port < NUM_IN[5:0])  ? port : 6'd0;
    wire [15:0] in_word = io_in[16*in_idx +: 16];
    wire        wait_done = (in_word[bsel] == lvl);

    // SLL/SRL/SRA take their amount from regs[rs2][3:0] -- MASKED, not
    // saturating, so a shift is total: every amount 0-15 is defined and 16+
    // wraps rather than trapping.  Barrel, combinational, resolved within
    // S_EXEC like every other ALU op, so a shift costs 3 cycles like every
    // other ALU op.  An iterative shifter would be smaller and would make the
    // cycle count data-dependent, which breaks the property the core exists
    // for -- see the draft's "The shifts" before replacing it with one.
    reg [15:0] alu;
    always @* begin
        case (op)
            OP_ALU: case (funct)
                        F_SUB:   alu = a - b;
                        F_AND:   alu = a & b;
                        F_OR:    alu = a | b;
                        F_XOR:   alu = a ^ b;
                        F_SLL:   alu = a << b[3:0];
                        F_SRL:   alu = a >> b[3:0];
                        F_SRA:   alu = $signed(a) >>> b[3:0];
                        default: alu = a + b;      // F_ADD; reserved -> ADD
                    endcase
            OP_ADDI: alu = a + imm8;
            OP_LI:   alu = imm11;
            OP_PORT: alu = (port < NUM_IN[5:0]) ? in_word : 16'd0;
            default: alu = 16'd0;
        endcase
    end

    // --- memory ports -----------------------------------------------------
    always @* begin
        mem_raddr = pc;
        if (state == S_EXEC && op == OP_LD) mem_raddr = mem_addr;
    end

    wire host_write_ok = imem_we && (imem_addr >= RO_WORDS[15:0])
                                 && (imem_addr <  IMEM_WORDS[15:0]);

    always @* begin
        if (running) begin
            mem_we    = (state == S_EXEC && op == OP_ST);
            mem_waddr = mem_addr;
            mem_wdata = {2'b00, d};        // data lanes only; top two stay 0
        end else begin
            mem_we    = host_write_ok;
            mem_waddr = imem_addr[ADDR_BITS-1:0];
            mem_wdata = imem_data;
        end
    end

    always @(posedge clk) begin
        if (mem_we) mem[mem_waddr] <= mem_wdata;
        mem_rdata <= mem[mem_raddr];
    end

    // --- Wishbone address -------------------------------------------------
    assign wb_adr = BUS_BASE[31:2] + {14'd0, bus_addr[15:2]};
    assign wb_sel = 4'hF;

    // --- watchdog ---------------------------------------------------------
    // One invariant: the CPU is released within WDT_CYCLES of reset, whatever
    // the program does.  Disarms permanently once released, so post-boot
    // programs (which may run indefinitely) are never watched.
    wire cpu_released = io_out[16*CPU_RST_PORT + CPU_RST_BIT];

    integer i;
    task do_reset;
        begin
            state     <= (AUTOSTART != 0) ? S_FETCH : S_IDLE;
            pc        <= {ADDR_BITS{1'b0}};
            running   <= (AUTOSTART != 0);
            halted    <= 1'b0;
            aborted   <= 1'b0;
            result    <= 16'd0;
            io_out    <= {16*NUM_OUT{1'b0}};
            retire    <= 1'b0;
            wb_cyc    <= 1'b0;
            wb_stb    <= 1'b0;
            wb_we     <= 1'b0;
            wdt       <= 32'd0;
            wdt_armed <= (WDT_CYCLES != 0);
            wdt_fired <= 1'b0;
            for (i = 0; i < 8; i = i + 1) regs[i] <= 16'd0;
        end
    endtask

    always @(posedge clk) begin
        retire <= 1'b0;

        if (rst) begin
            do_reset;
        end else begin
            // ---- watchdog ------------------------------------------------
            if (wdt_armed) begin
                if (cpu_released) begin
                    wdt_armed <= 1'b0;             // job done, permanently
                end else if (wdt >= WDT_CYCLES[31:0]) begin
                    io_out[16*CPU_RST_PORT + CPU_RST_BIT] <= 1'b1;
                    result    <= 16'hFFFF;
                    halted    <= 1'b1;
                    running   <= 1'b0;
                    wdt_fired <= 1'b1;
                    wdt_armed <= 1'b0;
                    wb_cyc    <= 1'b0;
                    wb_stb    <= 1'b0;
                    state     <= S_IDLE;
                end else begin
                    wdt <= wdt + 1'b1;
                end
            end

            if (abort) begin
                running <= 1'b0;
                halted  <= 1'b1;
                aborted <= 1'b1;
                wb_cyc  <= 1'b0;
                wb_stb  <= 1'b0;
                state   <= S_IDLE;
            end else case (state)
                S_IDLE: begin
                    if (run) begin
                        running <= 1'b1;
                        halted  <= 1'b0;
                        aborted <= 1'b0;
                        pc      <= start_pc;
                        for (i = 0; i < 8; i = i + 1) regs[i] <= 16'd0;
                        state   <= S_FETCH;
                    end
                end

                S_FETCH:  state <= S_DECODE;
                S_DECODE: begin ir <= mem_rdata; state <= S_EXEC; end

                S_EXEC: begin
                    case (op)
                        OP_ALU, OP_ADDI, OP_LI: begin
                            if (rd != 3'd0) regs[rd] <= alu;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_LD: state <= S_LOAD;
                        OP_ST: begin
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_JMP: begin
                            pc <= pc + imm14[ADDR_BITS-1:0];
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_JAL: begin
                            regs[6] <= {{(16-ADDR_BITS){1'b0}}, pc} + 16'd1;
                            pc <= pc + imm14[ADDR_BITS-1:0];
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_JALR: begin
                            regs[6] <= {{(16-ADDR_BITS){1'b0}}, pc} + 16'd1;
                            pc <= a[ADDR_BITS-1:0] + imm8[ADDR_BITS-1:0];
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_BEQ: begin
                            if (d == a) pc <= pc + imm8[ADDR_BITS-1:0];
                            else        pc <= pc + 1'b1;
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_BNE: begin
                            if (d != a) pc <= pc + imm8[ADDR_BITS-1:0];
                            else        pc <= pc + 1'b1;
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_PORT: begin
                            // dir=0: IN  -> rd = io_in[port]
                            // dir=1: OUT -> io_out[port] = rs
                            if (bdir) begin
                                if (port < NUM_OUT[5:0]) io_out[16*port +: 16] <= a;
                            end else begin
                                if (rd != 3'd0) regs[rd] <= alu;
                            end
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_BITOP: begin
                            if (port < NUM_OUT[5:0])
                                io_out[16*port + bsel] <= lvl;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_WAIT: begin
                            // Unbounded by design; the watchdog is what bounds
                            // it during boot, and `abort` afterwards.
                            if (wait_done) begin
                                pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                            end
                        end
                        OP_BUS: begin
                            bus_addr  <= a + imm6;
                            bus_wdata <= d;
                            wb_cyc    <= 1'b1;
                            wb_stb    <= 1'b1;
                            wb_we     <= 1'b0;      // read first, always (RMW)
                            state     <= S_BUSRD;
                        end
                        OP_DELAY: begin
                            if (ir[13:0] == 14'd0) begin
                                pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                            end else begin
                                dcount <= ir[13:0];
                                psc    <= DELAY_PRESCALE[PSCW-1:0];
                                state  <= S_DLY;
                            end
                        end
                        OP_HLT: begin
                            result  <= regs[7];
                            halted  <= 1'b1;
                            running <= 1'b0;
                            retire  <= 1'b1;
                            state   <= S_IDLE;      // restartable: see draft
                        end
                        default: begin
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                    endcase
                end

                S_LOAD: begin
                    if (rd != 3'd0) regs[rd] <= mem_rdata[15:0];
                    pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                end

                // ---- BUS read phase (also the R of a write's RMW) ---------
                S_BUSRD: if (wb_ack) begin
                    if (bdir) begin
                        wb_dat_w <= bhalf ? {bus_wdata, wb_dat_r[15:0]}
                                          : {wb_dat_r[31:16], bus_wdata};
                        wb_we    <= 1'b1;
                        state    <= S_BUSWR;        // cyc/stb stay asserted
                    end else begin
                        if (rd != 3'd0)
                            regs[rd] <= bhalf ? wb_dat_r[31:16] : wb_dat_r[15:0];
                        wb_cyc <= 1'b0; wb_stb <= 1'b0;
                        pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                    end
                end

                S_BUSWR: if (wb_ack) begin
                    wb_cyc <= 1'b0; wb_stb <= 1'b0; wb_we <= 1'b0;
                    pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                end

                S_DLY: begin
                    if (psc <= 1) begin
                        psc <= DELAY_PRESCALE[PSCW-1:0];
                        if (dcount <= 14'd1) begin
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end else begin
                            dcount <= dcount - 1'b1;
                        end
                    end else begin
                        psc <= psc - 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
