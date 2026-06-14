// PISC -- Programmable I/O Sequencer Core.
//
// A tiny 16-bit multicycle soft CPU used as a host-loaded peripheral for
// deterministic I/O choreography (reset/enable ordering, pulse timing,
// wait-for-pin). NOT a general compute core -- see docs/pisc_isa.md for the
// frozen ISA contract this module must obey. The golden reference is
// sim/pisc_model.py; verilog/test_pisc.py diffs this RTL against it.
//
// Microarchitecture: classic multicycle FSM over a single 1R1W memory that
// holds both program and data (maps to one ECP5 DP16KD). One instruction at a
// time, run to HLT.
//
//   FETCH  : issue read of mem[pc]
//   DECODE : capture instruction word (1-cycle BRAM read latency)
//   EXEC   : ALU / branch / io / start LD or DELAY
//   LOAD   : capture LD result into rd
//   DLY    : countdown for DELAY
//
// WAIT spins in EXEC until the input pin matches. `retire` pulses for one cycle
// as each instruction commits (debug / differential-test hook); `dbg_pc` then
// holds the next PC to be fetched.
`default_nettype none

module pisc #(
    parameter integer IMEM_WORDS     = 256,  // unified program/data words (power of 2)
    parameter integer NUM_OUT        = 8,    // number of 16-bit output ports
    parameter integer NUM_IN         = 8,    // number of 16-bit input ports
    parameter integer DELAY_PRESCALE = 1,    // core cycles per DELAY tick (>=1)
    // Derived; do not override. Declared here so it is visible to the port list.
    parameter integer ADDR_BITS      = $clog2(IMEM_WORDS)
)(
    input  wire                        clk,
    input  wire                        rst,

    // Host control.
    input  wire                        run,     // pulse: start executing at pc=0
    input  wire                        clr,     // pulse: clear arch state, go idle
    output reg                         running,
    output reg                         halted,
    output reg  [15:0]                 result,  // r7 latched at HLT

    // Host instruction-memory write port (honored only while !running).
    input  wire                        imem_we,
    input  wire [15:0]                 imem_addr,
    input  wire [15:0]                 imem_data,

    // I/O ports, packed: port p occupies bits [16*p +: 16].
    output reg  [16*NUM_OUT-1:0]       io_out,
    input  wire [16*NUM_IN-1:0]        io_in,

    // Debug / differential-test hooks.
    output reg                         retire,  // 1-cycle pulse as an instr commits
    output wire [ADDR_BITS-1:0]        dbg_pc
);
    // FSM states.
    localparam [2:0] S_IDLE   = 3'd0,
                     S_FETCH  = 3'd1,
                     S_DECODE = 3'd2,
                     S_EXEC   = 3'd3,
                     S_LOAD   = 3'd4,
                     S_DLY    = 3'd5;

    // Opcodes (see docs/pisc_isa.md).
    localparam [3:0] OP_JMP=4'h0, OP_ADD=4'h1, OP_SUB=4'h2, OP_ADDI=4'h3,
                     OP_AND=4'h4, OP_OR=4'h5, OP_LD=4'h6, OP_ST=4'h7,
                     OP_BNE=4'h8, OP_OUT=4'h9, OP_IN=4'hA, OP_SETB=4'hB,
                     OP_CLRB=4'hC, OP_WAIT=4'hD, OP_DELAY=4'hE, OP_HLT=4'hF;

    reg [2:0]          state;
    reg [ADDR_BITS-1:0] pc;
    reg [15:0]         ir;
    reg [15:0]         regs [0:7];
    reg [11:0]         dcount;            // DELAY tick counter
    reg [$clog2(DELAY_PRESCALE+1)-1:0] psc; // DELAY prescale sub-counter

    assign dbg_pc = pc;

    // --- unified 1R1W memory (program + data) ---------------------------
    reg [15:0] mem [0:IMEM_WORDS-1];
    reg [15:0] mem_rdata;
    integer j;
    initial begin                            // BRAM powers up zeroed on ECP5
        for (j = 0; j < IMEM_WORDS; j = j + 1) mem[j] = 16'd0;
        mem_rdata = 16'd0;
    end
    reg                  mem_we;
    reg [ADDR_BITS-1:0]  mem_waddr;
    reg [15:0]           mem_wdata;
    reg [ADDR_BITS-1:0]  mem_raddr;

    // --- instruction field decode (combinational on ir) -----------------
    wire [3:0] op   = ir[15:12];
    wire [2:0] rd   = ir[11:9];
    wire [2:0] rs   = ir[8:6];
    wire [2:0] rs2  = ir[5:3];
    wire [5:0] port = ir[5:0];
    wire [2:0] bsel = ir[11:9];          // bit index for SETB/CLRB/WAIT
    wire       wlvl = ir[8];             // wait level
    wire signed [15:0] imm6  = {{10{ir[5]}},  ir[5:0]};
    wire signed [15:0] imm12 = {{4{ir[11]}},  ir[11:0]};

    wire [15:0] a = regs[rs];
    wire [15:0] b = regs[rs2];
    wire [ADDR_BITS-1:0] mem_addr = (a + imm6);   // truncates = wrap (pow2 size)

    // Selected input word for IN / WAIT (clamp index into range).
    wire [5:0] in_idx = (port < NUM_IN[5:0]) ? port : 6'd0;
    wire [15:0] in_word = io_in[16*in_idx +: 16];
    wire        wait_done = (in_word[bsel] == wlvl);

    // ALU result for EXEC.
    reg [15:0] alu;
    always @* begin
        case (op)
            OP_ADD:  alu = a + b;
            OP_SUB:  alu = a - b;
            OP_AND:  alu = a & b;
            OP_OR:   alu = a | b;
            OP_ADDI: alu = a + imm6;
            OP_LD:   alu = mem_rdata;          // (committed in S_LOAD)
            OP_IN:   alu = (port < NUM_IN[5:0]) ? in_word : 16'd0;
            default: alu = 16'd0;
        endcase
    end

    // Memory read address: fetch from pc, except when EXEC starts a load.
    always @* begin
        mem_raddr = pc;
        if (state == S_EXEC && op == OP_LD)
            mem_raddr = mem_addr;
    end

    // Memory write port: host while idle, ST while running.
    always @* begin
        if (running) begin
            mem_we    = (state == S_EXEC && op == OP_ST);
            mem_waddr = mem_addr;
            mem_wdata = regs[rd];              // rd field is the store data reg
        end else begin
            mem_we    = imem_we;
            mem_waddr = imem_addr[ADDR_BITS-1:0];
            mem_wdata = imem_data;
        end
    end

    // Synchronous 1R1W memory.
    always @(posedge clk) begin
        if (mem_we)
            mem[mem_waddr] <= mem_wdata;
        mem_rdata <= mem[mem_raddr];
    end

    // --- helper: clear architectural state ------------------------------
    integer i;
    task do_clear;
        begin
            state    <= S_IDLE;
            pc       <= {ADDR_BITS{1'b0}};
            running  <= 1'b0;
            halted   <= 1'b0;
            result   <= 16'd0;
            io_out   <= {16*NUM_OUT{1'b0}};
            retire   <= 1'b0;
            for (i = 0; i < 8; i = i + 1) regs[i] <= 16'd0;
        end
    endtask

    // --- main FSM -------------------------------------------------------
    always @(posedge clk) begin
        retire <= 1'b0;                        // default; pulsed on commit

        if (rst) begin
            do_clear;
        end else if (clr) begin
            do_clear;
        end else begin
            case (state)
                // ---- idle: accept program loads, wait for run ----------
                S_IDLE: begin
                    if (run) begin
                        running <= 1'b1;
                        halted  <= 1'b0;
                        pc      <= {ADDR_BITS{1'b0}};
                        for (i = 0; i < 8; i = i + 1) regs[i] <= 16'd0;
                        state   <= S_FETCH;
                    end
                end

                // ---- fetch / decode (BRAM read latency) ----------------
                S_FETCH:  state <= S_DECODE;            // mem_raddr=pc this cycle
                S_DECODE: begin ir <= mem_rdata; state <= S_EXEC; end

                // ---- execute -------------------------------------------
                S_EXEC: begin
                    // Default: commit reg write (if any), advance, refetch.
                    case (op)
                        OP_ADD, OP_SUB, OP_AND, OP_OR, OP_ADDI: begin
                            if (rd != 3'd0) regs[rd] <= alu;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_IN: begin
                            if (rd != 3'd0) regs[rd] <= alu;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_LD: begin                    // result captured next cycle
                            state <= S_LOAD;
                        end
                        OP_ST: begin                    // write issued via mem_we
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_JMP: begin
                            pc <= pc + imm12[ADDR_BITS-1:0];
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_BNE: begin
                            if (regs[rs] != regs[rd]) pc <= pc + imm6[ADDR_BITS-1:0];
                            else                      pc <= pc + 1'b1;
                            retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_OUT: begin
                            if (port < NUM_OUT[5:0]) io_out[16*port +: 16] <= a;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_SETB: begin
                            if (port < NUM_OUT[5:0]) io_out[16*port + bsel] <= 1'b1;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_CLRB: begin
                            if (port < NUM_OUT[5:0]) io_out[16*port + bsel] <= 1'b0;
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                        OP_WAIT: begin
                            if (wait_done) begin
                                pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                            end                        // else spin in EXEC
                        end
                        OP_DELAY: begin
                            if (imm12 == 16'd0) begin
                                pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                            end else begin
                                dcount <= ir[11:0];
                                psc    <= DELAY_PRESCALE[$clog2(DELAY_PRESCALE+1)-1:0];
                                state  <= S_DLY;
                            end
                        end
                        OP_HLT: begin
                            result  <= regs[7];
                            halted  <= 1'b1;
                            running <= 1'b0;
                            retire  <= 1'b1;
                            state   <= S_IDLE;
                        end
                        default: begin                 // unreachable; treat as NOP
                            pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                        end
                    endcase
                end

                // ---- load writeback ------------------------------------
                S_LOAD: begin
                    if (rd != 3'd0) regs[rd] <= mem_rdata;
                    pc <= pc + 1'b1; retire <= 1'b1; state <= S_FETCH;
                end

                // ---- delay countdown -----------------------------------
                S_DLY: begin
                    if (psc <= 1) begin
                        psc <= DELAY_PRESCALE[$clog2(DELAY_PRESCALE+1)-1:0];
                        if (dcount <= 12'd1) begin
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
