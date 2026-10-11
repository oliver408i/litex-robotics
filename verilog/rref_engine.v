// rref_engine -- elementary row operations on a small rational matrix, as a
// port-mapped block for PSC18SR.  DEMO: written to be read, not to be fast.
//
// The PSC18SR program owns the algorithm (pivot search, loop order); this
// block only does the row operations.  It hangs off the core's IO ports, the
// "ports, not opcodes" idea from docs/psc18sr_isa_draft.md:
//
//     OUT  CMD, rX        ; command word (below)
//     SETB CTL, #0        ; rising edge starts it
//     CLRB CTL, #0
//     WAIT STS, #0, 0     ; block until not busy
//
// Matrix: up to 8x8.  Element (r,c) lives at address {r,c} = r*8+c as a
// reduced fraction: signed W-bit numerator, positive W-bit denominator.  The
// host loads and reads it over CSRs while the engine is idle.
//
// Command word:
//     [13:11] a   [10:8] b   [7:5] c   [4] neg   [3] inv   [2:0] op
//
//     op 1  TEST    zero <- (M[a][c] == 0)
//     op 2  SWAP    Ra <-> Rb
//     op 3  GETK    K <- M[a][c]  (then negated if neg, then 1/K if inv)
//     op 4  SCALE   Ra <- K * Ra
//     op 5  ADDMUL  Ra <- Ra + K * Rb
//
// Status word:
//     [15:12] rows  [11:8] cols  [3] err  [2] ovf  [1] zero  [0] busy
//     ovf and err are sticky until the host clears them.  err = GETK with
//     inv on a zero element.
`default_nettype none

module rref_engine #(parameter integer W = 32) (
    input  wire         clk,
    input  wire         rst,

    // PSC18SR side
    input  wire [15:0]  cmd,
    input  wire         go,
    output wire [15:0]  sts,

    // Host side (reads/writes only take effect while idle)
    input  wire [3:0]   rows,
    input  wire [3:0]   cols,
    input  wire         clr_flags,
    input  wire [5:0]   h_addr,
    input  wire         h_we,
    input  wire [W-1:0] h_wn,
    input  wire [W-1:0] h_wd,
    output wire [W-1:0] h_rn,
    output wire [W-1:0] h_rd,
    output reg  [31:0]  ops_done
);
    localparam [2:0] OP_TEST = 1, OP_SWAP = 2, OP_GETK = 3, OP_SCALE = 4, OP_ADDMUL = 5;

    reg  busy, zero, ovf, err;
    assign sts = {rows, cols, 4'd0, err, ovf, zero, busy};

    // ---- matrix memory --------------------------------------------------
    // One read port and one write port, shared: the engine owns them while
    // busy, the host otherwise.  Reads are synchronous -- data appears one
    // clock after the address.
    reg [W-1:0] mem_n [0:63];
    reg [W-1:0] mem_d [0:63];
    integer z;
    initial for (z = 0; z < 64; z = z + 1) begin mem_n[z] = 0; mem_d[z] = 1; end

    reg  [5:0]  e_addr;                 // engine read address
    reg         e_we;
    reg  [5:0]  e_waddr;
    reg [W-1:0] e_wn, e_wd;

    wire [5:0]  raddr = busy ? e_addr  : h_addr;
    wire        we    = busy ? e_we    : h_we;
    wire [5:0]  waddr = busy ? e_waddr : h_addr;
    wire [W-1:0] wn   = busy ? e_wn    : h_wn;
    wire [W-1:0] wd   = busy ? e_wd    : h_wd;

    reg [W-1:0] rd_n, rd_d;             // read data
    always @(posedge clk) begin
        if (we) begin mem_n[waddr] <= wn; mem_d[waddr] <= wd; end
        rd_n <= mem_n[raddr];
        rd_d <= mem_d[raddr];
    end
    assign h_rn = rd_n;
    assign h_rd = rd_d;

    // ---- the rational unit: r = x + k*y -----------------------------------
    reg                 rat_start;
    reg  signed [W-1:0] kn;  reg [W-1:0] kd;       // K, set by GETK
    reg  signed [W-1:0] xn;  reg [W-1:0] xd;
    reg  signed [W-1:0] yn;  reg [W-1:0] yd;
    wire signed [W-1:0] rn;  wire [W-1:0] rdn;
    wire                rat_ovf, rat_done;
    rref_rat #(.W(W)) u_rat (.clk(clk), .start(rat_start),
                             .xn(xn), .xd(xd), .kn(kn), .kd(kd), .yn(yn), .yd(yd),
                             .rn(rn), .rd(rdn), .ovf(rat_ovf), .done(rat_done));

    // ---- command sequencer ----------------------------------------------
    reg [15:0] c;                       // latched command
    wire [2:0] op  = c[2:0];
    wire       inv = c[3];
    wire       neg = c[4];
    wire [2:0] col = c[7:5];
    wire [2:0] rb  = c[10:8];
    wire [2:0] ra  = c[13:11];

    reg [2:0]  j;                       // column being worked on
    reg        go_q;
    reg signed [W-1:0] tn;  reg [W-1:0] td;     // SWAP holding register

    // The source row for the arithmetic ops: SCALE uses Ra itself.
    wire [2:0] src = (op == OP_SCALE) ? ra : rb;

    localparam [3:0] IDLE = 0, START = 1,
                     ELEM_WAIT = 2, ELEM_USE = 3,             // TEST, GETK
                     SW_RD_B = 4, SW_GET_A = 5, SW_WR_A = 6, SW_WR_B = 7,
                     AR_RD_A = 8, AR_GET_Y = 9, AR_GET_X = 10, AR_WAIT = 11,
                     NEXT_COL = 12, FINISH = 13;
    reg [3:0] st;

    always @(posedge clk) begin
        rat_start <= 1'b0;
        e_we      <= 1'b0;
        go_q      <= go;
        if (clr_flags) begin ovf <= 1'b0; err <= 1'b0; end

        if (rst) begin
            st <= IDLE; busy <= 1'b0; zero <= 1'b0; ovf <= 1'b0; err <= 1'b0;
            kn <= 1; kd <= 1; ops_done <= 0;
        end else case (st)
            IDLE: if (go && !go_q) begin
                c <= cmd; busy <= 1'b1; j <= 0; st <= START;
            end

            START: case (op)
                OP_TEST, OP_GETK:  begin e_addr <= {ra, col}; st <= ELEM_WAIT; end
                OP_SWAP:           begin e_addr <= {ra, j};   st <= SW_RD_B;   end
                OP_SCALE, OP_ADDMUL: begin e_addr <= {src, j}; st <= AR_RD_A;  end
                default:           st <= FINISH;
            endcase

            // ---- TEST / GETK: one element --------------------------------
            ELEM_WAIT: st <= ELEM_USE;
            ELEM_USE: begin
                if (op == OP_TEST) begin
                    zero <= (rd_n == 0);
                end else if (!inv) begin
                    kn <= neg ? -$signed(rd_n) : $signed(rd_n);
                    kd <= rd_d;
                end else if (rd_n == 0) begin
                    err <= 1'b1;                        // 1/0
                end else begin
                    // 1/(n/d) = d/n, with the sign moved onto the numerator.
                    kn <= (rd_n[W-1] ^ neg) ? -$signed(rd_d) : $signed(rd_d);
                    kd <= rd_n[W-1] ? -rd_n : rd_n;
                end
                st <= FINISH;
            end

            // ---- SWAP, one column at a time ------------------------------
            SW_RD_B:  begin e_addr <= {rb, j}; st <= SW_GET_A; end
            SW_GET_A: begin tn <= rd_n; td <= rd_d; st <= SW_WR_A; end     // Ra[j]
            SW_WR_A:  begin                                                // rd = Rb[j]
                e_we <= 1'b1; e_waddr <= {ra, j}; e_wn <= rd_n; e_wd <= rd_d;
                st <= SW_WR_B;
            end
            SW_WR_B:  begin
                e_we <= 1'b1; e_waddr <= {rb, j}; e_wn <= tn; e_wd <= td;
                st <= NEXT_COL;
            end

            // ---- SCALE / ADDMUL: Ra[j] <- x + K * y ----------------------
            //   SCALE:  x = 0,      y = Ra[j]
            //   ADDMUL: x = Ra[j],  y = Rb[j]
            AR_RD_A:  begin e_addr <= {ra, j}; st <= AR_GET_Y; end
            AR_GET_Y: begin yn <= rd_n; yd <= rd_d; st <= AR_GET_X; end
            AR_GET_X: begin
                if (op == OP_SCALE) begin xn <= 0;    xd <= 1;    end
                else                begin xn <= rd_n; xd <= rd_d; end
                rat_start <= 1'b1;
                st <= AR_WAIT;
            end
            AR_WAIT: if (rat_done) begin
                if (rat_ovf) ovf <= 1'b1;
                e_we <= 1'b1; e_waddr <= {ra, j}; e_wn <= rn; e_wd <= rdn;
                st <= NEXT_COL;
            end

            NEXT_COL: begin
                if (j + 1 >= cols) st <= FINISH;
                else begin
                    j <= j + 1;
                    e_addr <= (op == OP_SWAP) ? {ra, j + 3'd1} : {src, j + 3'd1};
                    st <= (op == OP_SWAP) ? SW_RD_B : AR_RD_A;
                end
            end

            FINISH: begin busy <= 1'b0; ops_done <= ops_done + 1; st <= IDLE; end

            default: st <= IDLE;
        endcase
    end
endmodule

`default_nettype wire
