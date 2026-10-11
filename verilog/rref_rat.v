// rref_rat -- exact rational arithmetic for the RREF demo engine.
//
// Computes one thing:   r = x + k * y
// where every value is num/den, num a W-bit signed integer, den a W-bit
// positive integer, and the result is fully reduced (gcd(num, den) = 1,
// zero is 0/1).  That one operation covers both row operations that do
// arithmetic: SCALE is 0 + K*a, ADDMUL is a + K*b.
//
// It runs in two passes through the same "reduce" step:
//
//     pass 1:   p = k * y          = (kn*yn) / (kd*yd)          -> reduce
//     pass 2:   r = x + p          = (xn*pd + pn*xd) / (xd*pd)  -> reduce
//
// Reducing is: g = gcd(|num|, den); num /= g; den /= g.  The products use the
// plain Verilog `*`; only gcd and the (exact) divide are iterative, because a
// 64-bit combinational divider is not a sensible thing to build.
//
// If a reduced result does not fit W bits, `ovf` goes high for that result
// and the stored value is truncated -- the host is expected to check.
//
// Written to be read, not to be fast: a single op takes a few hundred cycles.
`default_nettype none

// Binary (Stein) gcd of two NONZERO numbers, one step per clock.
module rref_gcd #(parameter integer G = 64) (
    input  wire         clk,
    input  wire         start,
    input  wire [G-1:0] a,
    input  wire [G-1:0] b,
    output reg  [G-1:0] g,
    output reg          done
);
    reg [G-1:0] u, v;
    reg [7:0]   k;             // common factors of two
    reg [1:0]   phase = 0;     // 0 idle, 1 strip common 2s, 2 subtract
    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            u <= a; v <= b; k <= 0; phase <= 1;
        end else if (phase == 1) begin
            if (!u[0] && !v[0]) begin u <= u >> 1; v <= v >> 1; k <= k + 1; end
            else phase <= 2;
        end else if (phase == 2) begin
            if (u == 0)      begin g <= v << k; done <= 1'b1; phase <= 0; end
            else if (!u[0])  u <= u >> 1;
            else if (!v[0])  v <= v >> 1;
            else if (u >= v) u <= u - v;
            else             v <= v - u;
        end
    end
endmodule

// Unsigned long division, one quotient bit per clock (restoring).
module rref_div #(parameter integer G = 64) (
    input  wire         clk,
    input  wire         start,
    input  wire [G-1:0] n,
    input  wire [G-1:0] d,
    output reg  [G-1:0] q,
    output reg          done
);
    reg [G-1:0] rem, dd;
    reg [7:0]   bits = 0;
    wire [G:0]  trial = {rem, q[G-1]};         // remainder with next bit shifted in
    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            q <= n; rem <= 0; dd <= d; bits <= G;
        end else if (bits != 0) begin
            if (trial >= dd) begin rem <= trial - dd; q <= {q[G-2:0], 1'b1}; end
            else             begin rem <= trial;      q <= {q[G-2:0], 1'b0}; end
            bits <= bits - 1;
            if (bits == 1) done <= 1'b1;
        end
    end
endmodule

module rref_rat #(parameter integer W = 32) (
    input  wire                clk,
    input  wire                start,
    input  wire signed [W-1:0] xn, input wire [W-1:0] xd,
    input  wire signed [W-1:0] kn, input wire [W-1:0] kd,
    input  wire signed [W-1:0] yn, input wire [W-1:0] yd,
    output reg  signed [W-1:0] rn, output reg [W-1:0] rd,
    output reg                 ovf,     // this result overflowed
    output reg                 done
);
    localparam integer G = 2 * W + 2;       // room for the pass-2 sum

    // The unreduced fraction being worked on, as sign + magnitude / den.
    reg          neg;
    reg [G-1:0]  num, den, g;
    reg          pass2;

    // Pass-1 result, input to pass 2.
    reg signed [W-1:0] pn;
    reg        [W-1:0] pd;

    // Products, straight from the formulas above.  Registered, so a multiply
    // and the sum after it are not one long path; that costs one clock
    // (the MUL / MUL2 states below).
    reg signed [2*W-1:0] p1_num;
    reg        [2*W-1:0] p1_den;
    reg signed [2*W:0]   xn_pd, pn_xd;
    reg        [2*W-1:0] p2_den;
    always @(posedge clk) begin
        p1_num <= kn * yn;
        p1_den <= kd * yd;
        xn_pd  <= xn * $signed({1'b0, pd});
        pn_xd  <= pn * $signed({1'b0, xd});
        p2_den <= xd * pd;
    end
    wire signed [2*W+1:0] p2_num = xn_pd + pn_xd;

    reg          g_start, d_start;
    reg [G-1:0]  d_n, d_d;
    wire [G-1:0] g_out, d_out;
    wire         g_done, d_done;
    rref_gcd #(.G(G)) u_gcd (.clk(clk), .start(g_start), .a(num), .b(den),
                             .g(g_out), .done(g_done));
    rref_div #(.G(G)) u_div (.clk(clk), .start(d_start), .n(d_n), .d(d_d),
                             .q(d_out), .done(d_done));

    localparam [3:0] IDLE = 0, GCD = 1, WAIT_GCD = 2, WAIT_NUM = 3, WAIT_DEN = 4,
                     RESULT = 5, PASS2 = 6, MUL = 7, MUL2 = 8;
    reg [3:0] st = IDLE;

    // Put a signed numerator and a denominator into num/den, ready to reduce.
    task load(input signed [G-1:0] n, input [G-1:0] d);
        begin
            neg <= n < 0;
            num <= (n < 0) ? -n : n;
            den <= d;
        end
    endtask

    always @(posedge clk) begin
        done <= 1'b0; g_start <= 1'b0; d_start <= 1'b0;
        case (st)
            IDLE: if (start) begin
                pass2 <= 1'b0; ovf <= 1'b0;
                st <= MUL;                 // products register this clock
            end
            MUL: begin
                load(p1_num, p1_den);
                st <= GCD;
            end

            // --- reduce num/den ------------------------------------------
            GCD: begin
                // gcd needs nonzero inputs; 0/anything reduces to 0/1.
                if (num == 0) begin den <= 1; st <= RESULT; end
                else begin g_start <= 1'b1; st <= WAIT_GCD; end
            end
            WAIT_GCD: if (g_done) begin
                g <= g_out;
                d_n <= num; d_d <= g_out; d_start <= 1'b1;
                st <= WAIT_NUM;
            end
            WAIT_NUM: if (d_done) begin
                num <= d_out;
                d_n <= den; d_d <= g; d_start <= 1'b1;
                st <= WAIT_DEN;
            end
            WAIT_DEN: if (d_done) begin
                den <= d_out;
                st <= RESULT;
            end

            // --- num/den is now reduced ----------------------------------
            RESULT: begin
                if (num >= (1 << (W-1)) || den >= (1 << (W-1))) ovf <= 1'b1;
                if (!pass2) begin
                    pn <= neg ? -$signed(num[W-1:0]) : $signed(num[W-1:0]);
                    pd <= den[W-1:0];
                    st <= PASS2;
                end else begin
                    rn <= neg ? -$signed(num[W-1:0]) : $signed(num[W-1:0]);
                    rd <= den[W-1:0];
                    done <= 1'b1;
                    st <= IDLE;
                end
            end

            PASS2: begin               // pn/pd are the reduced k*y now
                pass2 <= 1'b1;
                st <= MUL2;                // products register this clock
            end
            MUL2: begin
                load(p2_num, p2_den);
                st <= GCD;
            end

            default: st <= IDLE;
        endcase
    end
endmodule

`default_nettype wire
