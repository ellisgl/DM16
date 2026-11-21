`timescale 1ns/1ns
// bfloat16 arithmetic unit (one per PE), 3 register stages.
//
//   edge N   : s0 registers capture op, a, b (when start = 1)
//   edge N+1 : stage-A registers capture unpack / specials / alignment
//   during N+1..N+2 the result is valid (stage B, combinational) and the
//   PE writes it into acc at edge N+2.
//
// Format: 1 sign, 8 exponent (bias 127), 7 fraction - the top half of an
// IEEE-754 float32.
//
// Semantics:
//   * round to nearest, ties to even (results are correctly rounded:
//     the alignment frame is wide enough that add/sub/mul are exact
//     before the single rounding step)
//   * subnormal inputs and results are flushed to (signed) zero
//   * overflow -> +/-inf, any NaN or invalid op (inf-inf, 0*inf) -> 0x7FC0
//
// op: 0 = a + b, 1 = a - b, 2 = a * b, 3 = int16 -> bf16 (from a, rounded RNE)
module dm16_bf16 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [1:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] result
);

    localparam [15:0] QNAN = 16'h7FC0;

    // ------------------------------------------------------------------
    // s0: operand capture (keeps instruction decode / operand muxing off
    // the arithmetic paths)
    // ------------------------------------------------------------------
    reg [1:0]  s0_op;
    reg [15:0] s0_a, s0_b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_op <= 2'd0;
            s0_a  <= 16'd0;
            s0_b  <= 16'd0;
        end else if (start) begin
            s0_op <= op;
            s0_a  <= a;
            s0_b  <= b;
        end
    end

    // ------------------------------------------------------------------
    // Stage A (combinational from s0): unpack, specials, align / multiply
    // Frame: value = sig * 2^-16 * 2^(e - 127)
    // ------------------------------------------------------------------
    wire       sa = s0_a[15];
    wire       sb = s0_b[15] ^ (s0_op == 2'd1);     // subtract = add negated b
    wire [7:0] ea = s0_a[14:7];
    wire [7:0] eb = s0_b[14:7];
    wire [6:0] fa = s0_a[6:0];
    wire [6:0] fb = s0_b[6:0];

    wire a_zero = (ea == 8'd0);                     // includes FTZ'd subnormals
    wire b_zero = (eb == 8'd0);
    wire a_inf  = (ea == 8'hFF) && (fa == 7'd0);
    wire b_inf  = (eb == 8'hFF) && (fb == 7'd0);
    wire a_nan  = (ea == 8'hFF) && (fa != 7'd0);
    wire b_nan  = (eb == 8'hFF) && (fb != 7'd0);

    // add/sub: order by magnitude
    wire        a_big   = ({ea, fa} >= {eb, fb});
    wire        s_big   = a_big ? sa : sb;
    wire        s_sml   = a_big ? sb : sa;
    wire [7:0]  e_big   = a_big ? ea : eb;
    wire [7:0]  e_sml   = a_big ? eb : ea;
    wire [6:0]  f_big   = a_big ? fa : fb;
    wire [6:0]  f_sml   = a_big ? fb : fa;
    wire [7:0]  e_diff  = e_big - e_sml;
    wire [17:0] sig_big = {2'b01, f_big, 9'd0};
    wire [17:0] sig_sml = {2'b01, f_sml, 9'd0} >> e_diff[3:0];  // only used when e_diff < 10

    // multiply (DSP)
    wire [15:0] prod  = {1'b1, fa} * {1'b1, fb};                // value prod * 2^-14
    wire [10:0] e_mul = {3'd0, ea} + {3'd0, eb} - 11'd127;

    // int16 -> bf16 (operand a holds the integer)
    wire        i_neg = s0_a[15];
    wire [15:0] i_mag = i_neg ? (~s0_a + 16'd1) : s0_a;         // 0x8000 stays 32768

    reg         n_byp;
    reg  [15:0] n_byp_val;
    reg         n_sign;
    reg  [10:0] n_e;
    reg  [17:0] n_big, n_sml;
    reg         n_sub;

    always @(*) begin
        n_byp     = 1'b0;
        n_byp_val = 16'd0;
        n_sign    = 1'b0;
        n_e       = 11'd0;
        n_big     = 18'd0;
        n_sml     = 18'd0;
        n_sub     = 1'b0;
        case (s0_op)
            2'd0, 2'd1: begin
                if (a_nan || b_nan)            begin n_byp = 1; n_byp_val = QNAN; end
                else if (a_inf && b_inf)       begin n_byp = 1; n_byp_val = (sa != sb) ? QNAN : {sa, 15'h7F80}; end
                else if (a_inf)                begin n_byp = 1; n_byp_val = {sa, 15'h7F80}; end
                else if (b_inf)                begin n_byp = 1; n_byp_val = {sb, 15'h7F80}; end
                else if (a_zero && b_zero)     begin n_byp = 1; n_byp_val = {sa & sb, 15'd0}; end
                else if (a_zero)               begin n_byp = 1; n_byp_val = {sb, s0_b[14:0]}; end
                else if (b_zero)               begin n_byp = 1; n_byp_val = s0_a; end
                else if (e_diff >= 8'd10)      begin n_byp = 1; n_byp_val = {s_big, e_big, f_big}; end
                else begin
                    n_sign = s_big;
                    n_e    = {3'd0, e_big};
                    n_big  = sig_big;
                    n_sml  = sig_sml;
                    n_sub  = (s_big != s_sml);
                end
            end
            2'd2: begin
                if (a_nan || b_nan)                              begin n_byp = 1; n_byp_val = QNAN; end
                else if ((a_inf && b_zero) || (b_inf && a_zero)) begin n_byp = 1; n_byp_val = QNAN; end
                else if (a_inf || b_inf)                         begin n_byp = 1; n_byp_val = {sa ^ s0_b[15], 15'h7F80}; end
                else if (a_zero || b_zero)                       begin n_byp = 1; n_byp_val = {sa ^ s0_b[15], 15'd0}; end
                else begin
                    n_sign = sa ^ s0_b[15];
                    n_e    = e_mul;
                    n_big  = {prod, 2'b00};
                end
            end
            default: begin   // int16 -> bf16
                if (s0_a == 16'd0) begin n_byp = 1; n_byp_val = 16'd0; end
                else begin
                    n_sign = i_neg;
                    n_e    = 11'd143;                  // 127 + 16
                    n_big  = {2'd0, i_mag};
                end
            end
        endcase
    end

    reg         sA_byp;
    reg  [15:0] sA_byp_val;
    reg         sA_sign;
    reg  [10:0] sA_e;          // signed
    reg  [17:0] sA_big, sA_sml;
    reg         sA_sub;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sA_byp     <= 1'b1;
            sA_byp_val <= 16'd0;
            sA_sign    <= 1'b0;
            sA_e       <= 11'd0;
            sA_big     <= 18'd0;
            sA_sml     <= 18'd0;
            sA_sub     <= 1'b0;
        end else begin
            sA_byp     <= n_byp;
            sA_byp_val <= n_byp_val;
            sA_sign    <= n_sign;
            sA_e       <= n_e;
            sA_big     <= n_big;
            sA_sml     <= n_sml;
            sA_sub     <= n_sub;
        end
    end

    // ------------------------------------------------------------------
    // Stage B (combinational from stage-A regs): add, normalize, RNE round
    // ------------------------------------------------------------------
    wire [17:0] sig = sA_sub ? (sA_big - sA_sml) : (sA_big + sA_sml);

    reg [4:0] lz;
    integer i;
    always @(*) begin
        lz = 5'd0;
        for (i = 0; i <= 17; i = i + 1)
            if (sig[i]) lz = 5'd17 - i[4:0];   // highest set bit wins (last assignment)
    end

    wire [17:0] norm   = sig << lz;
    wire [10:0] e_norm = sA_e + 11'd1 - {6'd0, lz};
    wire [7:0]  mant   = norm[17:10];
    wire        g_bit  = norm[9];
    wire        sticky = |norm[8:0];
    wire        rup    = g_bit & (sticky | mant[0]);
    wire [8:0]  m_rnd  = {1'b0, mant} + {8'd0, rup};
    wire [10:0] e_fin  = m_rnd[8] ? e_norm + 11'd1 : e_norm;
    wire [6:0]  f_fin  = m_rnd[8] ? m_rnd[7:1] : m_rnd[6:0];

    always @(*) begin
        if (sA_byp)
            result = sA_byp_val;
        else if (sig == 18'd0)
            result = 16'd0;                                // exact cancellation: x - x = +0
        else if (!e_fin[10] && (e_fin >= 11'd255))
            result = {sA_sign, 8'hFF, 7'd0};               // overflow -> inf
        else if (e_fin[10] || (e_fin == 11'd0))
            result = {sA_sign, 15'd0};                     // underflow -> FTZ
        else
            result = {sA_sign, e_fin[7:0], f_fin};
    end

endmodule
