/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module reg_mul_add (m_rm,m_sign,m_exp10,m_is_nan,m_is_inf,m_inf_nan_frac,
                    m_sum,m_carry,m_z8,clk,clrn,e,a_rm,a_sign,a_exp10,
                    a_is_nan,a_is_inf,a_inf_nan_frac,a_sum,a_carry,a_z8);
    input      [39:0] m_sum;                            // partial mul stage
    input      [39:0] m_carry;
    input      [22:0] m_inf_nan_frac;
    input       [9:0] m_exp10;
    input       [7:0] m_z8;
    input       [1:0] m_rm;
    input             m_sign;
    input             m_is_nan;
    input             m_is_inf;
    input             e;                                // enable
    input             clk, clrn;                        // clock and reset
    output reg [39:0] a_sum;                            // addition stage
    output reg [39:0] a_carry;
    output reg [22:0] a_inf_nan_frac;
    output reg  [9:0] a_exp10;
    output reg  [7:0] a_z8;
    output reg  [1:0] a_rm;
    output reg        a_sign;
    output reg        a_is_nan;
    output reg        a_is_inf;
    always @ (posedge clk or negedge clrn) begin
        if (!clrn) begin
            a_rm           <= 0;
            a_sign         <= 0;
            a_exp10        <= 0;
            a_is_nan       <= 0;
            a_is_inf       <= 0;
            a_inf_nan_frac <= 0;
            a_sum          <= 0;
            a_carry        <= 0;
            a_z8           <= 0;
        end else if (e) begin
            a_rm           <= m_rm;
            a_sign         <= m_sign;
            a_exp10        <= m_exp10;
            a_is_nan       <= m_is_nan;
            a_is_inf       <= m_is_inf;
            a_inf_nan_frac <= m_inf_nan_frac;
            a_sum          <= m_sum;
            a_carry        <= m_carry;
            a_z8           <= m_z8;
        end
    end
endmodule
