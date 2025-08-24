/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module reg_cal_norm (c_rm,c_is_nan,c_is_inf,c_inf_nan_frac,c_sign,c_exp,
                     c_frac,clk,clrn,e,n_rm,n_is_nan,n_is_inf,
                     n_inf_nan_frac,n_sign,n_exp,n_frac);   // pipeline regs
    input      [27:0] c_frac;
    input      [22:0] c_inf_nan_frac;
    input       [7:0] c_exp;
    input       [1:0] c_rm;
    input             c_is_nan, c_is_inf, c_sign;
    input             e;                                  // e: enable
    input             clk, clrn;                          // clock and reset
    output reg [27:0] n_frac;
    output reg [22:0] n_inf_nan_frac;
    output reg  [7:0] n_exp;
    output reg  [1:0] n_rm;
    output reg        n_is_nan,n_is_inf,n_sign;
    always @ (posedge clk or negedge clrn) begin
        if (!clrn) begin
            n_rm           <= 0;
            n_is_nan       <= 0;
            n_is_inf       <= 0;
            n_inf_nan_frac <= 0;
            n_sign         <= 0;
            n_exp          <= 0;
            n_frac         <= 0;
        end else if (e) begin
            n_rm           <= c_rm;
            n_is_nan       <= c_is_nan;
            n_is_inf       <= c_is_inf;
            n_inf_nan_frac <= c_inf_nan_frac;
            n_sign         <= c_sign;
            n_exp          <= c_exp;
            n_frac         <= c_frac;
        end
    end
endmodule
