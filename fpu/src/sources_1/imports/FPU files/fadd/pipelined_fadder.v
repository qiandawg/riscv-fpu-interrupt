/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module pipelined_fadder (a,b,sub,rm,s,clk,clrn,e);    // pipelined fp adder
    input         clk, clrn;                          // clock and reset
    input  [31:0] a, b;                               // fp a and b
    input   [1:0] rm;                                 // round mode
    input         sub;                                // 1: sub; 0: add
    input         e;                                  // enable
    output [31:0] s;                                  // fp output
    wire   [26:0] a_small_frac;
    wire   [23:0] a_large_frac;
    wire   [22:0] a_inf_nan_frac;
    wire    [7:0] a_exp;
    wire          a_is_nan,a_is_inf;
    wire          a_sign;
    wire          a_op_sub;
    // exe1: alignment stage
    fadd_align alignment (a,b,sub,a_is_nan,a_is_inf,a_inf_nan_frac,a_sign,
                          a_exp,a_op_sub,a_large_frac,a_small_frac);
    wire   [26:0] c_small_frac;
    wire   [23:0] c_large_frac;
    wire   [22:0] c_inf_nan_frac;
    wire    [7:0] c_exp;
    wire    [1:0] c_rm;
    wire          c_is_nan,c_is_inf;
    wire          c_sign;
    wire          c_op_sub;
    // pipelined registers
    reg_align_cal reg_ac (rm,a_is_nan,a_is_inf,a_inf_nan_frac,a_sign,a_exp,
                          a_op_sub,a_large_frac,a_small_frac,clk,clrn,e,
                          c_rm,c_is_nan,c_is_inf,c_inf_nan_frac,c_sign,
                          c_exp,c_op_sub,c_large_frac,c_small_frac);
    wire   [27:0] c_frac;
    // exe2: calculation stage
    fadd_cal calculation (c_op_sub,c_large_frac,c_small_frac,c_frac);
    wire   [27:0] n_frac;
    wire   [22:0] n_inf_nan_frac;
    wire    [7:0] n_exp;
    wire    [1:0] n_rm;
    wire          n_is_nan,n_is_inf;
    wire          n_sign;
    // pipelined registers
    reg_cal_norm reg_cn (c_rm,c_is_nan,c_is_inf,c_inf_nan_frac,c_sign,c_exp,
                         c_frac,clk,clrn,e,n_rm,n_is_nan,n_is_inf,
                         n_inf_nan_frac,n_sign,n_exp,n_frac);
    // exe3: normalization stage
    fadd_norm normalization (n_rm,n_is_nan,n_is_inf,n_inf_nan_frac,n_sign,
                             n_exp,n_frac,s);
endmodule
