/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fadd_cal (op_sub,large_frac24,small_frac27, cal_frac); // calculation
    input  [23:0] large_frac24;
    input         op_sub;
    input  [26:0] small_frac27;
    output [27:0] cal_frac;
    wire   [27:0] aligned_large_frac = {1'b0,large_frac24,3'b000};
    wire   [27:0] aligned_small_frac = {1'b0,small_frac27};
    assign        cal_frac = op_sub?
                             aligned_large_frac - aligned_small_frac :
                             aligned_large_frac + aligned_small_frac;
endmodule
