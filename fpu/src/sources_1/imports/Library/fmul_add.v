/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fmul_add (z_sum,z_carry,z);                               // fmul add
    input  [39:0] z_sum;
    input  [39:0] z_carry;
    output [47:8] z;
    assign        z = z_sum + z_carry;
endmodule
