/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module mux2x5 (a0,a1,s,y);
    input  [4:0] a0,a1;
    input        s;
    output [4:0] y;
    assign y = s? a1 : a0;
endmodule
