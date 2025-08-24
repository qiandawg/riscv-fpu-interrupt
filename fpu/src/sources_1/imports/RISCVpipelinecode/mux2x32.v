/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module mux2x32 (a0,a1,s,y);                    // multiplexer, 32 bits
    input  [31:0] a0, a1;                      // inputs, 32 bits
    input         s;                           // input,   1 bit
    output [31:0] y;                           // output, 32 bits
    assign        y = s ? a1 : a0;             // if (s==1) y=a1; else y=a0;
endmodule
