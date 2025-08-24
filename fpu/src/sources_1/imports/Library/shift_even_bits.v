/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module shift_even_bits (a,b,sa);    // shift even bits until msb is 1x or 01
    input  [23:0] a;                // shift a = xxx...x by even bits
    output [23:0] b;                // to    b = 1xx...x or 01x...x
    output  [4:0] sa;               // shift amount, even number
    wire   [23:0] a5,a4,a3,a2,a1;
    assign a5 = a;
    assign sa[4]    = ~|a5[23:08];                               // 16-bit 0
    assign a4 = sa[4]? {a5[07:00],16'b0} : a5;
    assign sa[3]    = ~|a4[23:16];                               //  8-bit 0
    assign a3 = sa[3]? {a4[15:00], 8'b0} : a4;
    assign sa[2]    = ~|a3[23:20];                               //  4-bit 0
    assign a2 = sa[2]? {a3[19:00], 4'b0} : a3;
    assign sa[1]    = ~|a2[23:22];                               //  2-bit 0
    assign a1 = sa[1]? {a2[21:00], 2'b0} : a2;
    assign sa[0] = 0;
    assign b = a1;
endmodule
