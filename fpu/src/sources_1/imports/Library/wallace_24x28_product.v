/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module wallace_24x28_product (a,b,z);                    // 24*28 wt product
    input  [23:00] a;                                    // 24 bits
    input  [27:00] b;                                    // 28 bits
    output [51:00] z;                                    // product
    wire   [51:08] x;                                    // sum high
    wire   [51:08] y;                                    // carry high
    wire   [51:08] z_high;                               // product high
    wire   [07:00] z_low;                                // product low
    wallace_24x28 wt_partial (a, b, x, y, z_low);        // partial product
    assign z_high = x + y;
    assign z = {z_high,z_low};                           // product
endmodule
