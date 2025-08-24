/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module wallace_26x26_product (a,b,z);                    // 26*26 wt product
    input  [25:00] a;                                    // 26 bits
    input  [25:00] b;                                    // 26 bits
    output [51:00] z;                                    // product
    wire   [51:08] x;                                    // sum high
    wire   [51:08] y;                                    // carry high
    wire   [51:08] z_high;                               // product high
    wire   [07:00] z_low;                                // product low
    wallace_26x26 wt_partial (a, b, x, y, z_low);        // partial product
    assign z_high = x + y;
    assign z = {z_high,z_low};                           // product
endmodule
