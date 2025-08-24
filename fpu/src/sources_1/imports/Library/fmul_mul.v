/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fmul_mul (a,b,sign,exp10,s_is_nan,s_is_inf,inf_nan_frac,z_sum,
                 z_carry,z8);                                   // mul stage
    input  [31:0] a, b;
    output [39:0] z_sum;
    output [39:0] z_carry;
    output [22:0] inf_nan_frac;
    output  [9:0] exp10;
    output  [7:0] z8;
    output        sign;
    output        s_is_nan;
    output        s_is_inf;
    wire          a_expo_is_00 = ~|a[30:23];                    // exp = 00
    wire          b_expo_is_00 = ~|b[30:23];
    wire          a_expo_is_ff =  &a[30:23];                    // exp = ff
    wire          b_expo_is_ff =  &b[30:23];
    wire          a_frac_is_00 = ~|a[22:0];                     // frac = 0
    wire          b_frac_is_00 = ~|b[22:0];
    wire          a_is_inf = a_expo_is_ff &  a_frac_is_00;
    wire          b_is_inf = b_expo_is_ff &  b_frac_is_00;
    wire          a_is_nan = a_expo_is_ff & ~a_frac_is_00;
    wire          b_is_nan = b_expo_is_ff & ~b_frac_is_00;
    wire          a_is_0   = a_expo_is_00 &  a_frac_is_00;
    wire          b_is_0   = b_expo_is_00 &  b_frac_is_00;
    assign        s_is_inf = a_is_inf |  b_is_inf;
    assign        s_is_nan = a_is_nan | (a_is_inf & b_is_0) |
                             b_is_nan | (b_is_inf & a_is_0);
    wire   [22:0] nan_frac = (a[21:0] > b[21:0])?
                             {1'b1,a[21:0]} : {1'b1,b[21:0]};
    assign        inf_nan_frac = s_is_nan? nan_frac : 23'h0;
    assign        sign = a[31] ^ b[31];  
    assign        exp10 = {2'h0,a[30:23]} + {2'h0,b[30:23]} - 10'h7f +
                           a_expo_is_00 + b_expo_is_00;         // -126
    wire   [23:0] a_frac24 = {~a_expo_is_00,a[22:0]};
    wire   [23:0] b_frac24 = {~b_expo_is_00,b[22:0]};
    wallace_24x24 wt24 (a_frac24,b_frac24,z_sum,z_carry,z8);
endmodule
