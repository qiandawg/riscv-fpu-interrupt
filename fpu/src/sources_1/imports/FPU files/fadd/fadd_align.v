/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fadd_align (a,b,sub,s_is_nan,s_is_inf,inf_nan_frac,sign,temp_exp,
                   op_sub,large_frac24,small_frac27);      //alignment stage
    input  [31:0] a,b;
    input         sub;
    output [26:0] small_frac27;
    output [23:0] large_frac24;
    output [22:0] inf_nan_frac;
    output  [7:0] temp_exp;
    output        s_is_nan;
    output        s_is_inf;
    output        sign;
    output        op_sub;
    wire          exchange = (b[30:0] > a[30:0]);
    wire   [31:0] fp_large = exchange? b : a;
    wire   [31:0] fp_small = exchange? a : b;
    wire          fp_large_hidden_bit = |fp_large[30:23];
    wire          fp_small_hidden_bit = |fp_small[30:23];
    wire   [23:0] large_frac24 = {fp_large_hidden_bit,fp_large[22:0]};
    wire   [23:0] small_frac24 = {fp_small_hidden_bit,fp_small[22:0]};
    assign        temp_exp = fp_large[30:23];
    assign        sign = exchange? sub ^ b[31] : a[31];  
    assign        op_sub = sub ^ fp_large[31] ^ fp_small[31];
    wire          fp_large_expo_is_ff =  &fp_large[30:23]; // exp == 0xff
    wire          fp_small_expo_is_ff =  &fp_small[30:23];
    wire          fp_large_frac_is_00 = ~|fp_large[22:0];  // frac == 0x0
    wire          fp_small_frac_is_00 = ~|fp_small[22:0];
    wire          fp_large_is_inf=fp_large_expo_is_ff & fp_large_frac_is_00;
    wire          fp_small_is_inf=fp_small_expo_is_ff & fp_small_frac_is_00;
    wire          fp_large_is_nan=fp_large_expo_is_ff &~fp_large_frac_is_00;
    wire          fp_small_is_nan=fp_small_expo_is_ff &~fp_small_frac_is_00;
    assign        s_is_inf = fp_large_is_inf | fp_small_is_inf;
    wire          s_is_nan = fp_large_is_nan | fp_small_is_nan |
                             ((sub ^ fp_small[31] ^ fp_large[31]) &
                             fp_large_is_inf & fp_small_is_inf);
    wire   [22:0] nan_frac = (a[21:0] > b[21:0])?
                  {1'b1,a[21:0]} : {1'b1,b[21:0]};
    assign        inf_nan_frac = s_is_nan? nan_frac : 23'h0;
    wire    [7:0] exp_diff = fp_large[30:23] - fp_small[30:23];
    wire          small_den_only = (fp_large[30:23] != 0) &
                                   (fp_small[30:23] == 0);
    wire    [7:0] shift_amount = small_den_only? exp_diff - 8'h1 : exp_diff;
    wire   [49:0] small_frac50 = (shift_amount >= 26)?
                  {26'h0,small_frac24} :
                  {small_frac24,26'h0} >> shift_amount;
    assign        small_frac27 = {small_frac50[49:24],|small_frac50[23:0]};
endmodule
