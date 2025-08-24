/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fsqrt_newton (d,rm,fsqrt,ena,clk,clrn, s,busy,stall,count,reg_x);
    input         clk, clrn;                    // clock and reset
    input  [31:0] d;                            // fp s = root(d)
    input   [1:0] rm;                           // round mode
    input         fsqrt;                        // ID stage: fsqrt = i_fsqrt
    input         ena;                          // enable
    output [31:0] s;                            // fp output
    output [25:0] reg_x;                        // x_i
    output  [4:0] count;                        // for iteration control
    output        busy;                         // for generating stall
    output        stall;                        // for pipeline stall
    parameter     ZERO = 32'h00000000;
    parameter     INF  = 32'h7f800000;
    parameter     NaN  = 32'h7fc00000;
    wire          d_expo_is_00 = ~|d[30:23];    // d_expo = 00
    wire          d_expo_is_ff =  &d[30:23];    // d_expo = ff
    wire          d_frac_is_00 = ~|d[22:0];     // d_frac = 00
    wire          sign = d[31];
    //            e_q   = (e_d >> 1)      +    63 + (e_d % 2)
    wire    [7:0] exp_8 = {1'b0,d[30:24]} + 8'd63 + d[23];  // normalized
    //                  = 0               +    63 + 0 = 63  // denormalized
    //            d_f24 = denormalized? .f_d,0 :    .1,f_d  // shifted 1 bit
    wire   [23:0] d_f24 = d_expo_is_00? {d[22:0],1'b0} : {1'b1,d[22:0]};
    //            tmp = e_d is odd? shift one more bit : 1 bit, no change
    wire   [23:0] d_temp24 = d[23]? {1'b0,d_f24[23:1]} : d_f24;
    wire   [23:0] d_frac24;  // .1xx...x or .01x...x for denormalized number
    wire    [4:0] shamt_d;   // shift amount, even number
    shift_even_bits shift_d (d_temp24,d_frac24,shamt_d);
    // denormalized: e_q = 63 - shamt_d / 2
    // normalized:   e_q = exp_8 - 0
    wire    [7:0] exp0 = exp_8 - {4'h0,shamt_d[4:1]};
    reg           e1_sign,e1_e00,e1_eff,e1_f00;
    reg           e2_sign,e2_e00,e2_eff,e2_f00;
    reg           e3_sign,e3_e00,e3_eff,e3_f00;
    reg     [1:0] e1_rm,e2_rm,e3_rm;
    reg     [7:0] e1_exp,e2_exp,e3_exp;
    always @ (negedge clrn or posedge clk)
       if (!clrn) begin  // 3 pipeline registers: reg_e1, reg_e2, and reg_e3
          // reg_e1                reg_e2              reg_e3
          e1_sign <= 0;             e2_sign <= 0;        e3_sign <= 0;      
          e1_rm   <= 0;             e2_rm   <= 0;        e3_rm   <= 0;
          e1_exp  <= 0;             e2_exp  <= 0;        e3_exp  <= 0;
          e1_e00  <= 0;             e2_e00  <= 0;        e3_e00  <= 0;
          e1_eff  <= 0;             e2_eff  <= 0;        e3_eff  <= 0;
          e1_f00  <= 0;             e2_f00  <= 0;        e3_f00  <= 0;
      end else if (ena) begin
          e1_sign <= sign;          e2_sign <= e1_sign;  e3_sign <= e2_sign;
          e1_rm   <= rm;            e2_rm   <= e1_rm;    e3_rm   <= e2_rm;
          e1_exp  <= exp0;          e2_exp  <= e1_exp;   e3_exp  <= e2_exp;
          e1_e00  <= d_expo_is_00;  e2_e00  <= e1_e00;   e3_e00  <= e2_e00;
          e1_eff  <= d_expo_is_ff;  e2_eff  <= e1_eff;   e3_eff  <= e2_eff;
          e1_f00  <= d_frac_is_00;  e2_f00  <= e1_f00;   e3_f00  <= e2_f00;
      end
    wire   [31:0] frac0;                                // root = 1.xxxx...x
    root_newton24 frac_newton (d_frac24,fsqrt,ena,clk,clrn,
                               frac0,busy,count,reg_x,stall);
    wire   [26:0] frac = {frac0[31:6],|frac0[5:0]};     // sticky
    wire          frac_plus_1 =
        ~e3_rm[1] & ~e3_rm[0] &  frac[3] &  frac[2] & ~frac[1]  & ~frac[0] |
        ~e3_rm[1] & ~e3_rm[0] &  frac[2] & (frac[1] |  frac[0]) |
        ~e3_rm[1] &  e3_rm[0] & (frac[2] |  frac[1] |  frac[0]) &  e3_sign |
         e3_rm[1] & ~e3_rm[0] & (frac[2] |  frac[1] |  frac[0]) & ~e3_sign;
    wire   [24:0] frac_rnd = {1'b0,frac[26:3]} + frac_plus_1;
    wire    [7:0] expo_new = frac_rnd[24]? e3_exp + 8'h1 : e3_exp;
    wire   [22:0] frac_new = frac_rnd[24]? frac_rnd[23:1]: frac_rnd[22:0];
    assign s = final_result(e3_sign,e3_e00,e3_eff,e3_f00,
                           {e3_sign,expo_new,frac_new});
    function  [31:0] final_result;
        input        d_sign,d_e00,d_eff,d_f00;
        input [31:0] calc;
        casex ({d_sign,d_e00,d_eff,d_f00})
            4'b1xxx : final_result = NaN;       // -
            4'b000x : final_result = calc;      // nor
            4'b0100 : final_result = calc;      // den
            4'b0010 : final_result = NaN;       // nan
            4'b0011 : final_result = INF;       // inf
            default : final_result = ZERO;      // 0
        endcase
    endfunction
endmodule
