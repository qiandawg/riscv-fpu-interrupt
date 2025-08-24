/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module fpu (a,b,fc,wf,fd,ein1,clk,clrn,ed,wd,wn,ww,st_ds,e1n,e1w,     // fpu
            e2n,e2w,e3n,e3w, e1c,e2c,e3c,count_div,count_sqrt,e,ein2,rm);
    input         clk, clrn;                // clock and reset
    input  [31:0] a, b;                     // 32-bit fp numbers
    input   [4:0] fd;                       // fp dest reg number
    input   [2:0] fc;                       // fp control
    input         wf;                       // write fp regfile
    input         ein1;                     // no_cache_stall
    input         ein2;                     // for canceling E1 inst
    output [31:0] ed,wd;                    // wd: fp result
    output  [4:0] count_div,count_sqrt;     // for testing
    output  [4:0] e1n,e2n,e3n,wn;           // reg numbers
    output  [1:0] e1c,e2c,e3c;              // for testing
    output        e1w,e2w,e3w,ww;           // write fp regfile
    output        st_ds;                    // stall caused by fdiv or fsqrt
    output        e;                        // ein1 & ~st_ds
    input    [1:0]rm;
    reg    [31:0] wd;
    reg    [31:0] efa,efb;
    reg     [4:0] e1n,e2n,e3n,wn;
    reg     [1:0] e1c,e2c,e3c;
    reg           e1w0,e2w,e3w,ww,sub;
    wire   [31:0] s_add,s_mul,s_div,s_sqrt;
    wire   [25:0] reg_x_div,reg_x_sqrt;
    wire          busy_div,stall_div,busy_sqrt,stall_sqrt;
    wire          fdiv  = fc[2] & ~fc[1];
    wire          fsqrt = fc[2] &  fc[1];
    assign        e1w   = e1w0  &  ein2;
    assign        e     = ein1  & ~st_ds;
    pipelined_fadder f_add  (efa,efb,sub,rm,s_add,clk,clrn,e);
    pipelined_fmul   f_mul  (efa,efb,rm,s_mul,clk,clrn,e);
    fdiv_newton      f_div  (a,b,rm,fdiv, e,clk,clrn,s_div, busy_div,
                             stall_div, count_div, reg_x_div );
    fsqrt_newton     f_sqrt (a,rm,fsqrt,e,clk,clrn,s_sqrt,busy_sqrt,
                             stall_sqrt,count_sqrt,reg_x_sqrt);
    assign st_ds = stall_div | stall_sqrt;
    mux4x32 fsel (s_add,s_mul,s_div,s_sqrt,e3c,ed);
    always @ (negedge clrn or posedge clk)
      if (!clrn) begin                      // pipeline registers
          sub <= 0;              efa  <= 0;              efb <= 0;
          e1c <= 0;              e1w0 <= 0;              e1n <= 0;
          e2c <= 0;              e2w  <= 0;              e2n <= 0;
          e3c <= 0;              e3w  <= 0;              e3n <= 0;
          wd  <= 0;              ww   <= 0;              wn  <= 0;
      end else if (e) begin
          sub <= fc[0];          efa  <= a;              efb <= b;
          e1c <= fc[2:1];        e1w0 <= wf;             e1n <= fd;
          e2c <= e1c;            e2w  <= e1w;            e2n <= e1n;
          e3c <= e2c;            e3w  <= e2w;            e3n <= e2n;
          wd  <= ed;             ww   <= e3w;            wn  <= e3n;
      end
endmodule
