/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module dffe32 (d,clk,clrn,e,q);                         // a 32-bit register
    input      [31:0] d;                                // input d
    input             e;                                // e: enable
    input             clk, clrn;                        // clock and reset
    output reg [31:0] q;                                // output q
    always @(negedge clrn or posedge clk)
        if (!clrn)  q <= 0;                             // q = 0 if reset
        else if (e) q <= d;                             // save d if enabled
endmodule
