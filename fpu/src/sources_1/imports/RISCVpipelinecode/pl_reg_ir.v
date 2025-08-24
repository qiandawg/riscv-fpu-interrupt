/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module pl_reg_ir (pc,pc4,ins,wir,clk,clrn,dpc,dpc4,inst);   // IF/ID pipeline register
    input         clk, clrn;                      // clock and reset
    input         wir;                            // write enable
    input  [31:0] pc;                             // pc  in IF stage
    input  [31:0] pc4;                            // pc + 4 in IF stage
    input  [31:0] ins;                            // instruction in IF stage
    output [31:0] dpc;                            // pc     in ID stage
    output [31:0] dpc4;                           // pc + 4 in ID stage
    output [31:0] inst;                           // instruction in ID stage
    // dffe32          (d,  clk,clrn,e,  q);
    dffe32 pc_reg      (pc, clk,clrn,wir,dpc);    // pc   register
    dffe32 pc_plus4    (pc4,clk,clrn,wir,dpc4);   // pc+4 register
    dffe32 instruction (ins,clk,clrn,wir,inst);   // inst register
endmodule
