/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module pl_reg_pc (npc,wpc,clk,clrn,pc);                      // program counter
    input         clk, clrn;                              // clock and reset
    input         wpc;                                    // pc write enable
    input  [31:0] npc;                                    // next pc
    output [31:0] pc;                                     // program counter
    // dffe32        (d,  clk,clrn,e,  q);
    dffe32 prog_cntr (npc,clk,clrn,wpc,pc);               // program counter
endmodule
