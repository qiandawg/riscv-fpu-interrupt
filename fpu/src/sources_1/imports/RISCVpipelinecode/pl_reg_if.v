/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module pl_reg_if (pc,ins,clk, clrn, halt,dbg_imem_we, effectiveIMemAddr, dbg_imem_din);    // IF stage
    input  [31:0] pc;                                // program counter
    output [31:0] ins;                               // inst from inst mem
    input clk;
    input clrn;
    input  halt;
    input  dbg_imem_we;
    input  [31:0] effectiveIMemAddr;
    input  [31:0] dbg_imem_din;
       //Be sure to use forward slashes '/', even on Windows
//parameter IMEM_FILE = "/home/fpgauser/mips-cpu/Software/Assembly/HazardTest/imem.mem";
//parameter IMEM_FILE = "D:/RISCV-cpu/Software/Assembly/RISCVpipeLEDSwitches/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/RISCVpipeSwitchLED7Seg/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPUpipeline_test/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPU_test2/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPUpipeline_test/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPU_test2/imem.mem";
//parameter IMEM_FILE = "d:/mips-cpu/Software/Assembly/RISCVscmultiplytest/imem.mem";
//parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/riscv-cpu/Software/Assembly/RISCVsc_intTest/imem.mem";
parameter IMEM_FILE = "C:/JHU_Classes/RISC_V/riscv-cpu/Software/Assembly/RISCVscintSwitchLED7Seg/imem.mem";

   
    uram #(.A_WIDTH(14), .INIT_FILE(IMEM_FILE), .READ_DELAY(0)) imem     //BACKUP, Qian changing it to below version to avoid Bubble
        (.clk(clk), .we(dbg_imem_we), .cs(1'b1), .addr(effectiveIMemAddr), .data_in(dbg_imem_din), .data_out(ins));
//reg [31:0] pc_a;
//always @(posedge clk or negedge clrn)
//  if (!clrn) pc_a <= 32'b0;
//  else       pc_a <= effectiveIMemAddr;             // or pc[31:2] if word addressing

//uram #( .A_WIDTH(14), .INIT_FILE(IMEM_FILE), .READ_DELAY(0) ) imem (
//  .clk(clk), .we(dbg_imem_we), .cs(1'b1),
//  .addr(pc_a),   // word address is typical; adjust width
//  .data_in(dbg_imem_din),
//  .data_out(ins)
//);
   
endmodule






// IF→ID register
// inst_id <= ins_if when wpcir, else hold/NOP