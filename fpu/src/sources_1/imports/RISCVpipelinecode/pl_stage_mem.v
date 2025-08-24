`include "mfp_ahb_const.vh"

module pl_stage_mem (mwmem,mal,md,clk, clrn, mm,dbg_dmem_ce, dbg_dmem_we,dbg_dmem_din,dbg_dmem_addr,IO_Switch,
                                           IO_PB,IO_LED,IO_7SEGEN_N,
                                           IO_7SEG_N,IO_BUZZ,IO_RGB_SPI_MOSI,
                                           IO_RGB_SPI_SCK,IO_RGB_SPI_CS, IO_RGB_DC,
                                           IO_RGB_RST,IO_RGB_VCC_EN, IO_RGB_PEN,
                                           IO_CS,  IO_SCK,IO_SDO,UART_RX);

	input mwmem;
	input [31:0] mal;
	input [31:0] md;
	input clk,clrn;
	output [31:0] mm;
	input  dbg_dmem_ce;
   	input  dbg_dmem_we;
     	input  [31:0] dbg_dmem_din;
    	input  [31:0] dbg_dmem_addr;
    	input  [`MFP_N_SW-1 :0] IO_Switch;
    	input  [`MFP_N_PB-1 :0] IO_PB;
    	output [`MFP_N_LED-1:0] IO_LED;
    	output [ 7          :0] IO_7SEGEN_N;
    	output [ 6          :0] IO_7SEG_N;
    	output                  IO_BUZZ;                  
    	output                  IO_RGB_SPI_MOSI;
    	output                  IO_RGB_SPI_SCK;
    	output                  IO_RGB_SPI_CS;
    	output                  IO_RGB_DC;
    	output                  IO_RGB_RST;
    	output                  IO_RGB_VCC_EN;
    	output                  IO_RGB_PEN;
    	output                  IO_CS;
    	output                  IO_SCK;
    	input                   IO_SDO;
   	input                   UART_RX;

    	wire[31:0] data_mem; //data driven by data memory
    	wire[31:0] data_gpio; //data driven by GPIO module
    	wire[31:0] dataBus; //Resulting bus from the currently selected peripheral
        wire dataout_rdy;
    	assign mm = dataBus;
    
 	// write data memory // Check if memory mapped I/O
        wire[2:0] HSEL;
	//parameter DMEM_FILE = "/home/fpgauser/mips-cpu/Software/Assembly/HazardTest/dmem.mem";
//	parameter DMEM_FILE = "D:/RISCV-cpu/Software/Assembly/RISCVpipeLEDSwitches/dmem.mem";
//	parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/RISCVpipeSwitchLED7Seg/dmem.mem";
//	parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPUpipeline_test/dmem.mem";
//	parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPU_test2/dmem.mem";
//	parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPUpipeline_test/dmem.mem";
//	parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/RISCV-cpu/Software/Assembly/FPU_test2/dmem.mem";
    //parameter DMEM_FILE = "d:/mips-cpu/Software/Assembly/RISCVscmultiplytest/dmem.mem";
//    parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/riscv-cpu/Software/Assembly/RISCVsc_intTest/dmem.mem";
    parameter DMEM_FILE = "C:/JHU_Classes/RISC_V/riscv-cpu/Software/Assembly/RISCVscintSwitchLED7Seg/dmem.mem";
    
    	pipelinedcpu_decode pipelinedcpu_decode(mal,HSEL);
     	wire effectiveDMemWE = dbg_dmem_ce ? dbg_dmem_we : mwmem;
      	wire effectiveDMemCE = dbg_dmem_ce | HSEL[1];
      	wire[31:0] effectiveDMemAddr = dbg_dmem_ce ? dbg_dmem_addr : mal;
      uram #(.A_WIDTH(14), .INIT_FILE(DMEM_FILE), .READ_DELAY(0)) dmem
          (.clk(clk), .we(effectiveDMemWE), .cs(effectiveDMemCE), .addr(effectiveDMemAddr), .data_in(dataBus), .data_out(data_mem));     // data memory
    
   cpugpio gpio (.clk(clk),
        .clrn(clrn),
        .dataout(data_gpio),
        .dataout_ready(dataout_rdy),
        .datain(dataBus),
        .haddr(mal[7:2]),
        .we(mwmem),
        .HSEL(HSEL[2]),
        .IO_Switch(IO_Switch),
        .IO_PB(IO_PB),
        .IO_LED(IO_LED),
        .IO_7SEGEN_N(IO_7SEGEN_N),
        .IO_7SEG_N(IO_7SEG_N),
        .IO_BUZZ(IO_BUZZ),                
        .IO_RGB_SPI_MOSI(IO_RGB_SPI_MOSI),
        .IO_RGB_SPI_SCK(IO_RGB_SPI_SCK),
        .IO_RGB_SPI_CS(IO_RGB_SPI_CS),
        .IO_RGB_DC(IO_RGB_DC),
        .IO_RGB_RST(IO_RGB_RST),
        .IO_RGB_VCC_EN(IO_RGB_VCC_EN),
        .IO_RGB_PEN(IO_RGB_PEN),
        .IO_SDO(IO_SDO),
        .IO_CS(IO_CS),
        .IO_SCK(IO_SCK));
            
       
    
    assign dataBus = dbg_dmem_we ? dbg_dmem_din :
                     mwmem ? md : //data driven by cpu
                    HSEL[1] ? data_mem :
                    HSEL[2] ? data_gpio :
                    32'b0;
endmodule