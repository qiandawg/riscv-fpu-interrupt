module pl_reg_mw(mwreg,mm2reg,mm,mal,mrd,clk,clrn,wwreg,wm2reg,wm,wal,wrd,wremw,mwfpr,wwfpr,mem_csr_en,mfunc3, mem_csr_addr, csr_wdata_mem, is_mret_mem,  
                          wb_csr_en,wfunc3, wb_csr_addr, csr_wdata_wb, is_mret_wb);

	input mwreg;
	input mm2reg;
	input [31:0] mm;
	input [31:0] mal;
	input [4:0] mrd;
	input clk;
	input clrn;
	output reg wwreg;
	output reg wm2reg;
	output reg [31:0] wm;
	output reg [31:0] wal;
	output reg [4:0] wrd;
	input wremw;
	input mwfpr;
	output reg wwfpr;
	input mem_csr_en;
	input [2:0] mfunc3;
	input [11:0] mem_csr_addr;
	input [31:0] csr_wdata_mem;
	input is_mret_mem;  
    output reg wb_csr_en;
    output reg [2:0] wfunc3;
    output reg [11:0] wb_csr_addr;
    output reg [31:0] csr_wdata_wb;
    output reg is_mret_wb;
	
    always @(negedge clrn or posedge clk)
       if (!clrn) begin
        	wwreg <=0;
        	wm2reg <=0;
        	wal <=0;
        	wm <=0;
        	wrd <=0;
        	wwfpr <= 0;
        	wb_csr_en<= 0;
            wfunc3  <= 0;
            wb_csr_addr <= 0;
            csr_wdata_wb <= 0;
            is_mret_wb <= 0;
       end else begin
//       if (wremw==1)begin
        	wwreg <=mwreg;
        	wm2reg <=mm2reg;
        	wal <=mal;
        	wm <=mm;
        	wrd <=mrd;
            wwfpr <= mwfpr;
            wb_csr_en<= mem_csr_en;
            wfunc3  <= mfunc3;
            wb_csr_addr <= mem_csr_addr;
            csr_wdata_wb <= csr_wdata_mem;
            is_mret_wb <= is_mret_mem;

       end 
endmodule                       
	
