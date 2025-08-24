module pl_stage_wb (wal,wm,wm2reg, wres); 
	input [31:0]wal;
	input [31:0]wm;
	input wm2reg;
	output [31:0] wres;
	
	mux2x32 WBMux (wal,wm,wm2reg,wres);
endmodule
