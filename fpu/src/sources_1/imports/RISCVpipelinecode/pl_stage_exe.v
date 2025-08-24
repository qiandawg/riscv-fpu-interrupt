	
module pl_stage_exe(ea,eb,epc4,ealuc,ecall, eal,zout, trap_ex_v,trap_ex_is_intr,trap_ex_cause,ex_v,csr_is_ex,csr_rdata_ex);
	input   [31:0] ea;
	input   [31:0] eb;
	input [31:0] epc4;
	input [3:0] ealuc;
	input ecall;
	output [31:0] eal;
	output zout;
	output trap_ex_v;
	output trap_ex_is_intr;
	output [3:0] trap_ex_cause;
	input ex_v;
	input csr_is_ex;
	input [31:0] csr_rdata_ex;
	
	wire [31:0] ealu;

	alu alunit (ea,eb,ealuc,ealu,zout,overflow);            // alu
	
	wire [31:0] csrdata_or_alu;                            //Qian new
    mux2x32 csr_or_alu (ealu, csr_rdata_ex, csr_is_ex, csrdata_or_alu); //Qian New
    mux2x32 alu_eal   (csrdata_or_alu, epc4, ecall, eal);  // Qian new
   
    wire        trap_ex_v        = ex_v & overflow;
    wire        trap_ex_is_intr  = 1'b0;
    wire [3:0]  trap_ex_cause    = 4'd3;            // 3 = overflow
 

	
	
endmodule




// BELOW iS FUSED Code, currently still not working.
//module pl_stage_exe(clk, clrn,ea,eb,epc4,ealuc,ecall, ers1, ers2, efunc3, efuse, erv32m, estart_sdivide,estart_udivide,eal, mdwait,zout,eis_fpu);
//	input clk, clrn;
//	input   [31:0] ea;
//	input   [31:0] eb;
//	input [31:0] epc4;
//	input [3:0] ealuc;
//	input ecall;
//	input [2:0] efunc3;
//	output efuse;
//	input erv32m;
//	input [4:0] ers1;
//	input [4:0] ers2;
//	output [31:0] eal;
//	output mdwait;
//	output zout;
//	input estart_sdivide,estart_udivide;
//	input eis_fpu;
	
//	wire [31:0] ealu;
//	wire zout;
//	wire [31:0] al;
//	wire [31:0] c_rv32m;
	
//	alu alunit (ea,eb,ealuc,ealu,zout);            // alu
//	mux2x32 alu_eal (ealu,epc4,ecall,al);  
//	rv32m_fuseALU rv32MD(
//	   .rv32m(erv32m),
//	   .a(ea),
//	   .b(eb),
//	   .rs1(ers1),
//	   .rs2(ers2),
//	   .func3(efunc3),
//	   .clk(clk),
//	   .clrn(clrn),
//	   .ready(mdwait),
//	   .c(c_rv32m),
//	   .fuse(efuse),
//	   .estart_sdivide(estart_sdivide),
//	   .estart_udivide(estart_udivide));
    
//    mux2x32 rv32mmux (al,c_rv32m,erv32m,eal);
//endmodule
