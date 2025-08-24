module pl_reg_de ( cancel, wreg, m2reg, wmem, call, rv32m, 
        aluc, func3, dpc4, da, db, dd, rs1, rs2, rd, fuse, start_sdivide,start_udivide, clk,clrn,
                      ecancel,ewreg,em2reg,ewmem,ecall,erv32m, efuse,
                       ealuc,efunc3, epc4, ea,eb, ers1, ers2, erd, estart_sdivide,estart_udivide,
                       wremw,wfpr, ewfpr,ejal,jal,efwdfe,ed,fwdfe,dpc,epc,
                       ex_csr_en, ex_csr_addr, csr_wdata_ex, is_mret_ex, csr_en,csr_addr,mret );
    input clk;
    input clrn;
    input cancel;
    input wreg;
    input m2reg;
    input wmem;
    input call;
    input rv32m;
    input [3:0] aluc;
    input [2:0] func3;
    input [31:0] dpc4;
    input [31:0] da;
    input [31:0] db;
    input [31:0] dd;
    input [4:0] rs1;
    input [4:0] rs2;
    input [4:0] rd;
    input fuse;
    input start_sdivide,start_udivide;
    output   ecancel;
    output   ewreg;
    output   em2reg;
    output  ewmem;
    output  ecall;
    output erv32m;
    output efuse;
    output  [3:0] ealuc;
    output [2:0] efunc3;
    output  [31:0] ea;
    output reg [31:0] eb,ed;
//    output  [31:0] ed;
    output [4:0] ers1;
    output [4:0] ers2;
    output [4:0] erd;
    output  [31:0] epc4;
    output estart_sdivide,estart_udivide;
    input wremw;
//    input is_fpu;
    input wfpr;
    output reg ewfpr;
    output reg ejal;
    input jal;
    output efwdfe;
//    input [31:0] decode_b;
//    output [31:0] eb;
    input fwdfe;
    input [31:0] dpc;        // decode stage pc
    output [31:0] epc;
    output reg ex_csr_en;
    output reg [11:0] ex_csr_addr;
    output reg [31:0] csr_wdata_ex;
    output reg is_mret_ex;
    input csr_en;
    input [11:0] csr_addr; 
    input  mret;
    
    
//    reg [31:0] eb;
    reg ecancel;
    reg ewreg;
    reg em2reg;
    reg ewmem;
    reg ecall;
    reg [3:0] ealuc;
    reg [4:0] erd;
    reg [31:0] epc4;
    reg [31:0] ea;
//    reg [31:0] ed;
    reg [4:0] ers1;
    reg [4:0] ers2;
    reg erv32m; 
    reg [2:0] efunc3;
    reg efuse;
    wire estart_sdivide,estart_udivide;
    reg efwdfe; 
    reg [31:0] epc;
    
    assign estart_sdivide = start_sdivide;
    assign estart_udivide = start_udivide;
   
    always @(negedge clrn or posedge clk)
       if (!clrn) begin
        	ecancel <=0;
        	ewreg <=0;
        	em2reg <=0;
        	ewmem <=0;
        	ecall <=0;
        	ealuc <= 0;
        	erd <=0;
        	epc4 <=0;
        	ea <=0;
        	eb <=0;
        	ed <=0;
        	erd <=0;
            erv32m <=0;
            efunc3 <=0;
            ers1 <=0;
            ers2 <=0;
            efuse <=0;
//            eis_fpu <= 0;
            ewfpr <=0;
            ejal <= 0;
            efwdfe <= 0;
            epc <= 0;
     	    ex_csr_en <= 0;
            ex_csr_addr <= 0;
            csr_wdata_ex <= 0;
            is_mret_ex <= 0;
       end else begin
//       		if(wremw==1) begin
//       		if (cancel==1) begin
//           		ewreg <=0;
//        		em2reg <=0;
//        		ewmem <=0;
//        		ecall <=0;
//        		ealuc <= 0;
//        		erd <=0;
//        		epc4 <=0;
//        		ea <=0;
//        		eb <=0;
//        		ed <=0;
//        		erd <=0;
 //       	end else begin    			
       			ewreg <= wreg;
       			em2reg <= m2reg;
       			ewmem <= wmem;
       			ecall <= call;
       			ealuc <= aluc;
       			erd <= rd;
       			epc4 <= dpc4;
       			ea <= da;
       			eb <= db;
       			ed <= dd;
                erv32m <=rv32m;
                efunc3 <=func3;
                efuse <=fuse;
                ecancel <=cancel;
                ers1 <=rs1;
                ers2 <=rs2;
//                eis_fpu <= is_fpu;
                ewfpr <= wfpr;
                ejal <= jal;
                efwdfe <= fwdfe;
                epc <= dpc;
     	        ex_csr_en <= csr_en;
                ex_csr_addr <= csr_addr;
                csr_wdata_ex <= da;
                is_mret_ex <= mret;
//      		end
       end 
endmodule       
