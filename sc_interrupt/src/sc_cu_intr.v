module sc_cu_intr (opcode,func7,func3,z,aluc,alui,pcsrc,m2reg,bimm,call,wreg,wmem,
                    intr_synced,v,mstatus,cause,exc,wsta,wcau,wepc,
                    csrrs,csr_rw,selpc, csr_addr, mret,csr_en, mip, mie, intr_ack); // sc control unit
                    
    
    input  [6:0] opcode;
    input  [6:0] func7;
    input  [2:0] func3;
    input        z;
    input intr_synced;
//    output inta;
    input v;
    output [3:0] aluc;
    output [1:0] alui;
    output [1:0] pcsrc;
    output       m2reg;
    output       bimm;
    output       call;
    output       wreg;
    output       wmem;
    output  [31:0] cause;
    input   [31:0] mstatus;
    output        exc;                                    // exc or int occurs
    output        wsta;                                   // write status reg
    output        wcau;                                   // write cause reg
    output        wepc;                                   // move to c0 regs
//    output csr_en;
//    output [2:0] csr_cmd;
    output [1:0] csrrs;   // mtc0
    output csr_rw;   //mfc0
    output [1:0] selpc;
    output mret;
    input  [11:0] csr_addr;
    output csr_en;
    input [31:0] mip;
    input [31:0] mie;
    output intr_ack;
    

        // instruction decode
    wire i_lui = (opcode == 7'b0110111);
    wire i_jal   = (opcode == 7'b1101111);
    wire i_jalr = (opcode == 7'b1100111) & (func3 == 3'b000);
    wire i_beq   = (opcode == 7'b1100011) & (func3 == 3'b000);
    wire i_bne   = (opcode == 7'b1100011) & (func3 == 3'b001);
    wire i_lw    = (opcode == 7'b0000011) & (func3 == 3'b010);
    wire i_sw    = (opcode == 7'b0100011) & (func3 == 3'b010);
    wire i_addi  = (opcode == 7'b0010011) & (func3 == 3'b000);
    wire i_xori  = (opcode == 7'b0010011) & (func3 == 3'b100);
    wire i_ori   = (opcode == 7'b0010011) & (func3 == 3'b110);
    wire i_andi  = (opcode == 7'b0010011) & (func3 == 3'b111);
    wire i_slli  = (opcode == 7'b0010011) & (func3 == 3'b001) & (func7 == 7'b0000000);
    wire i_srli  = (opcode == 7'b0010011) & (func3 == 3'b101) & (func7 == 7'b0000000);
    wire i_srai  = (opcode == 7'b0010011) & (func3 == 3'b101) & (func7 == 7'b0100000);
    wire i_add   = (opcode == 7'b0110011) & (func3 == 3'b000) & (func7 == 7'b0000000);
    wire i_sub   = (opcode == 7'b0110011) & (func3 == 3'b000) & (func7 == 7'b0100000);
    wire i_slt   = (opcode == 7'b0110011) & (func3 == 3'b010) & (func7 == 7'b0000000);
    wire i_xor   = (opcode == 7'b0110011) & (func3 == 3'b100) & (func7 == 7'b0000000);
    wire i_or    = (opcode == 7'b0110011) & (func3 == 3'b110) & (func7 == 7'b0000000);
    wire i_and   = (opcode == 7'b0110011) & (func3 == 3'b111) & (func7 == 7'b0000000);

// Start of INterrupt code block //system type instruction  [ csr (12 bits) ][ rs1 ][funct3][  rd   ][ opcode ]
      wire i_csr    = (opcode == 7'b1110011);

    // Decode CSR instructions    
  wire i_csrrw  = i_csr & (func3 == 3'b001);
  wire i_csrrs  = i_csr & (func3 == 3'b010);
//  wire i_csrrc  = i_csr& (func3 == 3'b011);
//  wire i_csrrwi  = i_csr & (func3 == 3'b101);
//  wire i_csrrsi  = i_csr& (func3 == 3'b110);
//  wire i_csrrci = i_csr & (func3 == 3'b111);

  wire csr_rw = i_csrrw;
  wire csr_en = i_csr;
  
 wire i_mret = i_csr & (func3   == 3'b000)& (csr_addr== 12'h302);                      // opcode == 1110011
            
  wire i_ecall = (opcode == 7'b1110011) && (func3 == 3'b000) && (csr_addr == 12'h000); // i_syscall = ecall in riscv 
  wire mret = i_mret;
  wire unimplemented_inst = ~(i_csrrw | i_csrrs | i_mret | i_ecall | i_slt|
       i_add | i_sub | i_and | i_or | i_xor | i_slli| i_srli| i_srai|
       i_jalr | i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_beq |
       i_bne| i_lui| i_jal);
//  wire rd_is_status = (rd == 5'd12);                    // is cp0 status reg
//  wire rd_is_cause  = (rd == 5'd13);                    // is cp0 cause reg
//  wire rd_is_epc    = (rd == 5'd14);                    // is cp0 epc reg

    wire csr_is_mstatus = (csr_addr == 12'h300);
    wire csr_is_mcause  = (csr_addr == 12'h342);
    wire csr_is_mepc    = (csr_addr == 12'h341);

  wire overflow = v & (i_add | i_sub | i_addi);         // overflow
//  wire int_int  = sta[0] & intr;                        // sta[0]: enable OLD CODE
//  wire exc_sys  = sta[1] & i_ecall;                   // sta[1]: enable
//  wire exc_uni  = sta[2] & unimplemented_inst;          // sta[2]: enable
//  wire exc_ovr  = sta[3] & overflow;                    // sta[3]: enable


//// Raw line from external interrupt pin:
//wire ext_raw       = intr_synced;

//// Combine raw + pending bit:
//wire ext_pending   = ext_raw | mip[11];

// Maskable interrupt: only taken when MIE=1, MEIE=1, and pending  
wire int_int = mstatus[3]  // MIE bit in mstatus
             & mie[11]     // MEIE bit in mie CSR
             & mip[11];
wire intr_ack = int_int;
//---------------------------------------------------------------------
// 2) Synchronous exceptions (always trap when they occur)
//---------------------------------------------------------------------
wire exc_sys  = i_ecall;            // ECALL
wire exc_uni  = unimplemented_inst; // Illegal opcode
wire exc_ovr  = overflow;           // Your custom overflow trap





  // exccode: 0 0 : intr                                // generate exccode
  //          0 1 : i_syscall
  //          1 0 : unimplemented_inst
  //          1 1 : overflow
  wire exccode0 = i_ecall | overflow;
  wire exccode1 = unimplemented_inst | overflow;
  // mfc0:    0 0 : alu_mem                             // generate mux sel
  //          0 1 : sta
  //          1 0 : cau
  //          1 1 : epc
  assign csrrs[0] = i_csrrs & csr_is_mstatus | i_csrrs & csr_is_mepc;
  assign csrrs[1] = i_csrrs & csr_is_mcause  | i_csrrs & csr_is_mepc;
  // selpc:   0 0 : npc                                 // generate mux sel
  //          0 1 : epc
  //          1 0 : exc_base
  //          1 1 : x
//  assign selpc[0] = i_mret; WRONG no priority , 3 is possible
//  assign selpc[1] = exc; WRONG

// priority encode: exception (2'b10) else mret (2'b01) else normal (2'b00)
wire [1:0] selpc = exc     ? 2'b10 :
                   i_mret  ? 2'b01 :
                   2'b00 ;  //default
 
 
  assign cause = {28'h0,exccode1,exccode0,2'b00};       // cause
  assign exc   = int_int | exc_sys | exc_uni | exc_ovr; // exc or int occurs
  assign wsta  = exc | i_csrrw & csr_is_mstatus | i_mret;    // write status reg  
  assign wcau  = exc | i_csrrw & csr_is_mcause;              // write cause reg
  assign wepc  = exc | i_csrrw & csr_is_mepc;                // write epc reg
  assign regrt    = i_addi| i_andi| i_ori| i_xori| i_lw | i_lui| i_csrrs;
  
  // end of interrupt code block
  
  
  
  


    // control signals
  	 assign aluc[0]  = i_sub  | i_xori | i_xor  | i_andi  |
							 i_slli | i_srli |  i_srai | i_beq | i_bne;//
    assign aluc[1]  = i_xor  | i_slli  | i_srli  | i_srai  | i_xori | i_beq | // Check i_beq and i_bne
                     i_bne  | i_lui | i_slt; //
    assign aluc[2]  = i_or   | i_srli  | i_srai  | i_ori  | i_lui | i_andi; //
    assign aluc[3]  = i_xori | i_xor | i_srai | i_beq | i_bne;
    assign m2reg    = i_lw;
    assign wmem     = i_sw;
    assign wreg     = i_lui  | i_jal | i_jalr | i_lw | i_addi | i_xori | i_ori |
							 i_andi | i_slli | i_srli | i_srai | i_add | i_sub | i_slt | 
							 i_xor | i_or | i_and | i_csrrs;
    assign pcsrc[0] = i_beq & z | i_bne & ~z | i_jal; //
    assign pcsrc[1] = i_jal | i_jalr; //
    assign call     = i_jal | i_jalr; //
    assign alui[0]  = i_lui | i_slli | i_srli | i_srai; //
    assign alui[1]  = i_lui | i_sw; //
    assign bimm     = i_sw | i_lw | i_addi | i_lui | i_slli | i_srli | i_srai |
							 i_xori | i_ori | i_andi; //
							     
							 
//    reg [31:0] mstatus, mie, mtvec, mepc, mcause, mip;
    
   //TODO fix wreg same time as TRAP
   /*
   wire take_trap = (selpc == 2'b10);   // using your existing select
wire wreg_final = wreg & ~take_trap;
wire wmem_final = wmem & ~take_trap;

// use wreg_final and wmem_final where you currently use wreg/wmem
regfile rf (..., .wreg(wreg_final), ... );
/* your data memory write enable  assign mem_we = wmem_final;*/


         
endmodule
