module pl_id_cu (
    input clk, clrn,
    input  [6:0] opcode,
    input  [6:0] func7,
    input  [2:0] func3,
    input  [4:0] rs1,
    input  [4:0] rs2,
    input  [4:0] mrd,
    input        mm2reg,
    input        mwreg,
    input  [4:0] erd,
    input        em2reg,
    input        ewreg,
    input        ecancel,
    input        z,
    input        mdwait,
    input        efuse,
    input [4:0] ers1,
    input [4:0] ers2,
    input [2:0] efunc3,
    input       erv32m,
    output   wpcir,
    output reg   wremw,
    output       cancel,
    output [3:0] aluc,
    output [1:0] alui,
    output [1:0] pcsrc,
    output       m2reg,
    output       bimm,
    output       calls,
    output       wreg,
    output       wmem,
    output       rv32m,
    output       fuse,
    output [1:0] fwda,
    output [1:0] fwdb,
    output start_sdivide,start_udivide,
    //fpu I/O
    output wire is_fpu,
    output [2:0] fc,
    input  [4:0] fs,ft,e1n,e2n,e3n,
    input mwfpr,ewfpr,
    output wf,
    input        e1w,e2w,e3w,stall_div_sqrt,st,
    output wfpr,
    output fwdla,fwdlb,fwdfa,fwdfb,
    output jal,
    output swfp,
    output fwdf,fwdfe,
    input [11:0] csr_addr,
    output trap_id_v,
    output [3:0] cause_id,
    output intr_id,
    input [31:0] mstatus,
    input [31:0] mip,
    input [31:0] mie,
    output mret,
    input id_v,
    output csr_en
    );

    // Instruction decode
    wire i_lui   = ~ecancel & (opcode == 7'b0110111);
    wire i_jal   = (opcode == 7'b1101111);
    wire i_jalr  = (opcode == 7'b1100111) & (func3 == 3'b000);
    wire i_beq   =  (opcode == 7'b1100011) & (func3 == 3'b000);
    wire i_bne   = (opcode == 7'b1100011) & (func3 == 3'b001);
    wire i_lw    = ~ecancel & (opcode == 7'b0000011) & (func3 == 3'b010);
    wire i_sw    = ~ecancel & (opcode == 7'b0100011) & (func3 == 3'b010);
    wire i_addi  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b000);
    wire i_xori  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b100);
    wire i_ori   = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b110);
    wire i_andi  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b111);
    wire i_slli  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b001) & (func7 == 7'b0000000);
    wire i_srli  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b101) & (func7 == 7'b0000000);
    wire i_srai  = ~ecancel & (opcode == 7'b0010011) & (func3 == 3'b101) & (func7 == 7'b0100000);
    wire i_add   = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b000) & (func7 == 7'b0000000);
    wire i_sub   = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b000) & (func7 == 7'b0100000);
    wire i_slt   = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b010) & (func7 == 7'b0000000);
    wire i_xor   = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b100) & (func7 == 7'b0000000);
    wire i_or    = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b110) & (func7 == 7'b0000000);
    wire i_and   = ~ecancel & (opcode == 7'b0110011) & (func3 == 3'b111) & (func7 == 7'b0000000);

    // RV32M Instructions
    wire i_mul    = (opcode == 7'b0110011) & (func3 == 3'b000) & (func7 == 7'b0000001);
    wire i_mulh   = (opcode == 7'b0110011) & (func3 == 3'b001) & (func7 == 7'b0000001);
    wire i_mulhsu = (opcode == 7'b0110011) & (func3 == 3'b010) & (func7 == 7'b0000001);
    wire i_mulhu  = (opcode == 7'b0110011) & (func3 == 3'b011) & (func7 == 7'b0000001);
    wire i_div    = (opcode == 7'b0110011) & (func3 == 3'b100) & (func7 == 7'b0000001);
    wire i_divu   = (opcode == 7'b0110011) & (func3 == 3'b101) & (func7 == 7'b0000001);
    wire i_rem    = (opcode == 7'b0110011) & (func3 == 3'b110) & (func7 == 7'b0000001);
    wire i_remu   = (opcode == 7'b0110011) & (func3 == 3'b111) & (func7 == 7'b0000001);
    
    // RV32F Floating-Point Instructions (basic, non-fused)
    wire i_fadd = ~ecancel & (opcode == 7'b1010011) & (func7 == 7'b0000000); // & (func3 == 3'b000);
    wire i_fsub = ~ecancel & (opcode == 7'b1010011) & (func7 == 7'b0000100); // & (func3 == 3'b000);
    wire i_fmul = ~ecancel & (opcode == 7'b1010011) & (func7 == 7'b0001000); // & (func3 == 3'b000);
    wire i_fdiv = ~ecancel & (opcode == 7'b1010011) & (func7 == 7'b0001100); // & (func3 == 3'b000);
    wire i_fsqrt= ~ecancel & (opcode == 7'b1010011) & (func7 == 7'b0101100); // & (func3 == 3'b000);
//    wire i_lwc1 = ~ecancel & (opcode == 7'b0000111) & (func3 == 3'b010); // load word to FPR     //  CUSTOM instruction to load to floating point Register
//    wire i_swc1 = ~ecancel & (opcode == 7'b0100111) & (func3 == 3'b010); // store word from FPR  //  CUSTOM instruction to load to floating point Register   
    
    wire stall_fp,stall_lwc1,stall_swc1;
    wire i_lwc1 = ~ecancel & (opcode == 7'b0000111) & (func3 == 3'b010); // load word to FPR     //  CUSTOM instruction to load to floating point Register TODO change to FLW
    wire i_swc1 = ~ecancel & (opcode == 7'b0100111) & (func3 == 3'b010); // store word from FPR  //  CUSTOM instruction to load to floating point Register
    wire stall_lw;
    wire   [2:0] fop;
    wire stall_others = stall_lw | stall_fp | stall_lwc1 | stall_swc1 | st;
    
    // Interrupts
    wire i_csr    = (opcode == 7'b1110011);
    // Decode CSR instructions    
    wire i_csrrw  = i_csr & (func3 == 3'b001);
    wire i_csrrs  = i_csr & (func3 == 3'b010);
    wire i_mret = i_csr & (func3   == 3'b000)& (csr_addr== 12'h302);                      // opcode == 1110011
    wire i_ecall   = id_v & ~ecancel & (opcode==7'b1110011) & (func3==3'b000) & (csr_addr==12'h000);  
      
    
    
    // Register source use
    wire i_rs1 = i_jalr | i_beq | i_bne | i_lw | i_sw | i_addi | i_xori | i_ori |
                 i_andi | i_slli | i_srli | i_srai | i_add | i_sub | i_slt | i_xor | i_or | i_and;

    wire i_rs2 = i_beq | i_bne | i_sw | i_add | i_sub | i_slt | i_xor | i_or | i_and ;
    
    wire       i_fs = i_fadd | i_fsub | i_fmul | i_fdiv | i_fsqrt; // use fs
    wire       i_ft = i_fadd | i_fsub | i_fmul | i_fdiv;           // use ft
    wire       fasmds;
    
    // Forwarding Logic
    reg [1:0] fwda_reg, fwdb_reg;
    wire mul_fuse, rem_fuse;
   
    
    
    
    always @(*) begin
        fwda_reg = 2'b00;
        if (ewreg && (erd != 0) && (erd == rs1) && ~em2reg)
            fwda_reg = 2'b01;
        else if (mwreg && (mrd != 0) && (mrd == rs1))
            fwda_reg = mm2reg ? 2'b11 : 2'b10;

        fwdb_reg = 2'b00;
        if (ewreg && (erd != 0) && (erd == rs2) && ~em2reg)
            fwdb_reg = 2'b01;
        else if (mwreg && (mrd != 0) && (mrd == rs2))
            fwdb_reg = mm2reg ? 2'b11 : 2'b10;
    end

    assign fwda = fwda_reg;
    assign fwdb = fwdb_reg;


    assign aluc[0]  = i_sub  | i_xori | i_xor  | i_andi  | i_slli | i_srli |  i_srai | i_beq | i_bne;//
    assign aluc[1]  = i_xor  | i_slli  | i_srli  | i_srai  | i_xori | i_beq | i_bne  | i_lui | i_slt; //
    assign aluc[2]  = i_or   | i_srli  | i_srai  | i_ori  | i_lui | i_andi; //
    assign aluc[3]  = i_xori | i_xor | i_srai | i_beq | i_bne;
    assign m2reg = i_lw;
    assign pcsrc[0] = wpcir & ( (i_beq & z) | (i_bne & ~z) | i_jal );
    assign pcsrc[1] = wpcir & ( i_jal | i_jalr );
    assign calls = i_jal | i_jalr;
    assign alui[0] = i_lui | i_slli | i_srli | i_srai;
    assign alui[1] = i_lui | i_sw | i_swc1| i_lwc1;
    assign bimm = i_sw | i_lw | i_addi | i_lui | i_slli | i_srli | i_srai | i_xori | i_ori | i_andi |i_swc1| i_lwc1;
    
    assign rv32m = i_mul | i_mulh | i_mulhsu | i_mulhu | i_div | i_divu | i_rem | i_remu;
    //assign fuse = 1'b0; // Not defined - placeholder

    //assign wpcir = ~(ewreg & em2reg & (erd != 0) &
    //                ((i_rs1 & (erd == rs1)) | (i_rs2 & (erd == rs2))));

    assign wreg = (i_lui | i_jal | i_jalr | i_lw | i_addi | i_xori |
                   i_ori | i_andi | i_slli | i_srli | i_srai |
                   i_add | i_sub | i_slt | i_xor | i_or | i_and | 
                   i_mul | i_mulh | i_mulhsu | i_mulhu | i_div | i_divu | i_rem | i_remu | i_fadd | i_fsub | i_fmul | i_fdiv| i_csrrs ) & wpcir;


        
    assign regrt = i_addi| i_andi| i_ori| i_xori| i_lw | i_lui| i_csrrs;
    
    assign wmem = (i_sw | i_swc1) & wpcir;
    assign cancel = pcsrc[0] | pcsrc[1];
    
    
    
    // Floating point Logic
    assign jal   = i_jal;
        // fop:  000: fadd  001: fsub  01x: fmul  10x: fdiv  11x: fsqrt
    
    assign fop[0]   = i_fsub;                     // fpu control code
    assign fop[1]   = i_fmul | i_fsqrt;
    assign fop[2]   = i_fdiv | i_fsqrt;
    
    // stall caused by fp data harzards LOGIC
    
    
    assign stall_lw = ewreg & em2reg & (erd != 0) & (i_rs1 & (erd == rs1) |
                                                     i_rs2 & (erd == rs2));
    
    // FPU stall logic like MIPS
    
    // Final FPU control signals
    assign fc = fop & {3{~stall_others}};  // Like MIPS: block fc if stall active
    assign wf = i_fs & wpcir;              // FPU reg write enable
    // if it is a floating point instruction via (i_fs, i_ft) we need to check if the floating point source/target register is in the previous instruction
    assign stall_fp = (e1w & (i_fs & (e1n == fs) | i_ft & (e1n == ft))) |  // if fs == e1n
                      (e2w & (i_fs & (e2n == fs) | i_ft & (e2n == ft)));
                      
                           
    assign fwdfa    = e3w & (e3n == fs);          // forward fpu e3d to fp a
    assign fwdfb    = e3w & (e3n == ft);          // forward fpu e3d to fp b
    assign wfpr     = i_lwc1 & wpcir;             // fp rf y write enable
    assign fwdla    = mwfpr & i_fs & (mrd == fs);        // forward mmo to fp a   ADDED logic to forward only if floating point instruction
    assign fwdlb    = mwfpr & i_fs & (mrd == ft);        // forward mmo to fp b   ADDED logic to forward only if floating point instruction
    assign stall_lwc1 = ewfpr & (i_fs & (erd == fs) | i_ft & (erd == ft));
    assign swfp       = i_swc1;                   // select signal
    assign fwdf       = swfp & e3w & (ft == e3n); // forward to id  stage
    assign fwdfe      = swfp & e2w & (ft == e2n); // forward to exe stage
    assign stall_swc1 = swfp & e1w & (ft == e1n); // stallS
    assign fasmds     = i_fs;                   // Qian:  not sure what this is for
    
 
    //Floating point logic end
    
    // Interrupt Start/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Start of INterrupt code block //system type instruction  [ csr (12 bits) ][ rs1 ][funct3][  rd   ][ opcode ]

    //  wire i_csrrc  = i_csr& (func3 == 3'b011);
    //  wire i_csrrwi  = i_csr & (func3 == 3'b101);
    //  wire i_csrrsi  = i_csr& (func3 == 3'b110);
    //  wire i_csrrci = i_csr & (func3 == 3'b111);
    
    wire csr_rw = i_csrrw;
    assign csr_en = i_csr;


    
    assign mret = i_mret;
    wire unimplemented_inst = ~(i_csrrw | i_csrrs | i_mret | i_ecall | i_lui |
                                 i_jal | i_jalr| i_beq | i_bne | i_lw| i_sw | 
                                 i_addi | i_xori | i_ori | i_andi | i_slli| 
                                 i_srli | i_srai | i_add  | i_sub | i_slt |
                                 i_xor | i_or | i_and | i_mul | i_mulh | i_mulhsu |
                                 i_mulhu  | i_div | i_divu  | i_rem  | i_remu |
                                 i_fadd | i_fsub | i_fmul | i_fdiv | i_fsqrt |
                                 i_lwc1 | i_swc1);


    
    // Keep ~ecancel: it squashes ID in the *same cycle* an older EX redirect arrives

    wire ecall_id  = i_ecall;                              // id_v already inside
    wire illegal_id= id_v & ~ecancel & unimplemented_inst;
    assign intr_id   = id_v & ~ecancel & mstatus[3] & mie[11] & mip[11];
    assign trap_id_v = illegal_id | i_ecall | intr_id;     // no double id_v/~ecancel

    
    
    
    assign cause_id =
        intr_id    ? 4'd0 :
        ecall_id   ? 4'd1 :
        illegal_id ? 4'd2 :
                     4'd0;   // default (unused when trap_id_v=0)
                     
                     
    // TODO decode Program counter  sent
    
    

  // end of interrupt code block
    
  // Interrupt End
  
    
    rv32m_fuse rv32m_fuse(
    .rv32m(rv32m),
    .erv32m(erv32m),
    .rs1(rs1),
    .rs2(rs2),
    .ers1(ers1),
    .ers2(ers2),
    .func3(func3),
    .efunc3(efunc3),
    .fuse(fuse),
    .mul_fuse(mul_fuse),
    .rem_fuse(rem_fuse)
);

  // wire start_sdivide,start_udivide;
    
 
        
    Start_Div Start_Div(
       .clk(clk),
       .func3(func3),
       .fuse(fuse),
       .rv32m(rv32m),
       .start_sdivide(start_sdivide),
       .start_udivide(start_udivide));


      assign wpcir = ~((ewreg & em2reg & (erd != 0) &((i_rs1 & (erd == rs1)) | (i_rs2 & (erd == rs2))))| stall_div_sqrt| stall_others);

endmodule
