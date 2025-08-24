                                         /************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module sccpu_intr (clk,clrn,halt,inst,mem,pc,wmem,alu,data,intr,int_sync,intr_ack);
    input  [31:0] inst;                           // inst from inst memory
    input  [31:0] mem;                            // data from data memory
    input         clk, clrn;                      // clock and reset
    input         halt;                           // suspend the PC at current location
    input         intr;                           // interrupt request      output [31:0] pc;                             // program counter
    output [31:0] alu;                            // alu output
    output [31:0] data;                           // data to data memory
    output [31:0] pc;
    output        wmem;                           // write data memory
    output        int_sync;                           // interrupt acknowledge
    output        intr_ack;
    parameter     BASE = 32'h00000008;            // exc/int handler entry
    parameter     ZERO = 32'h00000000;            // zero
    // instruction fields   // instruction fields
    wire    [6:0] opcode   = inst[6:0];               // op
    wire    [4:0] rs1   = inst[19:15];            // rs1
    wire    [4:0] rs2   = inst[24:20];             // rs2
    wire    [4:0] rd   = inst[11:7];             // rd
    wire    [2:0] func3 = inst[14:12];             // func3
    wire    [6:0] func7 = inst[31:25];             // func7
    wire   [15:0] imm  = inst[15:00];             // immediate
    wire   [11:0] csr_addr = inst[31:20];         // csr address  NEW
    
    // control signals
    wire    [3:0] aluc;                           // alu operation control
    wire    [1:0] pcsrc;                          // select pc source
    wire          wreg;                           // write regfile
    wire          bimm;                          // control to mux for immediate value
    wire          m2reg;                          // instruction is an lw
    wire    [1:0] alui;                          // alu input b is an i32
    wire          call;                            // control to mux for pc+4 vs output wb mux
    wire          wmem;                           // write memory
    
	wire    [31:0] imme32;
	wire	[31:0] braddr;
	wire    [31:0] jalraddr;
	wire    [31:0] jaladdr;
    wire    [1:0] csrrs;                           // move from c0 regs
    wire    [1:0] selpc;                          // select for pc
    wire          v;                              // overflow
    wire          exc;                            // exc or int occurs
//    wire          msta;                           // write status reg
//    wire          mcau;                           // write cause reg
//    wire          mepc;                           // write epc reg
    wire          csr_rw;                           // move to c0 regs   
    // datapath wires
    wire   [31:0] p4;                             // pc+4
    wire   [31:0] bpc;                            // branch target address
    wire   [31:0] npc;                            // next pc
    wire   [31:0] qa;                             // regfile output port a
    wire   [31:0] qb;                             // regfile output port b
    wire   [31:0] alua;                           // alu input a
    wire   [31:0] alub;                           // alu input b
    wire   [31:0] wd;                             // regfile write port data
    wire   [31:0] r;                              // alu out or mem
    wire          z;                              // alu, zero tag
    wire   [31:0] sta;                            // output of status reg
    wire   [31:0] mcause;                            // output of cause reg
    wire   [31:0] mcau;
    wire   [31:0] mie;
    wire   [31:0] mip;
    wire   [31:0] mepc;                            // output of epc reg
    wire   [31:0] sta_in;                         // data in for status reg
    wire   [31:0] cau_in;                         // data in for cause reg
    wire   [31:0] epc_in;                         // data in for epc reg
    wire   [31:0] sta_lr;                         // status left/right shift
    wire   [31:0] pc_npc;                         // pc or npc
    wire   [31:0] cause;                          // exc/int cause (from cu)
    wire   [31:0] res_c0;                         // r or c0 regs
    wire   [31:0] n_pc;                           // next pc
    wire   [31:0] sta_r = {4'h0,mstatus[31:4]};       // status >> 4
    wire   [31:0] sta_l = {mstatus[27:0],4'h0};       // status << 4
    wire mret;   
    wire [31:0] csr_rdata;
    wire [31:0] mstatus, trap_vector;
    reg intr_synced;
    reg reset_mip11;
    wire cu_intr_ack;
    wire intr_ack =  cu_intr_ack;
    wire int_sync = intr_synced;
    
    //async interrupt control
    // very simple edge-detect + latch
    reg sync0, sync1;

//
    always @(posedge clk or negedge clrn) begin
        if (!clrn) begin
            // Asynchronous reset
            sync0   <= 1'b0;
            sync1   <= 1'b0;
            intr_synced    <= 1'b0;
            reset_mip11 <= 1'b0;
        end else begin
            // 1) Two-stage synchronizer
            sync0 <= intr;
            sync1 <= sync0;
            reset_mip11 <= cu_intr_ack;
            // 2) Edge detect: set pending on rising edge
            if (sync0 & ~sync1)
                intr_synced <= 1'b1;
            // 3) Clear pending when core acks
            else if (cu_intr_ack)
                intr_synced <= 1'b0;
            // else retain previous pending
        end
    end
//
    
    // control unit
    sc_cu_intr cu(.opcode(opcode),.func7(func7),.func3(func3),.z(z),.aluc(aluc),.alui(alui)
                    ,.pcsrc(pcsrc),.m2reg(m2reg),.bimm(bimm),.call(call),.wreg(wreg),.wmem(wmem),
                    .intr_synced(intr_synced),.v(v),.mstatus(mstatus),.cause(cause),.exc(exc),.wsta(wsta),.wcau(wcau),.wepc(wepc),
                    .csrrs(csrrs),.csr_rw(csr_rw),.selpc(selpc), .csr_addr(csr_addr), .mret(mret), .csr_en(csr_en),.mip(mip),.mie(mie), .intr_ack(cu_intr_ack)); // control unit
    // datapath
    dff32 pcreg (n_pc,clk,clrn,pc);              // pc register
    pc4 pc4func (pc,halt,p4); // pc + 4 (pc + 0 if halt)
    mux2x32 alu_b (qb,imme32,bimm,alub);           // alu input b
    mux2x32 alu_m (alu,mem,m2reg,r);              // alu out or mem
//    mux2x32 link  (r,p4,call,wd);                  // r or p4   NEW
    mux4x32 nextpc(p4,braddr,jalraddr,jaladdr,pcsrc,npc);      // next pc
    regfile rf (rs1,rs2,wd,rd,wreg,clk,clrn,qa,qb); // register file
    alu_ov alunit (qa,alub,aluc,alu,z,v);            // alu
    
    
//    dffe32  c0sta (sta_in,clk,clrn,wsta,mstatus);     // c0 status register  NEW
//    dffe32  c0cau (cau_in,clk,clrn,wcau,mcau);     // c0 cause register
//    dffe32  c0epc (epc_in,clk,clrn,wepc,mepc);     // c0 epc register    
//    mux2x32 cau_x (cause,qa,csr_rw,cau_in);         // mux  for cause reg
//    mux2x32 sta_1 (sta_r,sta_l,exc,sta_lr);       // mux1 for status reg
//    mux2x32 sta_2 (sta_lr,qa,csr_rw,sta_in);        // mux2 for status reg
//    mux2x32 epc_1 (pc,npc,inta,pc_npc);           // mux1 for epc reg
//    mux2x32 epc_2 (pc_npc,qa,csr_rw,epc_in);        // mux2 for epc reg
//    mux4x32 nxtpc (npc,mepc,BASE,ZERO,selpc,n_pc); // mux for pc
//    mux4x32 fr_c0 (r,mstatus,mcau,mepc,csrrs,res_c0);    // r or c0 regs      

// start code change
    // single CSR engine:


    csr_unit csr (
      .clk       (clk),
      .reset     (clrn),
      .intr      (intr_synced),
      .cu_intr_ack(reset_mip11),
      // CSR instruction interface
      .csr_en    (csr_en),
      .csr_cmd   (func3),
      .csr_addr  (csr_addr),
      .csr_wdata (qa),           // rs1 → CSR write data
      .csr_rdata (csr_rdata),    // read data

      // trap/interrupt side
      .trap_set  (exc),          // from CU
      .trap_cause(cause),        // from CU
      .trap_pc   (pc),           // save PC
      .trap_vector(trap_vector),
      .mret      (mret),       // from CU

      // for datapath visibility
      .mstatus_out(mstatus),
      .mip(mip),
      .mie(mie),
      .mepc_out   (mepc)
    );

    // PC multiplexer now uses CSR's vector base:
    mux4x32 nxtpc (
      .a0(npc),                  // normal
      .a1(mepc),                 // mret returns here
      .a2(trap_vector),          // mtvec entry
      .a3(32'h00000000),         // unused
      .s(selpc),
      .y(n_pc) );

    // Final write-back data: either ALU/mem, CSR read, or link-PC:
//    wire [31:0] alu_or_mem;
//    mux2x32 alu_mem_mux(alu, mem, m2reg, alu_or_mem);

    wire [31:0] wb_base = csr_en ? csr_rdata : r;

    mux2x32 link_mux(wb_base, p4, call, wd);


// END Code change
    
    jal_addr jala(pc,inst,jaladdr);
    jalr_addr jalra(qa,inst,jalraddr);
    imme immeblock(inst,alui,imme32);
    branch_addr bra(pc,inst,braddr);
    assign data = qb;                             // regfile output port b
    
    //Bad instruction detector - used in simulation to catch if the C compiler
    //generates instructions unimplemented in this variant
    //mips_bad_inst_det binstd(.inst(inst));
endmodule
