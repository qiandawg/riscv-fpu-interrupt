    `include "mfp_ahb_const.vh"
    
    module pl_computer (      // pipelined cpu
        input SI_CLK100MHZ,
        input lock,
        input         SI_ClkIn,             // clock
        input         SI_Reset_N,           // reset
        output [31:0] pc,                   // program counter
        output [31:0] inst,                 // instruction in ID stage 
        output [31:0] eal,                  // alu or epc4 in EXE stage
        output [31:0] mal,                  // eal in MEM stage
        output [31:0] wres,                 // ? fixed: removed erroneous comma here
        output [31:0] e3d, wd,
        output  [4:0] e1n, e2n, e3n, wn,
        output        ww, stl_lw, stl_fp, stl_lwc1, stl_swc1, stl,
        output        e,
        input  [`MFP_N_SW-1 :0] IO_Switch,
        input  [`MFP_N_PB-1 :0] IO_PB, 
        output [`MFP_N_LED-1:0] IO_LED,
        output [ 7          :0] IO_7SEGEN_N,
        output [ 6          :0] IO_7SEG_N,
        output                  IO_BUZZ,
        output                  IO_RGB_SPI_MOSI,
        output                  IO_RGB_SPI_SCK,
        output                  IO_RGB_SPI_CS,
        output                  IO_RGB_DC,
        output                  IO_RGB_RST,
        output                  IO_RGB_VCC_EN,
        output                  IO_RGB_PEN,
        output                  IO_CS,
        output                  IO_SCK,
        input                   IO_SDO,
        input                   UART_RX,
        inout [8:1] JB,
        input [26:0] counter,
        input intr );
        
        wire clk;
        wire clrn;
        wire dbg_resetn_cpu;
        wire dbg_halt_cpu;
        
        assign clk = SI_ClkIn;
        assign clrn=SI_Reset_N & dbg_resetn_cpu; 
        wire[31:0] dbg_imem_addr;
        wire[31:0] dbg_imem_din;
        wire dbg_imem_ce;
        wire dbg_imem_we;
    
        wire[31:0] dbg_dmem_addr;
        wire[31:0] dbg_dmem_din;
        wire dbg_dmem_ce;
        wire dbg_dmem_we;
        wire [1:0] rm = inst[14]? 2'b00: inst[13:12]; // Only extracting bit 12 and 13 because we are only supporting 4 rounding modes.
        wire[31:0] effectiveIMemAddr = dbg_imem_ce ? dbg_imem_addr : pc;
    
        //Declare
        wire fuse, is_fpu,mwfpr,stall_div_sqrt,wfpr;
        wire jal,estart_sdivide,estart_udivide,ejal;
    
        wire   [31:0] qfa,qfb,fa,fb,dfa,dfb,wmo;   // for iu
        wire    [4:0] fs,ft,fd,wrn; 
        wire    [2:0] fc;
        wire    [1:0] e1c,e2c,e3c;                     // for testing
        wire          fwdla,fwdlb,fwdfa,fwdfb,efwdfe,fwdfe,wf,e1w,e2w,e3w,wwfpr;
        wire   [4:0] cnt_div,cnt_sqrt; // for debugging
        //for multi-threading, not used here.
        
        // signals in IF stage
        wire   [31:0] pc4;             // pc+4 in IF stage
        wire   [31:0] ins;             // instruction in IF stage
        wire   [31:0] npc;             // next pc in IF stage
        // signals in ID stage
        wire   [31:0] dpc;             // pc in ID stage
        wire   [31:0] dpc4;            // pc+4 in ID stage
        wire   [31:0] bra;             // branch target of beq and bne instructions
        wire   [31:0] jalra;           // jump target of jalr instruction
        wire   [31:0] jala;            // jump target of jal instruction
        wire   [31:0] da;              // operand a in ID stage
        wire   [31:0] db,decode_b;              // operand b in ID stage
        wire   [31:0] dd;              // reg data to mem in ID stage
        wire    [4:0] rd = inst[11:7]; // destination register number in ID stage
        wire    [3:0] aluc;            // alu control in ID stage
        wire    [1:0] pcsrc;           // next pc (npc) select in ID stage
        wire          wpcir;           // pipepc and pipeir write enable
        wire          m2reg;           // memory to register in ID stage
        wire          wreg;            // register file write enable in ID stage
        wire          wmem;            // memory write in ID stage
        wire          call;            // jalr, jal in ID stage
        wire          cancel;          // cancel in ID stage
        // signals in EXE stage
        wire   [31:0] epc4;            // pc+4 in EXE stage
        wire   [31:0] ea;              // operand a in EXE stage
        wire   [31:0] eb;              // operand b in EXE stage
        wire   [31:0] ed,edata;              // reg data to mem in EXE stage
        wire    [4:0] erd;             // destination register number in EXE stage
        wire    [3:0] ealuc;           // alu control in EXE stage
        wire          em2reg;          // memory to register in EXE stage
        wire          ewreg;           // register file write enable in EXE stage
        wire          ewfpr;
        wire          ewmem;           // memory write in EXE stage
        wire          ecall;           // jalr, jal in EXE stage
        wire          ecancel;         // cancel in EXE stage
        wire    [4:0] rs1;
        wire    [4:0] rs2;
        //wire    [4:0] rd;
        wire    [4:0] ers1;
        wire    [4:0] ers2;
        //wire    [4:0] erd;
        wire    [2:0] func3;
        wire    [2:0] efunc3;
        wire    [2:0] mfunc3;
        wire    [2:0] wfunc3;
        wire          erv32m;
        wire          efuse;
        // signals in MEM stage
        wire   [31:0] mm;              // memory data out in MEM stage
        wire   [31:0] md;              // reg data to mem in MEM stage
        wire    [4:0] mrd;             // destination register number in MEM stage
        wire          mm2reg;          // memory to register in MEM stage
        wire          mwreg;           // register file write enable in MEM stage
        wire          mwmem;           // memory write in MEM stage
        // signals in WB stage
        wire   [31:0] wal;             // mal in WB stage
        wire   [31:0] wm;              // memory data out in WB stage
        wire    [4:0] wrd;             // destination register number in WB stage
        wire          wm2reg;          // memory to register in WB stage
        wire          wwreg;           // register file write enable in WB stage
        wire          zout;
        wire          rv32m;
        wire          z=~|(da^dd);
        wire          mdwait;
        wire          start_sdivide,start_udivide,wremw;
        
        
        wire [31:0] mstatus, trap_vector;
        wire   [31:0] sta_r = {4'h0,mstatus[31:4]};       // status >> 4
        wire   [31:0] sta_l = {mstatus[27:0],4'h0};       // status << 4
        wire mret;   
        wire [31:0] csr_rdata;
    
        reg intr_synced;
        reg reset_mip11;
        wire cu_intr_ack;
        wire intr_ack =  cu_intr_ack;
        wire int_sync = intr_synced;
        
        //Interrupt addition Qian
        wire [31:0] epc;   // execution stage PC
        wire [31:0] mpc;   // memory stage PC
        wire [31:0] mie;   //  
        wire [31:0] mip;
        wire trap_if_v, trap_id_v, trap_ex_v, trap_mem_v;
        wire [3:0] cause_if, cause_id, cause_ex, cause_mem;
        wire       intr_if,  intr_id,  intr_ex,  intr_mem;
        
        reg        take_trap_r;
        reg [3:0]  cause_low_sel_r;
        reg        is_intr_sel_r;
        reg [31:0] trap_pc_source_r;
        wire [31:0] n_pc;
        reg if_v, id_v, ex_v, mem_v, wb_v;
        wire [31:0] mcause;
        wire [31:0] mtvec;
        wire trap_in_mem = 0;
        wire trap_in_ex;
        wire trap_in_id;
        wire trap_in_if;
        wire take_trap;
        wire bubble_mem;
        wire wwreg_final;
        wire [31:0] csr_wdata_ex,csr_wdata_mem, csr_wdata_wb;
        wire [31:0] trap_cause;
        wire [31:0] csr_rdata_ex;
        wire [31:0] mepc;
        wire [11:0] csr_addr;
        wire kill_wb;
         wire take_trap_raw;
         wire take_trap_pc;
        wire ex_csr_en, mem_csr_en, wb_csr_en;
        wire is_mret_ex, is_mret_mem, is_mret_wb;

        //STALL logic
        wire stall_mem = 0;  //dcache_busy; no dcache yet or when data miss/AXI wait
        wire flush_mem;  // EX stalls either because its op is multi-cycle or MEM can't accept it
        wire stall_ex  = 0;  // muldiv_busy | stall_mem; TODO
        wire flush_ex;   // ID stalls when it must bubble EX (load_use) or when EX/MEM are stalling
        wire stall_id  = 0;  // load_use | stall_ex;
        wire flush_id;     // IF stalls if IMEM can't deliver or if ID can't accept new work
        wire stall_if  =0 ; // ~imem_ready | stall_id;
        wire flush_if;// WB rarely stalls in a single-issue design (one write port); keep it 0
        wire stall_wb  = 1'b0;
        wire flush_wb;
        reg sync0, sync1;   // very simple edge-detect + latch  used for async interrupt control
        wire [11:0] ex_csr_addr, mem_csr_addr, wb_csr_addr;
        wire [1:0] selpc;
        
        // Interrupt addition end Qian
        
        
        
        
        
        // program counter
        pl_reg_pc prog_cnt (n_pc,wpcir,clk,clrn, pc);    //Qian modify
        
        pc4 pc4func (pc,dbg_halt_cpu,pc4); // pc + 4 (pc + 0 if halt)
        mux4x32 nextpc(pc4,bra,jalra,jala,pcsrc,npc);      // next pc
        pl_reg_if pipeif(pc,ins,clk, clrn, dbg_halt_cpu,dbg_imem_we, effectiveIMemAddr, dbg_imem_din);
        //pl_stage_if if_stage (pcsrc,pc,bra,jalra,jala,npc,pc4,ins);             // IF stage
        // IF/ID pipeline register
        pl_reg_ir fd_reg ( pc, pc4,ins, wpcir,clk,clrn, dpc,dpc4,inst);
        pl_stage_id id_stage (.mrd(mrd),.mm2reg(mm2reg),.mwreg(mwreg),.erd(erd),.em2reg(em2reg),.ewreg(ewreg),.ecancel(ecancel),.mdwait(mdwait),.dpc(dpc),.inst(inst),
                              .eal(eal),.mal(mal),.mm(mm),.wrd(wrd),.wres(wres),.wwreg(wwreg_final),.clk(clk),.clrn(clrn),.brad(bra),.jalrad(jalra),
                              .jalad(jala),.pcsrc(pcsrc),.wpcir(wpcir),.cancel(cancel),.wreg(wreg),.m2reg(m2reg),.wmem(wmem),.calls(call),.aluc(aluc),.da(da),
                              .db(db),.dd(dd),.rs1(rs1),.rs2(rs2),.func3(func3),.rv32m(rv32m),.fuse(fuse),.ers1(ers1),.ers2(ers2),.efunc3(efunc3),
                              .efuse(efuse),.erv32m(erv32m),.start_sdivide(start_sdivide),.start_udivide(start_udivide),.wremw(wremw),.is_fpu(is_fpu),
                              .fc(fc),.fs(fs),.ft(ft),.e1n(e1n),.e2n(e2n),.e3n(e3n),.z(zout),
                              .mwfpr(mwfpr),.ewfpr(ewfpr),.wf(wf),.e1w(e1w),.e2w(e2w),.e3w(e3w),.stall_div_sqrt(stall_div_sqrt),.st(1'b0),.wfpr(wfpr),
                              .fwdla(fwdla),.fwdlb(fwdlb),.fwdfa(fwdfa),.fwdfb(fwdfb),.fwdfe(fwdfe),.e3d(e3d),.dfb(dfb),.ed(ed),.efwdfe(efwdfe),.edata(edata),.jal(jal),
                              .csr_addr(csr_addr),.trap_id_v(trap_id_v),.cause_id(cause_id), .intr_id(intr_id),.mstatus(mstatus),.mip(mip),.mie(mie),.mret(mret), 
                              .id_v(id_v),.csr_en(csr_en)
                              );                // ID stage
        // ID/EXE pipeline register
        pl_reg_de de_reg ( .cancel(cancel), .wreg(wreg), .m2reg(m2reg), .wmem(wmem), .call(call), .rv32m(rv32m),
            .aluc(aluc), .func3(func3), .dpc4(dpc4), .da(da), .db(db), .dd(dd), .rs1(rs1), .rs2(rs2), .rd(rd), 
            .fuse(fuse), .start_sdivide(start_sdivide) ,.start_udivide(start_udivide), .clk(clk),.clrn(clrn), .ecancel(ecancel),
            .ewreg(ewreg),.em2reg(em2reg),.ewmem(ewmem), .ecall(ecall), .erv32m(erv32m), .efuse(efuse), 
                          .ealuc(ealuc), .efunc3(efunc3), .epc4(epc4), .ea(ea) ,.eb(eb), .ers1(ers1), .ers2(ers2), .erd(erd),
                          .estart_sdivide(estart_sdivide),.estart_udivide(estart_udivide),
                          .wremw(wremw),.wfpr(wfpr), .ewfpr(ewfpr), .ejal(ejal), .jal(jal), .efwdfe(efwdfe) , .ed(ed),.fwdfe(fwdfe)
                          ,.dpc(dpc),.epc(epc), .ex_csr_en(ex_csr_en), .ex_csr_addr(ex_csr_addr), .csr_wdata_ex(csr_wdata_ex),
                           .is_mret_ex(is_mret_ex),  .csr_en(csr_en)   ,  .csr_addr(csr_addr)   ,   .mret(mret)    
                          );

    //    pl_stage_exe exe_stage (clk, clrn, ea,eb,epc4,ealuc,ecall, ers1, ers2, efunc3, efuse, erv32m, estart_sdivide,estart_udivide,eal, mdwait, zout);                   // EXE stage
        pl_stage_exe exe_stage (ea,eb,epc4,ealuc,ecall, eal,zout,trap_ex_v,intr_ex,cause_ex,ex_v,ex_csr_en,csr_rdata_ex);                   // EXE stage
        // EXE/MEM pipeline register
        pl_reg_em em_reg (ewreg,em2reg,ewmem,eal,edata,erd,clk,clrn,
                          mwreg,mm2reg,mwmem,mal,md,mrd,wremw,mwfpr,ewfpr,epc,mpc,
                          ex_csr_en, efunc3, ex_csr_addr, csr_wdata_ex, is_mret_ex,
                          mem_csr_en, mfunc3, mem_csr_addr, csr_wdata_mem, is_mret_mem                          
                          );
        pl_stage_mem mem_stage (mwmem,mal,md,clk, clrn, mm,dbg_dmem_ce, dbg_dmem_we,dbg_dmem_din,dbg_dmem_addr,IO_Switch,
                                               IO_PB,IO_LED,IO_7SEGEN_N,
                                               IO_7SEG_N,IO_BUZZ,IO_RGB_SPI_MOSI,
                                               IO_RGB_SPI_SCK,IO_RGB_SPI_CS, IO_RGB_DC,
                                               IO_RGB_RST,IO_RGB_VCC_EN, IO_RGB_PEN,
                                               IO_CS,  IO_SCK,IO_SDO,UART_RX);                          // MEM stage
        // MEM/WB pipeline register
        pl_reg_mw mw_reg (mwreg,mm2reg,mm,mal,mrd,clk,clrn,wwreg,wm2reg,wm,wal,wrd,wremw,mwfpr,wwfpr,
                          mem_csr_en,mfunc3, mem_csr_addr, csr_wdata_mem, is_mret_mem,  
                          wb_csr_en,wfunc3, wb_csr_addr, csr_wdata_wb, is_mret_wb);
        assign wwreg_final = wwreg & ~kill_wb;   // used to prevent writes when wb interrupt happens
        pl_stage_wb wb_stage (wal,wm,wm2reg, wres);                             // WB stage
        
        //
        
        
        // FPU call
    
            // CORRECT for RV32F R-type (add.s, sub.s, etc.)
        assign fs = inst[19:15];
        assign ft = inst[24:20];
        
    //    and rd = inst[11:7];  // already declared
        
    
    //    wire fasmds;  // Qian: don't know what this is for so commented it out.
        
        regfile2w fpr (fs,ft,wd,wn,ww,wm,wrd,wwfpr,~clk,clrn,qfa,qfb);
        mux2x32 fwd_f_load_a (qfa,mm,fwdla,fa);       // forward lwc1 to fp a
        mux2x32 fwd_f_load_b (qfb,mm,fwdlb,fb);       // forward lwc1 to fp b
        mux2x32 fwd_f_res_a  (fa,e3d,fwdfa,dfa);       // forward fp res to fp a
        mux2x32 fwd_f_res_b  (fb,e3d,fwdfb,dfb);       // forward fp res to fp b
        fpu fp_unit (dfa,dfb,fc,wf,rd,1'b1,clk,clrn,e3d,wd,wn,ww,
                     stall_div_sqrt,e1n,e1w,e2n,e2w,e3n,e3w,
                     e1c,e2c,e3c,cnt_div,cnt_sqrt,e,1'b1,rm);
                         
        
        debug_control debug_if(.serial_tx(JB[2]), .serial_rx(JB[3]), .cpu_clk(clk),
            .sys_rstn(SI_Reset_N), .cpu_imem_addr(dbg_imem_addr), 
            .cpu_debug_to_imem_data(dbg_imem_din), .cpu_imem_to_debug_data(inst),
            .cpu_imem_we(dbg_imem_we), .cpu_imem_ce(dbg_imem_ce),
            .cpu_dmem_addr(dbg_dmem_addr), .cpu_debug_to_dmem_data(dbg_dmem_din),
            .cpu_imem_to_debug_data_ready(dbg_imem_ce & ~dbg_imem_we),
            .cpu_dmem_to_debug_data_ready(dbg_dmem_ce & ~dbg_dmem_we),
            .cpu_dmem_to_debug_data(mm), .cpu_dmem_we(dbg_dmem_we),
            .cpu_dmem_ce(dbg_dmem_ce), .cpu_resetn_cpu(dbg_resetn_cpu),
            .cpu_halt_cpu(dbg_halt_cpu));
    
         
        csr_unit_pipeline_adapter csr_unit_adapter(         
          .clk       (clk),
          .rstn     (clrn),
          // From pipeline
          .id_v(id_v),
          .ex_v(ex_v),
          .mem_v(mem_v), //TODO dont exist yet
          .wb_v(wb_v),   // TODO dont exist yet
          // EX stage (decode results already latched into EX regs)
          .csr_is_ex(ex_csr_en),              // 1 if EX instr is CSR op
          .csr_cmd_ex(efunc3), // funct3 (001/010/011)
          .csr_addr_ex(ex_csr_addr),
          .csr_wdata_ex(csr_wdata_ex),           // RS1 value after forwarding
          .is_mret_ex(is_mret_ex),        
          // Commit/WB control
          .csr_is_wb(wb_csr_en),
          .csr_cmd_wb(wfunc3),
          .csr_addr_wb(wb_csr_addr),
          .csr_wdata_wb(csr_wdata_wb),
          .is_mret_wb(is_mret_wb),

          // Commit mask
          .kill_wb(kill_wb),                // squash at commit

  // ---- Trap/redirect interface ----
          .take_trap_raw(take_trap),    // 1-cycle request from arbiter (ID/EX/MEM)
          .take_trap(take_trap_pc),        // output gated per A1 (redirect happens here) used for PC

          
          // Trap arbiter (immediate)
          .trap_set(take_trap_r),          // from CU
          .trap_cause(trap_cause),        // from CU
          .trap_pc(trap_pc_source_r),           // save PC
          .trap_vector(trap_vector),        
          // External interrupt handshake
          .intr_synced(intr_synced),
          .cu_intr_ack(reset_mip11),
        
          // Read result for EX (to write back to rd)
          .csr_rdata_ex(csr_rdata_ex),
        
          // Exposed CSRs to the core
          .mstatus(mstatus),
          .mie(mie),
          .mip(mip),
          .mepc(mepc),
          .mcause(mcause), 
          .mtvec(mtvec)
        );

            
        
        // valid signal to avoid initial read unimplemented instruction and flush and stall logic
        always @(posedge clk or negedge clrn) begin
          if (!clrn) begin
            if_v  <= 1'b0; 
            id_v  <= 1'b0; 
            ex_v  <= 1'b0; 
            mem_v <= 1'b0; 
            wb_v  <= 1'b0;
          end else begin
            // IF gets valid when you actually start fetching (can be 1 in the first cycle after reset)
            if (!stall_if)  if_v  <= 1'b1;        
            if (!stall_id)  id_v  <=  if_v   & ~flush_id;
            if (!stall_ex)  ex_v  <=  id_v   & ~flush_ex;
            if (!stall_mem) mem_v <= (flush_mem | bubble_mem) ? 1'b0 : ex_v;
            if (!stall_wb)  wb_v  <=  mem_v  & ~flush_wb;
          end
        end
        
        

        
        //8888888888888888888888888888888888888888888   I N T E R R U P T       L O G I C       S T A R T            88888888888888888888888888
            //async interrupt control
        // very simple edge-detect + latch
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
                reset_mip11 <= intr_id;
                // 2) Edge detect: set pending on rising edge
                if (sync0 & ~sync1)
                    intr_synced <= 1'b1;
                // 3) Clear pending when core acks
                else if (intr_id)
                    intr_synced <= 1'b0;
                // else retain previous pending
            end
        end    
            
            
        
            
            
            
            // Maskable interrupt: only taken when MIE=1, MEIE=1, and pending  
        wire int_int = mstatus[3]  // MIE bit in mstatus
                 & mie[11]     // MEIE bit in mie CSR
                 & mip[11];
        assign intr_ack = int_int;
        
        assign trap_in_mem = trap_mem_v;
        assign trap_in_ex = ~trap_in_mem & trap_ex_v;
        assign trap_in_id = ~trap_in_mem & ~trap_in_ex & trap_id_v;
        assign trap_in_if = ~trap_in_mem & ~trap_in_ex & ~trap_in_id & trap_if_v;
        assign take_trap = trap_in_mem | trap_in_ex | trap_in_id | trap_in_if;



        assign bubble_mem = take_trap & trap_in_ex;
        assign kill_wb = ~wb_v;
        
        // 2) Synchronous exceptions (always trap when they occur)
        //---------------------------------------------------------------------
        
        
    // create a WB-committed mret pulse
    wire mret_pc = wb_v & is_mret_wb & ~kill_wb;  // same gating as adapter's mret_to_unit
    
    // priority encode: exception (2'b10) else mret (2'b01) else normal (2'b00)
    assign selpc =
        take_trap_pc ? 2'b10 :   // trap → mtvec
        mret_pc      ? 2'b01 :   // mret → mepc (after mepc write has committed)
                       2'b00 ;   // normal
    
       
        
        // oldest-wins select
        assign trap_mem_v = 0;  // comment out if trap_mem_v exist
        assign trap_if_v = 0;  // comment out if trap_id_v exist
        
        always @* begin
          take_trap_r       = 1'b0;
          cause_low_sel_r   = 4'd0;
          is_intr_sel_r     = 1'b0;
          trap_pc_source_r  = 32'h0;
          
        
          if (trap_mem_v) begin
            take_trap_r      = 1'b1;
            cause_low_sel_r  = cause_mem;
            is_intr_sel_r    = intr_mem;
            trap_pc_source_r = mpc;
          end else if (trap_ex_v) begin
            take_trap_r      = 1'b1;
            cause_low_sel_r  = cause_ex;
            is_intr_sel_r    = intr_ex;
            trap_pc_source_r = epc;
          end else if (trap_id_v) begin
            take_trap_r      = 1'b1;
            cause_low_sel_r  = cause_id;
            is_intr_sel_r    = intr_id;
            trap_pc_source_r = dpc;   // good choice for interrupts
          end else if (trap_if_v) begin
            take_trap_r      = 1'b1;
            cause_low_sel_r  = cause_if;
            is_intr_sel_r    = intr_if;
            trap_pc_source_r = pc;
          end
        end
        assign trap_cause = {26'b0, cause_low_sel_r, 2'b0};
        
      
//        assign flush_if  = cancel | ecancel | take_trap;                          // not used since IF doesnt have trap
        assign flush_id  = ecancel | (take_trap & ~trap_in_if);                    // flush ID unless trap is in IF only
        assign flush_ex  =           (take_trap & (trap_in_id | trap_in_if));      // EX is younger than ID/IF traps
        assign flush_mem =           (take_trap & trap_in_mem);                    // kill trapping instr in MEM
        assign flush_wb  = 1'b0;                                                   // usually unused

        
        //8888888888888888888888888888888888888888888   I N T E R R U P T       L O G I C       E N D           888888888888888888888888888888
        
        
        //
            // PC multiplexer now uses CSR's vector base:
        mux4x32 nxtpc (
          .a0(npc),                  // normal
          .a1(mepc),                 // mret returns here
          .a2(trap_vector),          // mtvec entry
          .a3(32'h00000000),         // unused
          .s(selpc),
          .y(n_pc) );
    
    
    
    
        ila_0 my_ila (
        .clk(SI_CLK100MHZ),                  // Clock used for ILA
        .probe0(inst),          // Probe for data bus
        .probe1(pc),       // Probe for address bus
        .probe2(clk),   // Probe for control signal 1
        .probe3(SI_Reset_N),    // Probe for control signal 2
        .probe4(lock),
        .probe5(counter),
        .probe6(IO_Switch),
        .probe7(IO_LED),
        .probe8(dbg_resetn_cpu),
        .probe9(dbg_imem_we),
        .probe10(dbg_imem_addr),
        .probe11(dbg_imem_din),
        .probe12(dbg_dmem_ce),
        .probe13(wpcir),
        .probe14(dbg_dmem_addr),
        .probe15(dbg_dmem_din),
        .probe16(dbg_halt_cpu),
        .probe17(IO_7SEGEN_N),
        .probe18(IO_7SEG_N),
        .probe19(npc) // input wire [31:0]  probe19
        );
    
    endmodule
