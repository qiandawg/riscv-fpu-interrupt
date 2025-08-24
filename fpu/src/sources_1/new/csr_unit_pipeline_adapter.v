// Glue so a single-cycle csr_unit works in a 5-stage pipeline (Option A)
// - EX reads CSR for rd
// - WB commits CSR writes/MRET (precise state)
// - If WB writes mtvec in the same cycle a trap is taken, delay redirect by 1 cycle
module csr_unit_pipeline_adapter (
  input  wire        clk, rstn,

  // valids (only wb_v used here)
  input  wire        id_v, ex_v, mem_v, wb_v,

  // ---------- EX: read path ----------
  input  wire        csr_is_ex,
  input  wire [2:0]  csr_cmd_ex,       // not used here, but handy for hazards
  input  wire [11:0] csr_addr_ex,
  input  wire [31:0] csr_wdata_ex,     // not used here, for debug
  input  wire        is_mret_ex,       // not used here

  // ---------- WB: commit path ----------
  input  wire        csr_is_wb,
  input  wire [2:0]  csr_cmd_wb,
  input  wire [11:0] csr_addr_wb,
  input  wire [31:0] csr_wdata_wb,
  input  wire        is_mret_wb,

  // Commit mask
  input  wire        kill_wb,

  // ---- Trap/redirect interface ----
  input  wire        take_trap_raw,    // 1-cycle request from arbiter (ID/EX/MEM)
  output wire        take_trap,        // gated per A1 (redirect happens here)
  input  wire        trap_set,         // still pass to CSR unit immediately
  input  wire [31:0] trap_cause,
  input  wire [31:0] trap_pc,
  output wire [31:0] trap_vector,      // current mtvec from csr_unit

  // External interrupt handshake
  input  wire        intr_synced,
  input  wire        cu_intr_ack,

  // EX read result (to rd mux)
  output wire [31:0] csr_rdata_ex,

  // Exposed CSRs (as needed by core)
  output wire [31:0] mstatus, mie, mip, mepc, mcause, mtvec
);

  // ---------------- Commit-ordered control (Option A) ----------------
  wire csr_commit = wb_v & csr_is_wb & ~kill_wb;

  // Address to csr_unit:
  //  - use WB address on the cycle we commit a write (so write hits right CSR)
  //  - otherwise use EX address so EX sees a correct read
  wire [11:0] csr_addr_to_unit  = csr_commit ? csr_addr_wb  : csr_addr_ex;
  wire [2:0]  csr_cmd_to_unit   = csr_cmd_wb;         // only meaningful on commit
  wire [31:0] csr_wdata_to_unit = csr_wdata_wb;       // only used on commit
  wire        csr_en_to_unit    = csr_commit;
  wire        mret_to_unit      = wb_v & is_mret_wb & ~kill_wb;

  // ---------------- EX read with WB→EX bypass ----------------
  wire [31:0] csr_rdata_unit; // combinational read from csr_unit

  // If committing a CSR write and EX reads the same CSR this cycle, show EX the new data.
  wire wb_writing_same_csr = csr_commit && (csr_addr_ex == csr_addr_wb);
  assign csr_rdata_ex = wb_writing_same_csr ? csr_wdata_wb : csr_rdata_unit;

  // ---------------- A1 redirect gate (delay if WB writes mtvec now) ----------------
  localparam [11:0] CSR_MTVEC = 12'h305;

  wire wb_writes_mtvec_now = csr_commit && (csr_addr_wb == CSR_MTVEC);

  // If a trap request coincides with an mtvec commit, arm a one-cycle delayed redirect.
  reg delayed_trap;
  always @(posedge clk or negedge rstn) begin
    if (!rstn) delayed_trap <= 1'b0;
    else       delayed_trap <= wb_writes_mtvec_now ? take_trap_raw : 1'b0;
  end

  // Final redirect pulse: either immediate (no mtvec write) or the 1-cycle delayed one
  assign take_trap = (take_trap_raw & ~wb_writes_mtvec_now) | delayed_trap;
  
  


  // ---------------- Underlying single-cycle CSR unit ----------------
  csr_unit u (
    .clk        (clk),
    .reset      (rstn),

    .intr       (intr_synced),
    .cu_intr_ack(cu_intr_ack),

    .csr_en     (csr_en_to_unit),
    .csr_cmd    (csr_cmd_to_unit),
    .csr_addr   (csr_addr_to_unit),
    .csr_wdata  (csr_wdata_to_unit),
    .csr_rdata  (csr_rdata_unit),

    // CSR trap bookkeeping (can occur before redirect; Option A gates redirect only)
    .trap_set   (trap_set),
    .trap_cause (trap_cause),
    .trap_pc    (trap_pc),
    .trap_vector(trap_vector),
    .mret       (mret_to_unit),

    .mstatus_out(mstatus),
    .mie        (mie),
    .mip        (mip),
    .mepc_out   (mepc)
  );

  // Expose mtvec and tie off mcause unless your csr_unit already exposes it
  assign mtvec  = trap_vector;
  assign mcause = 32'h0000_0000; // TODO: connect csr_unit mcause_out if you expose it

endmodule
