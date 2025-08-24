module csr_unit (
    input  wire         clk,
    input  wire         reset,
    
    // External interrupt request (for pending bit update)
    input  wire         intr,
    input wire          cu_intr_ack,

    // CSR instruction interface
    input  wire         csr_en,        // high when instruction is CSR op
    input  wire [2:0]   csr_cmd,       // funct3: 001=CSRRW, 010=CSRRS, 011=CSRRC
    input  wire [11:0]  csr_addr,      // instr[31:20]
    input  wire [31:0]  csr_wdata,     // rs1 value for write
    output reg  [31:0]  csr_rdata,     // data to write back to rd

    // Trap/interrupt interface
    input  wire         trap_set,      // asserted when entering a trap
    input  wire [31:0]  trap_cause,    // cause value to write into mcause
    input  wire [31:0]  trap_pc,       // current PC (to save in mepc)
    output wire [31:0]  trap_vector,   // mtvec value (vector base)
    input  wire         mret,          // asserted when mret executes

    // For core datapath
    output reg  [31:0]  mstatus_out,
    output reg  [31:0]  mie,
    output reg  [31:0]  mip,
    output reg  [31:0]  mepc_out
    
);

    // Machine-level CSRs
    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mip;

    // Expose trap vector (mtvec)
    assign trap_vector = mtvec;

    // ------------------------------------------------------------------
    // Sequential CSR updates: trap entry, mret, CSR writes, pending bits
    // ------------------------------------------------------------------
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            mstatus <= 32'b0;
            mie     <= 32'b0;
            mtvec   <= 32'b0;
            mepc    <= 32'b0;
            mcause  <= 32'b0;
            mip     <= 32'b0;
        end else begin
            // 1) Trap entry takes highest priority
            if (trap_set) begin
                mepc   <= trap_pc;
                mcause <= trap_cause;
                // Push MIE -> MPIE (bit7) and disable MIE (bit3)
                mstatus[7] <= mstatus[3];
                mstatus[3] <= 1'b0;
                // Clear external interrupt pending bit
                mip[11]    <= 1'b0;

            // 2) mret next
            end else if (mret) begin
                mstatus[3] <= mstatus[7];
                mstatus[7] <= 1'b1;

            // 3) CSR instruction writes
            end else if (csr_en) begin
                case (csr_addr)
                    12'h300: begin // mstatus
                        case (csr_cmd)
                            3'b001: mstatus <= csr_wdata;                   // CSRRW
                            3'b010: mstatus <= mstatus | csr_wdata;         // CSRRS
                            3'b011: mstatus <= mstatus & ~csr_wdata;        // CSRRC
                        endcase
                    end
                    12'h304: begin // mie
                        case (csr_cmd)
                            3'b001: mie <= csr_wdata;
                            3'b010: mie <= mie | csr_wdata;
                            3'b011: mie <= mie & ~csr_wdata;
                        endcase
                    end
                    12'h305: begin // mtvec
                        case (csr_cmd)
                            3'b001: mtvec <= csr_wdata;
                            3'b010: mtvec <= mtvec | csr_wdata;
                            3'b011: mtvec <= mtvec & ~csr_wdata;
                        endcase
                    end
                    12'h341: begin // mepc
                        case (csr_cmd)
                            3'b001: mepc <= csr_wdata;
                            3'b010: mepc <= mepc | csr_wdata;
                            3'b011: mepc <= mepc & ~csr_wdata;
                        endcase
                    end
                    12'h342: begin // mcause
                        case (csr_cmd)
                            3'b001: mcause <= csr_wdata;
                            3'b010: mcause <= mcause | csr_wdata;
                            3'b011: mcause <= mcause & ~csr_wdata;
                        endcase
                    end
                    12'h344: begin // mip
                        case (csr_cmd)
                            3'b001: mip <= csr_wdata;
                            3'b010: mip <= mip | csr_wdata;
                            3'b011: mip <= mip & ~csr_wdata;
                        endcase
                    end
                endcase
            end

         // 4) Update external interrupt pending bit (MEIP)
        if (intr) begin
            // Set mip[11] on rising edge of intr (pulse)
            mip[11] <= 1'b1;
        end else if (cu_intr_ack) begin
            // Clear mip[11] when the core acknowledges the interrupt
            mip[11] <= 1'b0;
        // If neither intr_synced nor cu_intr_ack, mip[11] retains its value
        end
    end
    end

    // ------------------------------------------------------------------
    // Combinational CSR read
    // ------------------------------------------------------------------
    always @(*) begin
        case (csr_addr)
            12'h300: csr_rdata = mstatus;
            12'h304: csr_rdata = mie;
            12'h305: csr_rdata = mtvec;
            12'h341: csr_rdata = mepc;
            12'h342: csr_rdata = mcause;
            12'h344: csr_rdata = mip;
            default: csr_rdata = 32'b0;
        endcase
    end

    // ------------------------------------------------------------------
    // Expose outputs for rest of CPU
    // ------------------------------------------------------------------
    always @(*) begin
        mstatus_out = mstatus;
        mepc_out    = mepc;
    end

endmodule
