`timescale 1ns/1ns

module sc_computer_tb;
    reg [8*8 :1] opcode;
    reg [8*28:1] desc;
    reg [8*5 :1] rs;
    reg [8*5 :1] rt;
    reg [8*5 :1] rd;
    reg [31:0] imm;
    reg [31:0] branch_addr;
    reg [31:0] jump_addr;

    wire [31:0] instr_spy = sc_computer_tb.sc_interrupt_sys.inst; // spying on the instruction
    wire [31:0] pc4_spy   = sc_computer_tb.sc_interrupt_sys.pc; // spying on the PC

    // Initialize the always block for instruction decoding
    always @* begin
        // Immediate calculations
        imm          = {instr_spy[31:12], 12'b0}; // LUI stores immediate in upper 20 bits, lower bits zeroed
        branch_addr  = {{20{instr_spy[31]}}, instr_spy[31:20], 2'b00}; // Branch address calculation
        jump_addr    = {pc4_spy[31:12], instr_spy[31:12], 2'b00}; // Jump address calculation

        // Default values
        opcode = "N/A     "; 
        desc = "undefined instruction"; 
        rs = "N/A  ";
        rt = "N/A  ";
        rd = "N/A  ";

        // Check for NOP first (special case of ADDI x0, x0, 0)
        if (instr_spy == 32'h00000013) begin
            opcode = "NOP     ";
            desc = "No operation (addi x0,x0,0)";
            rs = "x0   ";
            rt = "N/A  ";
            rd = "x0   ";
        end
        else begin
            // Instruction decoding logic
            case (instr_spy[6:0])
                7'b0110011: begin // R-type
                    case (instr_spy[14:12])
                        3'b000: begin // ADD/SUB
                            case (instr_spy[30])
                                1'b0: begin opcode = "ADD     "; desc = "rd = rs1 + rs2"; end
                                1'b1: begin opcode = "SUB     "; desc = "rd = rs1 - rs2"; end
                            endcase
                        end
                        3'b111: begin opcode = "AND     "; desc = "rd = rs1 & rs2"; end
                        3'b110: begin opcode = "OR      "; desc = "rd = rs1 | rs2"; end
                        3'b100: begin opcode = "XOR     "; desc = "rd = rs1 ^ rs2"; end
                        3'b001: begin opcode = "SLL     "; desc = "rd = rs1 << rs2[4:0]"; end
                        3'b101: begin 
                            case (instr_spy[30]) 
                                1'b0: begin opcode = "SRL     "; desc = "rd = rs1 >> rs2[4:0]"; end // Logical Shift Right
                                1'b1: begin opcode = "SRA     "; desc = "rd = rs1 >>> rs2[4:0]"; end // Arithmetic Shift Right
                            endcase
                        end
                        3'b010: begin opcode = "SLT     "; desc = "rd = (rs1 < rs2) ? 1 : 0"; end
                        3'b011: begin opcode = "SLTU    "; desc = "rd = (rs1 < rs2) ? 1 : 0"; end
                        default: begin opcode = "N/A     "; desc = "undefined R-type op"; end
                    endcase
                end 

                7'b0000011: begin // LOAD
                    case (instr_spy[14:12])
                        3'b000: begin opcode = "LB      "; desc = "rd = *(char*)(offset + rs1)"; end
                        3'b001: begin opcode = "LH      "; desc = "rd = *(short*)(offset + rs1)"; end
                        3'b010: begin opcode = "LW      "; desc = "rd = *(int*)(offset + rs1)"; end
                        3'b100: begin opcode = "LBU     "; desc = "rd = *(unsigned char*)(offset + rs1)"; end
                        3'b101: begin opcode = "LHU     "; desc = "rd = *(unsigned short*)(offset + rs1)"; end
                        default: begin opcode = "N/A     "; desc = "undefined LOAD op"; end
                    endcase
                end 

                7'b0010011: begin // I-type immediate
                    case (instr_spy[14:12])
                        3'b000: begin opcode = "ADDI    "; desc = "rd = rs1 + imm"; end
                        3'b111: begin opcode = "ANDI    "; desc = "rd = rs1 & imm"; end
                        3'b110: begin opcode = "ORI     "; desc = "rd = rs1 | imm"; end
                        3'b100: begin opcode = "XORI    "; desc = "rd = rs1 ^ imm"; end
                        3'b001: begin opcode = "SLLI    "; desc = "rd = rs1 << shamt"; end
                        3'b101: begin 
                            case (instr_spy[30])
                                1'b0: begin opcode = "SRLI    "; desc = "rd = rs1 >> shamt"; end // Logical Shift Right
                                1'b1: begin opcode = "SRAI    "; desc = "rd = rs1 >>> shamt"; end // Arithmetic Shift Right
                            endcase
                        end
                        3'b010: begin opcode = "SLTI    "; desc = "rd = (rs1 < imm) ? 1 : 0"; end
                        3'b011: begin opcode = "SLTIU   "; desc = "rd = (rs1 < imm) ? 1 : 0"; end
                        default: begin opcode = "N/A     "; desc = "undefined I-type op"; end
                    endcase
                end 

                7'b1100011: begin // BRANCH
                    case (instr_spy[14:12])
                        3'b000: begin opcode = "BEQ     "; desc = "if (rs1 == rs2) pc += offset * 4"; end
                        3'b001: begin opcode = "BNE     "; desc = "if (rs1 != rs2) pc += offset * 4"; end
                        3'b100: begin opcode = "BLT     "; desc = "if (rs1 < rs2) pc += offset * 4"; end
                        3'b101: begin opcode = "BGE     "; desc = "if (rs1 >= rs2) pc += offset * 4"; end
                        3'b110: begin opcode = "BLTU    "; desc = "if (rs1 < rs2) pc += offset * 4"; end
                        3'b111: begin opcode = "BGEU    "; desc = "if (rs1 >= rs2) pc += offset * 4"; end
                        default: begin opcode = "N/A     "; desc = "undefined branch op"; end
                    endcase
                end 

                7'b1101111: begin // JAL
                    opcode = "JAL     "; desc = "rd = pc + 4, pc = target address"; 
                end 

                7'b1100111: begin // JALR
                    opcode = "JALR    "; desc = "rd = pc + 4, pc = (rs1 + imm) & ~1"; 
                end 

                7'b0100011: begin // STORE
                    case (instr_spy[14:12])
                        3'b000: begin opcode = "SB      "; desc = "*(char*)(offset + rs1) = rs2"; end
                        3'b001: begin opcode = "SH      "; desc = "*(short*)(offset + rs1) = rs2"; end
                        3'b010: begin opcode = "SW      "; desc = "*(int*)(offset + rs1) = rs2"; end
                        default: begin opcode = "N/A     "; desc = "undefined STORE op"; end
                    endcase
                end 

                7'b0110111: begin // LUI
                    opcode = "LUI     "; 
                    desc = "rd = imm"; // Set immediate value without shifting, just loading upper bits
                end

                7'b1110011: begin // System instructions (CSR and privileged)
                    case (instr_spy[14:12])
                        3'b000: begin // Privileged instructions
                            case (instr_spy[31:20])
                                12'h000: begin opcode = "ECALL   "; desc = "Environment call"; end
                                12'h001: begin opcode = "EBREAK  "; desc = "Environment break"; end
                                12'h302: begin opcode = "MRET    "; desc = "Machine return from trap"; end
                                default: begin opcode = "N/A     "; desc = "undefined privileged op"; end
                            endcase
                        end                    
                        3'b001: begin opcode = "CSRRW   "; desc = "rd = CSR; CSR = rs1"; end
                        3'b010: begin opcode = "CSRRS   "; desc = "rd = CSR; CSR |= rs1"; end
                        3'b011: begin opcode = "CSRRC   "; desc = "rd = CSR; CSR &= ~rs1"; end
                        3'b101: begin opcode = "CSRRWI  "; desc = "rd = CSR; CSR = imm"; end
                        3'b110: begin opcode = "CSRRSI  "; desc = "rd = CSR; CSR |= imm"; end
                        3'b111: begin opcode = "CSRRCI  "; desc = "rd = CSR; CSR &= ~imm"; end
                        default: begin opcode = "N/A     "; desc = "undefined system op"; end
                    endcase
                end 
                
                default: begin
                    opcode = "N/A     "; desc = "undefined instruction"; // Catch all for undefined instructions
                end
            endcase
        end

        // Register Mapping for rs1
        case (instr_spy[19:15]) 
            5'd0  : rs = "x0   ";
            5'd1  : rs = "x1   "; 
            5'd2  : rs = "x2   "; 
            5'd3  : rs = "x3   "; 
            5'd4  : rs = "x4   "; 
            5'd5  : rs = "x5   ";
            5'd6  : rs = "x6   ";
            5'd7  : rs = "x7   ";
            5'd8  : rs = "x8   ";
            5'd9  : rs = "x9   ";
            5'd10 : rs = "x10  ";
            5'd11 : rs = "x11  ";
            5'd12 : rs = "x12  ";
            5'd13 : rs = "x13  ";
            5'd14 : rs = "x14  ";
            5'd15 : rs = "x15  ";
            5'd16 : rs = "x16  ";
            5'd17 : rs = "x17  ";
            5'd18 : rs = "x18  ";
            5'd19 : rs = "x19  ";
            5'd20 : rs = "x20  ";
            5'd21 : rs = "x21  ";
            5'd22 : rs = "x22  ";
            5'd23 : rs = "x23  ";
            5'd24 : rs = "x24  ";
            5'd25 : rs = "x25  ";
            5'd26 : rs = "x26  ";
            5'd27 : rs = "x27  ";
            5'd28 : rs = "x28  ";
            5'd29 : rs = "x29  ";
            5'd30 : rs = "x30  ";
            5'd31 : rs = "x31  ";
            default: rs = "N/A  ";
        endcase

        // Register Mapping for rt
        case (instr_spy[24:20]) 
            5'd0  : rt = "x0   ";
            5'd1  : rt = "x1   "; 
            5'd2  : rt = "x2   "; 
            5'd3  : rt = "x3   "; 
            5'd4  : rt = "x4   "; 
            5'd5  : rt = "x5   ";
            5'd6  : rt = "x6   ";
            5'd7  : rt = "x7   ";
            5'd8  : rt = "x8   ";
            5'd9  : rt = "x9   ";
            5'd10 : rt = "x10  ";
            5'd11 : rt = "x11  ";
            5'd12 : rt = "x12  ";
            5'd13 : rt = "x13  ";
            5'd14 : rt = "x14  ";
            5'd15 : rt = "x15  ";
            5'd16 : rt = "x16  ";
            5'd17 : rt = "x17  ";
            5'd18 : rt = "x18  ";
            5'd19 : rt = "x19  ";
            5'd20 : rt = "x20  ";
            5'd21 : rt = "x21  ";
            5'd22 : rt = "x22  ";
            5'd23 : rt = "x23  ";
            5'd24 : rt = "x24  ";
            5'd25 : rt = "x25  ";
            5'd26 : rt = "x26  ";
            5'd27 : rt = "x27  ";
            5'd28 : rt = "x28  ";
            5'd29 : rt = "x29  ";
            5'd30 : rt = "x30  ";
            5'd31 : rt = "x31  ";
            default: rt = "N/A  ";
        endcase

        // Register Mapping for rd
        case (instr_spy[11:7]) 
            5'd0  : rd = "x0   ";
            5'd1  : rd = "x1   "; 
            5'd2  : rd = "x2   "; 
            5'd3  : rd = "x3   "; 
            5'd4  : rd = "x4   "; 
            5'd5  : rd = "x5   ";
            5'd6  : rd = "x6   ";
            5'd7  : rd = "x7   ";
            5'd8  : rd = "x8   ";
            5'd9  : rd = "x9   ";
            5'd10 : rd = "x10  ";
            5'd11 : rd = "x11  ";
            5'd12 : rd = "x12  ";
            5'd13 : rd = "x13  ";
            5'd14 : rd = "x14  ";
            5'd15 : rd = "x15  ";
            5'd16 : rd = "x16  ";
            5'd17 : rd = "x17  ";
            5'd18 : rd = "x18  ";
            5'd19 : rd = "x19  ";
            5'd20 : rd = "x20  ";
            5'd21 : rd = "x21  ";
            5'd22 : rd = "x22  ";
            5'd23 : rd = "x23  ";
            5'd24 : rd = "x24  ";
            5'd25 : rd = "x25  ";
            5'd26 : rd = "x26  ";
            5'd27 : rd = "x27  ";
            5'd28 : rd = "x28  ";
            5'd29 : rd = "x29  ";
            5'd30 : rd = "x30  ";
            5'd31 : rd = "x31  ";
            default: rd = "N/A  ";
        endcase
    end // always @*

    `include "mfp_ahb_const.vh" 

    // Declare the clocks and control signals
    reg clk, clrn, memclk, intr;
    wire [31:0] inst, pc, aluout, memout;
    reg [`MFP_N_SW-1 :0] IO_Switch;
    reg [`MFP_N_PB-1 :0] IO_PB;
    wire [`MFP_N_LED-1:0] IO_LED;
    wire [4:1] JA;
    wire [4:1] JB;
    wire [7:0] AN;
    wire CA, CB, CC, CD, CE, CF, CG;
    wire JC_2,
   JC_4,
JC_1,
 JC_7,
 JC_8,
JC_9,
 JC_10;
    wire [4:1] JD;
    reg UART_RX;

    // Module instantiation
    sc_interrupt_sys sc_interrupt_sys(
        .SI_Reset_N(clrn),
        .SI_ClkIn(clk),
        .inst(inst),
        .pc(pc),
        .aluout(aluout),
        .memout(memout),
        .memclk(memclk),
        .intr(intr),
        .inta(inta),
        .IO_Switch(IO_Switch),
        .IO_PB(IO_PB),
        .IO_LED(IO_LED),
        .IO_7SEGEN_N(AN),
        .IO_7SEG_N({CA, CB, CC, CD, CE, CF, CG}),
        .IO_BUZZ(),
        .IO_RGB_SPI_MOSI(JC_2),
        .IO_RGB_SPI_SCK(JC_4),
        .IO_RGB_SPI_CS(JC_1),
        .IO_RGB_DC(JC_7),
        .IO_RGB_RST(JC_8),
        .IO_RGB_VCC_EN(JC_9),
        .IO_RGB_PEN(JC_10),
        .IO_CS(JA[1]),
        .IO_SCK(JA[4]),
        .IO_SDO(JA[3]),
        .UART_RX(UART_RX),
        .JB(JB)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #10 clk = ~clk; // Clock cycle of 20ns
    end

    // Memory clock generation
    initial begin
        memclk = 0;
        forever #5 memclk = ~memclk; // Memory clock cycle of 10ns
    end

    // Reset signal
    initial begin
        clrn = 0;
        #5 clrn = 1; // Set reset low for 5ns, then release
    end
    
    initial begin
        intr = 0;
        #1500 intr = ~intr; // toggle intr every 1005ms
        #1000 intr = ~intr; // toggle intr every 1005ms
    end
    
    // IO Switch simulation
    initial begin
        IO_Switch = 16'haaaa;
        forever 
            #100 IO_Switch = ~IO_Switch; // Toggle switch state every 1ms
    end 

endmodule