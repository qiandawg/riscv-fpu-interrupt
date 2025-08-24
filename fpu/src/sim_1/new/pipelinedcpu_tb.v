/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/


`timescale 1ns/1ns
`include "mfp_ahb_const.vh" 
module pipelinedcpu_tb;
    reg [8*8 :1] opcode; 
    reg [8*28:1] desc;
    reg [8*5 :1] rs;
    reg [8*5 :1] rt;
    reg [8*5 :1] rd;
    reg [31:0] imm;
    reg [31:0] branch_addr;
    reg [31:0] jump_addr;

    wire [31:0] instr_spy = pipelinedcpu_tb.cpu.inst; // spying on the instruction
    wire [31:0] pc4_spy   = pipelinedcpu_tb.cpu.pc; // spying on the PC

    reg         clk, clrn;

    // New: system/board-level
    reg         SI_CLK100MHZ;
    wire        lock = 1'b1;          // tie high in TB
    reg  [26:0] counter;
    reg         intr;
    
    // DUT observable buses
    wire [31:0] pc, inst, eal, mal, wres;
    
    // Extra DUT outputs you're already instantiating
    wire [31:0] e3d, wd;
    wire  [4:0] e1n, e2n, e3n, wn;
    wire        ww, stl_lw, stl_fp, stl_lwc1, stl_swc1, stl;
    wire        e;                    // unused but declare it
    
    // IOs & headers
    reg  [`MFP_N_SW-1 :0] IO_Switch;
    reg  [`MFP_N_PB-1 :0] IO_PB;
    wire [`MFP_N_LED-1:0] IO_LED;
    wire [7:0]            IO_7SEGEN_N;
    wire [6:0]            IO_7SEG_N;
    wire                  IO_BUZZ;
    wire [10:1]           JC;
    wire [4:1]            JA;
    wire [8:1]            JB;
    
    
    
    // UART
    reg                   UART_RX;    // drive from TB
    // If pl_computer expects IO_SDO (input), drive it via JA[3]:
    assign JA[3] = 1'b0;              // or a task/driver later


    // Initialize the always block for instruction decoding
    always @* begin
        // Immediate calculations
        imm          = {instr_spy[31:12], 12'b0}; // LUI stores immediate in upper 20 bits, lower bits zeroed
        branch_addr  = {{20{instr_spy[31]}}, instr_spy[31:20], 2'b00}; // Branch address calculation
        jump_addr    = {pc4_spy[31:12], instr_spy[31:12], 2'b00}; // Jump address calculation

        // Default values
        opcode = "N/A  "; 
        desc = "undefined instruction"; 
        rs = "N/A  ";
        rt = "N/A  ";
        rd = "N/A  ";

        // Instruction decoding logic
        case (instr_spy[6:0])
            7'b0110011: begin // R-type
                case (instr_spy[14:12])
                    3'b000: begin // ADD/SUB
                        case (instr_spy[30])
                            1'b0: begin opcode = "ADD  "; desc = "rd = rs1 + rs2"; end
                            1'b1: begin opcode = "SUB  "; desc = "rd = rs1 - rs2"; end
                        endcase
                    end
                    3'b111: begin opcode = "AND  "; desc = "rd = rs1 & rs2"; end
                    3'b110: begin opcode = "OR   "; desc = "rd = rs1 | rs2"; end
                    3'b100: begin opcode = "XOR  "; desc = "rd = rs1 ^ rs2"; end
                    3'b001: begin opcode = "SLL  "; desc = "rd = rs1 << rs2[4:0]"; end
                    3'b101: begin 
                        case (instr_spy[30]) 
                            1'b0: begin opcode = "SRL  "; desc = "rd = rs1 >> rs2[4:0]"; end // Logical Shift Right
                            1'b1: begin opcode = "SRA  "; desc = "rd = rs1 >>> rs2[4:0]"; end // Arithmetic Shift Right
                        endcase
                    end
                    3'b010: begin opcode = "SLT  "; desc = "rd = (rs1 < rs2) ? 1 : 0"; end
                    3'b011: begin opcode = "SLTU "; desc = "rd = (rs1 < rs2) ? 1 : 0"; end
                    default: begin opcode = "N/A  "; desc = "undefined R-type op"; end
                endcase
            end 

            7'b0000111: begin // LOAD (Including FLW)
                case (instr_spy[14:12])
                    3'b010: begin opcode = "FLW  "; desc = "rd = *(float*)(offset + rs1)"; end
                    3'b000: begin opcode = "LB   "; desc = "rd = *(char*)(offset + rs1)"; end
                    3'b001: begin opcode = "LH   "; desc = "rd = *(short*)(offset + rs1)"; end
                    3'b010: begin opcode = "LW   "; desc = "rd = *(int*)(offset + rs1)"; end
                    3'b100: begin opcode = "LBU  "; desc = "rd = *(unsigned char*)(offset + rs1)"; end
                    3'b101: begin opcode = "LHU  "; desc = "rd = *(unsigned short*)(offset + rs1)"; end
                    default: begin opcode = "N/A  "; desc = "undefined LOAD op"; end
                endcase
            end 

            7'b0010011: begin // I-type immediate
                case (instr_spy[14:12])
                    3'b000: begin opcode = "ADDI "; desc = "rd = rs1 + imm"; end
                    3'b111: begin opcode = "ANDI "; desc = "rd = rs1 & imm"; end
                    3'b110: begin opcode = "ORI  "; desc = "rd = rs1 | imm"; end
                    3'b100: begin opcode = "XORI "; desc = "rd = rs1 ^ imm"; end
                    3'b001: begin opcode = "SLLI "; desc = "rd = rs1 << shamt"; end
                    3'b101: begin 
                        case (instr_spy[30])
                            1'b0: begin opcode = "SRLI "; desc = "rd = rs1 >> shamt"; end // Logical Shift Right
                            1'b1: begin opcode = "SRAI "; desc = "rd = rs1 >>> shamt"; end // Arithmetic Shift Right
                        endcase
                    end
                    3'b010: begin opcode = "SLTI "; desc = "rd = (rs1 < imm) ? 1 : 0"; end
                    3'b011: begin opcode = "SLTIU"; desc = "rd = (rs1 < imm) ? 1 : 0"; end
                    default: begin opcode = "N/A  "; desc = "undefined I-type op"; end
                endcase
            end 

            7'b1100011: begin // BRANCH
                case (instr_spy[14:12])
                    3'b000: begin opcode = "BEQ  "; desc = "if (rs1 == rs2) pc += offset * 4"; end
                    3'b001: begin opcode = "BNE  "; desc = "if (rs1 != rs2) pc += offset * 4"; end
                    3'b100: begin opcode = "BLT  "; desc = "if (rs1 < rs2) pc += offset * 4"; end
                    3'b101: begin opcode = "BGE  "; desc = "if (rs1 >= rs2) pc += offset * 4"; end
                    3'b110: begin opcode = "BLTU "; desc = "if (rs1 < rs2) pc += offset * 4"; end
                    3'b111: begin opcode = "BGEU "; desc = "if (rs1 >= rs2) pc += offset * 4"; end
                    default: begin opcode = "N/A  "; desc = "undefined branch op"; end
                endcase
            end 

            7'b0100011: begin // SW,SH,SB
                case (instr_spy[14:12])
                    3'b000: begin opcode = "SB  "; desc = "*(char*)(offset + rs1) = rs2"; end
                    3'b001: begin opcode = "SH  "; desc = "*(short*)(offset + rs1) = rs2"; end
                    3'b010: begin opcode = "SW  "; desc = "*(int*)(offset + rs1) = rs2"; end

                    default: begin opcode = "N/A  "; desc = "undefined branch op"; end
                endcase
            end 
            
                        
            7'b1101111: begin // JAL
                opcode = "JAL  "; desc = "rd = pc + 4, pc = target address"; 
            end 

            7'b1100111: begin // JALR
                opcode = "JALR "; desc = "rd = pc + 4, pc = (rs1 + imm) & ~1"; 
            end 

            7'b0100111: begin // STORE (Including FSW)
                case (instr_spy[14:12])
                    3'b010: begin opcode = "FSW  "; desc = "*(float*)(offset + rs1) = rs2"; end
                    3'b000: begin opcode = "SB   "; desc = "*(char*)(offset + rs1) = rs2"; end
                    3'b001: begin opcode = "SH   "; desc = "*(short*)(offset + rs1) = rs2"; end
                    3'b010: begin opcode = "SW   "; desc = "*(int*)(offset + rs1) = rs2"; end
                    default: begin opcode = "N/A  "; desc = "undefined STORE op"; end
                endcase
            end 

            7'b0110111: begin // LUI
                opcode = "LUI  "; 
                desc = "rd = imm"; // Set immediate value without shifting, just loading upper bits
            end
            
            // RV32M Instructions (Multiply/Divide)
            7'b0110011: begin
                if (instr_spy[31:25] == 7'b0000001) begin // Specific RV32M funct7 check
                    case (instr_spy[14:12])
                        3'b000: begin opcode = "MUL  "; desc = "rd = rs1 * rs2"; end
                        3'b001: begin opcode = "MULH "; desc = "rd = (rs1 * rs2) >> 32 (signed high)"; end
                        3'b010: begin opcode = "MULHSU"; desc = "rd = (rs1 * rs2) >> 32 (signed/unsigned)"; end
                        3'b011: begin opcode = "MULHU"; desc = "rd = (rs1 * rs2) >> 32 (unsigned high)"; end
                        3'b100: begin opcode = "DIV  "; desc = "rd = rs1 / rs2"; end
                        3'b101: begin opcode = "DIVU "; desc = "rd = rs1 / rs2 (unsigned)"; end
                        3'b110: begin opcode = "REM  "; desc = "rd = rs1 % rs2"; end
                        3'b111: begin opcode = "REMU "; desc = "rd = rs1 % rs2 (unsigned)"; end
                        default: begin opcode = "N/A  "; desc = "undefined M-type op"; end
                    endcase
                end
            end

            // RV32F Instructions (Floating-point Arithmetic, including FSQRT.S)
            7'b1010011: begin // Floating Point Ops (RV32F)
    case (instr_spy[31:25]) // funct7 for RV32F operations
        7'b0000000: begin opcode = "FADD.S "; desc = "rd = rs1 + rs2 (FP single precision)"; end // Add
        7'b0000100: begin opcode = "FSUB.S "; desc = "rd = rs1 - rs2 (FP single precision)"; end // Subtract
        7'b0001000: begin opcode = "FMUL.S "; desc = "rd = rs1 * rs2 (FP single precision)"; end // Multiply
        7'b0001100: begin opcode = "FDIV.S "; desc = "rd = rs1 / rs2 (FP single precision)"; end // Divide
        7'b0101100: begin opcode = "FSQRT.S"; desc = "rd = sqrt(rs1) (FP single precision)"; end // Square Root
        default: begin opcode = "N/A     "; desc = "undefined FP op"; end
    endcase

    // Rounding mode (instr_spy[14:12]) is also important but does not change the opcode itself
    // For debugging purposes, we can include rounding mode information
    case (instr_spy[14:12])
        3'b000: desc = {desc, ", rounding=RNE (Round to Nearest)"}; // Round to Nearest, ties to Even
        3'b001: desc = {desc, ", rounding=RTZ (Round toward Zero)"}; // Round toward Zero
        3'b010: desc = {desc, ", rounding=RDN (Round Downward)"};    // Round Downward
        3'b011: desc = {desc, ", rounding=RUP (Round Upward)"};      // Round Upward
        3'b100: desc = {desc, ", rounding=RMM (Round to Max Magnitude)"}; // Round to Maximum Magnitude
        default: desc = {desc, ", rounding=N/A"}; // Invalid rounding mode
    endcase
end
          
            default: begin
                opcode = "N/A  "; desc = "undefined instruction"; // Catch all for undefined instructions
            end
        endcase

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







   
      pl_computer cpu( .SI_ClkIn(clk),
                    .SI_Reset_N(clrn),                  
                    .pc(pc),
                    .inst(inst),
                    .eal(eal),
                    .mal(mal),
                    .wres(wres),
                    .e3d(e3d),
                    .wd(wd),
                    .e1n(e1n),
                    .e2n(e2n),
                    .e3n(e3n),
                    .wn(wn),
                    .ww(ww),
                    .stl_lw(stl_lw),
                    .stl_fp(stl_fp),
                    .stl_lwc1(stl_lwc1),
                    .stl_swc1(stl_swc1),
                    .stl(stl),
                    .IO_Switch(IO_Switch),
                    .IO_PB(IO_PB),
                    .IO_LED(IO_LED),
                    .IO_7SEGEN_N(IO_7SEGEN_N),
                    .IO_7SEG_N(IO_7SEG_N), 
                    .IO_BUZZ(IO_BUZZ),
                    .IO_RGB_SPI_MOSI(JC[2]),
                    .IO_RGB_SPI_SCK(JC[4]),
                    .IO_RGB_SPI_CS(JC[1]),
                    .IO_RGB_DC(JC[7]),
                    .IO_RGB_RST(JC[8]),
                    .IO_RGB_VCC_EN(JC[9]),
                    .IO_RGB_PEN(JC[10]), 
                    .IO_CS(JA[1]),
                    .IO_SCK(JA[4]),
                    .IO_SDO(JA[3]),
                    .UART_RX(UART_TXD_IN),
                    .JB(JB));                    
               

    initial begin
               clk  = 0;
         forever
             #10 clk = ~ clk;
     end
 
      initial begin
         clrn  = 0;
         forever
             #20 clrn  = 1;
     end
     
     initial begin
         IO_Switch = 0;
         forever 
           #70 IO_Switch = 16'h135a;
      end 

endmodule
/*
   0 -  28
  92 - 120
 120 - 148
*/