`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2024 09:31:37 AM
// Design Name: 
// Module Name: rv32m_fuseALU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module rv32m_fuseALU(
    input rv32m,
    input [31:0] a,
    input [31:0] b,
    input [4:0] rs1,
    input [4:0] rs2,
    input [2:0] func3,
    input clk,
    input clrn,
    output ready,
    output [31:0] c,
    output fuse,
    input estart_sdivide,estart_udivide
    );
    wire [31:0] c_mulmux,c_remmux,c_remumux;
    wire [4:0] s_rs1,s_rs2;
    wire [2:0] s_func3;
    wire [31:0] c_mul,c_mulh,c_mulhsu,c_mulu,c_div,c_divu,c_rem,c_remu;
    wire [31:0] fc_mul,fc_rem,fc_remu;
    wire mul_fuse, rem_fuse;
    wire s_rv32m;
    
    // Register Logic
    
    dffe fuserv32m (rv32m,clk,clrn,ready,s_rv32m);
    dffe5 fusers1 (rs1,clk,clrn,ready,s_rs1);
    dffe5 fusers2 (rs2,clk,clrn,ready,s_rs2);
    dffe3 fusefunc3 (func3,clk,clrn,ready,s_func3);
    
    dffe32 fusec_mul (c_mul,clk,clrn,rv32m&ready,c_mulmux);
    dffe32 fusec_rem (c_rem,clk,clrn,rv32m&ready,c_remmux);
    dffe32 fusec_remu (c_remu,clk,clrn,rv32m&ready,c_remumux);
    
    mux2x32 fusemuxmul (c_mul,c_mulmux,fuse,fc_mul);
    mux2x32 fusemuxrem (c_rem,c_remmux,fuse,fc_rem);
    mux2x32 fusemuxmreu (c_remu,c_remumux,fuse,fc_remu);
    
    mux8x32 rv32mux  (fc_mul,c_mulh,c_mulhsu,c_mulu,c_div,c_divu,fc_rem,fc_remu,func3,c);
    
    
    
 
    mul_div mul_div(
    .a(a),
    .b(b),
    .rs1(rs1),
    .rs2(rs2),
    .func3(func3),
    .clk(clk),
    .clrn(clrn),
    .rv32m(rv32m),
    .c_mulh(c_mulh),
    .c_mulhsu(c_mulhsu),
    .c_mulu(c_mulu),
    .c_div(c_div),
    .c_divu(c_divu),
    .c_mul(c_mul),
    .c_rem(c_rem),
    .c_remu(c_remu),
    .ready(ready),
    .fuse(fuse),
    .estart_sdivide(estart_sdivide),
    .estart_udivide(estart_udivide));

endmodule
