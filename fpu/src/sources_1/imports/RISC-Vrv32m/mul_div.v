`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/11/2024 02:05:48 AM
// Design Name: 
// Module Name: mul_div
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


module mul_div(
    input [31:0] a,
    input [31:0] b,
    input [4:0] rs1,
    input [4:0] rs2,
    input [2:0] func3,
    input clk,
    input clrn,
    input rv32m,
    output [31:0] c_mulh,
    output [31:0] c_mulhsu,
    output [31:0] c_mulu,
    output [31:0] c_div,
    output [31:0] c_divu,
    output [31:0] c_mul,
    output [31:0] c_rem,
    output [31:0] c_remu,
    output ready,
    output fuse,
    input estart_sdivide,
    input estart_udivide);

    wire [31:0] lower_product;
    wire [31:0] upper_product;
    wire [31:0] quotient_signed,quotient_unsigned;
    wire [31:0] rem_signed,rem_unsigned;
    wire ready_signed;
    wire ready_unsigned;
    wire error_unsigned;
    wire error_signed;
    wire [63:0] product1,product2,product3;
    
    //wire ready;
    
//    wire start_sdivide,start_udivide;
    
 
        
//    Start_Div Start_Div(
//       .clk(clk),
//       .func3(func3),
//       .fuse(fuse),
//       .rv32m(rv32m),
//       .start_sdivide(start_sdivide),
//       .start_udivide(start_udivide));
    
    multiplysignedsigned multiplysignedsigned(
       .signed_data1(a),
       .signed_data2(b),
       .result(product1));
       
    multiplysignedunsigned multiplysignedunsigned(
       .signed_data1(a),
       .unsigned_data2(b),
       .result(product2));
          
    multiplyunsignedunsigned multiplyunsignedunsigned(
       .unsigned_data1(a),
       .unsigned_data2(b),
       .result(product3));
       
    UDivide UDivide(
       .A(a),
       .B(b),
       .reset(clrn),
       .start(estart_udivide),
       .clk(clk),
       .D(quotient_unsigned),
       .R(rem_unsigned),
       .ok(ready_unsigned),
       .err(error_unsigned));

    SDivide SDivide(
       .A(a),
       .B(b),
       .reset(clrn),
       .start(estart_sdivide),
       .clk(clk),
       .D(quotient_signed),
       .R(rem_signed),
       .ok(ready_signed),
       .err(error_signed));
 // Inputs
//reg start;
// Outputs
//wire pulse;

   Basic_and andfun(ready_unsigned,ready_signed,ready); 
   
//   mux4x32 lowerpart(product3[31:0],product1[31:0],
//                    product2[31:0],product3[31:0],func3,lower_product);
//   mux4x32 upperpart(product3[63:32],product1[63:32],
//                    product2[63:32],product3[63:32],func3,upper_product);
 
       
   //assign ready = ready_unsigned | ready_signed;
   assign c_mulhsu = product2[63:32];
   assign c_mulu = product3[63:32];
   assign c_mulh = product1[63:32];
   assign c_mul = product1[31:0];
   assign c_div = quotient_signed;
   assign c_divu = quotient_unsigned;
   assign c_rem = rem_signed;
   assign c_remu = rem_unsigned;

endmodule
