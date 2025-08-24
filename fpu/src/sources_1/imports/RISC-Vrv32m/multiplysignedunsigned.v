`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/25/2024 10:56:33 AM
// Design Name: 
// Module Name: multiplysignedunsigned
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

module multiplysignedunsigned (
    input  signed [31:0] signed_data1,       // Signed 32-bit input
    input         [31:0] unsigned_data2,       // Unsigned 32-bit input
    output signed [63:0] result  // Signed 64-bit product
);
    // Sign-extend 'a' to 64 bits
    wire signed [63:0] a_ext = {{32{signed_data1[31]}}, signed_data1};

    // Zero-extend 'b' to 64 bits
    wire        [63:0] b_ext = {32'b0, unsigned_data2};

    // Perform signed * unsigned = signed multiplication
    assign result = a_ext * b_ext;
endmodule
