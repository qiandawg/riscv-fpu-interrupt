`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/25/2024 10:56:33 AM
// Design Name: 
// Module Name: multiplyunsignedunsigned
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

module multiplyunsignedunsigned(
  input [31:0] unsigned_data1,
  input [31:0] unsigned_data2,
  output [63:0] result
);

 
  assign     result = unsigned_data1 * unsigned_data2;
 

endmodule