`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/25/2024 10:56:33 AM
// Design Name: 
// Module Name: multiplysignedsigned
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


module multiplysignedsigned(
  input signed [31:0] signed_data1,
  input signed [31:0] signed_data2,
  output signed [63:0] result
);

       assign result = $signed(signed_data1) * $signed(signed_data2);
 
endmodule
