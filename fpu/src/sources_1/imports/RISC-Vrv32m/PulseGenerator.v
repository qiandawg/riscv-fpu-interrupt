`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/18/2024 04:05:22 PM
// Design Name: 
// Module Name: PulseGenerator
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


module PulseGenerator (
    input wire clk,     // Clock signal
    input wire request, // Request signal
    output reg pulse    // Output pulse signal
);
reg prev_request;

always @(posedge clk) begin
   if (request && !prev_request)
       pulse <= 1'b1;
   else
       pulse <= 1'b0;
   prev_request <= request;
end

endmodule
