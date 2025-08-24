`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/16/2024 09:44:01 AM
// Design Name: 
// Module Name: diffe5
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


module dffe5(d,clk,clrn,e,q);                         // a 32-bit register
    input      [4:0] d;                                // input d
    input             e;                                // e: enable
    input             clk, clrn;                        // clock and reset
    output reg [4:0] q;                                // output q
    always @(negedge clrn or posedge clk)
        if (!clrn)  q <= 0;                             // q = 0 if reset
        else if (e) q <= d;                             // save d if enabled
endmodule
