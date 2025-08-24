`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/16/2024 09:44:01 AM
// Design Name: 
// Module Name: diffe
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


module dffe (d,clk,clrn,e,q);         // dff (async) with write enable
    input      d, clk, clrn;          // inputs d, clk, clrn (active low)
    input      e;                     // enable
    output reg q;                     // output q, register type
    always @ (posedge clk or negedge clrn) begin // always block, "or"
        if (!clrn)  q <= 0;           // if clrn is asserted, reset dff
        else if (e) q <= d;           // else if enabled store d to dff
    end
endmodule
