`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/16/2024 09:03:54 AM
// Design Name: 
// Module Name: mux8x32
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


module mux8x32(
    input [31:0] a0,
    input [31:0] a1,
    input [31:0] a2,
    input [31:0] a3,
    input [31:0] a4,
    input [31:0] a5,
    input [31:0] a6,
    input [31:0] a7,
    input [2:0] s,
    output [31:0] y
    );
    
    function [31:0] select;  //function name (=return value, 32 bits)
         input [31:0] a0,a1,a2,a3,a4,a5,a6,a7; // notice the order of the input arguements
         input [2:0] s;   // notice the order of the input arguments
         case (s)
           3'b000: select = a0; // if (s==0) return value = a0
           3'b001: select = a1; // if (s==1) return value = a0
           3'b010: select = a2; // if (s==2) return value = a0
           3'b011: select = a3; // if (s==3) return value = a0
           3'b100: select = a4; // if (s==4) return value = a0
           3'b101: select = a5; // if (s==5) return value = a0
           3'b110: select = a6; // if (s==6) return value = a0
           3'b111: select = a7; // if (s==7) return value = a0
         endcase
    endfunction
    assign y = select(a0,a1,a2,a3,a4,a5,a6,a7,s);  // call the function with parameters
           
endmodule
