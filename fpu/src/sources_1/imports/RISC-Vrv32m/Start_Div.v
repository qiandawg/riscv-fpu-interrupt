`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/11/2024 12:51:06 PM
// Design Name: 
// Module Name: Start_Div
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


module Start_Div(
    input clk,
    input [2:0] func3,
    input fuse,
    input rv32m,
    output start_sdivide,
    output start_udivide
    );
    reg ustartflag,sstartflag;
   
    always @(*) begin

       if((rv32m==1))
       begin
            ustartflag = 0;
            sstartflag = 0;
            if ((func3==3'h4)&&(fuse==0))
            begin
             sstartflag = 1'h1;
            end 
            if ((func3==3'h5)&&(fuse==0))
            begin
              ustartflag = 1'h1;

            end  
            if ((func3==3'h6)&&(fuse==0))
            begin
             sstartflag = 1'h1;
            end 
            if ((func3==3'h7)&&(fuse==0))
            begin
             ustartflag = 1'h1;
            end 
         end
    end
             PulseGenerator PulseGen1(
              .request(sstartflag),
               .clk(clk),
               .pulse(start_sdivide)
               );
             PulseGenerator PulseGen2(
               .request(ustartflag),
               .clk(clk),
               .pulse(start_udivide)
               );
   
endmodule


