`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/24/2021 12:11:01 PM
// Design Name: 
// Module Name: Divide
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
  module SDivide(  
   input      clk,  
   input      reset,  
   input      start,  
   input [31:0]  A,  
   input [31:0]  B,  
   output [31:0]  D,  
   output [31:0]  R,  
   output     ok ,   // =1 when ready to get the result   
   output err  
   );  
   reg       active;   // True if the divider is running  
   reg       Dsign;
   reg      signA;
   reg      signB; // sign will be the xor of sign bits, and
   // signA and signB are the sign bits of A And B
   reg [4:0]    cycle;   // Number of cycles to go  
   reg [31:0]   result;   // Begin with A, end with D  
   reg [31:0]   denom;   // B  
   reg [31:0]   work;    // Running R  
   // Calculate the current digit  
   wire [32:0]   sub = { work[30:0], result[31] } - denom;  
       assign err = ~|B;  // True if B == 0  //Test if divide by 0
   // Send the results to our master  
   assign D = result;  
   assign R = work;  
   assign ok = ~active;
 
 
   // The state machine  
   always @(posedge clk, negedge reset,posedge start) begin  
   //$display("Always sdivide.v %b",start); 
     if (!reset) begin  
       active <= 0;  
       cycle <= 0;  
       result <= 0;  
       denom <= 0;  
       work <= 0;
       Dsign <= 0;
       signA <= 0;
       signB <= 0;
       $display("Initialized variables sdivide.v %b",reset);
 
     end  
     else if(start) begin  
           // Set up for an unsigned divide
           $display("start triggered: %b", start);
         Dsign <= A[31]^B[31];
         signA <= A[31];
         signB <= B[31];
         if (B[31]==1) begin
               denom <= ~B +1;
         end else begin
               denom <= B;
         end
         if (A[31] == 1) begin
               result <= ~A +1;
         end else begin
               result <= A;
         end
         
         cycle <= 5'd31;  

         work <= 32'b0;  
         active <= 1;  
         $display("active level (set 1): %b", active); 
       end else if (active) begin  
         // Run an iteration of the divide. 
          $display("active level: %b", active); 
         if (sub[32] == 0) begin  
           work <= sub[31:0];  
           result <= {result[30:0], 1'b1};  
         end  
         else begin  
           work <= {work[30:0], result[31]};  
           result <= {result[30:0], 1'b0};  
         end  
         if (cycle == 0) begin 

           if (signA ==0 && signB ==1) begin
               if (sub[32] == 0) begin  
                    work <= sub[31:0];  
                    result <= ~{result[30:0], 1'b1}+1;  
                end  
           else begin  
                    work <= {work[30:0], result[31]};  
                    result <= ~{result[30:0], 1'b0}+1;  
            end  

               // result <= ~result +1;
           end else if (signA==1 && signB ==0) begin
                if (sub[32] == 0) begin  
                    work <= ~sub[31:0]+1;  
                    result <= ~{result[30:0], 1'b1}+1;  
                end  
            else begin  
                    work <= ~{work[30:0], result[31]}+1;  
                    result <= ~{result[30:0], 1'b0}+1;  
            end  

               // result <= ~result+1;
                //work <= ~work +1;
           end else if (signA == 1 && signB == 1) begin
                if (sub[32] == 0) begin  
                    work <= ~sub[31:0]+1;  
                    result <= {result[30:0], 1'b1};  
                end  
                else begin  
                    work <= ~{work[30:0], result[31]}+1;  
                    result <= {result[30:0], 1'b0};  
                end  

                //work <= ~work +1;
           end          

           active <= 0;
            $display("active level(0): %b", active);           

           end  
         
         cycle <= cycle - 5'd1;  
       end  

   end  
 endmodule 