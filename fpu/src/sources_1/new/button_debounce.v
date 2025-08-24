`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/29/2025 12:54:54 PM
// Design Name: 
// Module Name: button_debounce
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



module button_debounce #(
    parameter DEBOUNCE_PERIOD = 1000000  // 10ms at 100MHz clock
)(
    input  wire clk,           // System clock
    input  wire btn_in,        // Raw button input
    output reg  btn_debounced  // Debounced button output
);

    // State registers
    reg [1:0] btn_sync;        // Synchronizer
    reg [19:0] counter;        // Counter for debounce period
    reg btn_state;             // Current stable button state

    // Synchronize the button input to prevent metastability
    always @(posedge clk) begin
        btn_sync <= {btn_sync[0], btn_in};
    end

    // Debounce logic
    always @(posedge clk) begin
        if (btn_sync[1] != btn_state) begin
            // Button state is different from stable state, start/continue counting
            if (counter == DEBOUNCE_PERIOD - 1) begin
                // Counter reached threshold, update stable state
                btn_state <= btn_sync[1];
                counter <= 0;
            end else begin
                // Keep counting
                counter <= counter + 1;
            end
        end else begin
            // Button state matches stable state, reset counter
            counter <= 0;
        end
    end

    // Output the debounced button state
    always @(posedge clk) begin
        btn_debounced <= btn_state;
    end

endmodule