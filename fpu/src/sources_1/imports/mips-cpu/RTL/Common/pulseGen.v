/* pulseGen.v
 *
 * A simple pulse generator
 */

module pulseGen (input clk,
                 input rst_p,
                 output reg pulse);
    
    parameter NBITS = 0;
    parameter DIVISOR = 0;

    reg[NBITS-1:0] counter;
        
    always @(posedge clk, posedge rst_p) begin
        if(rst_p) begin
            pulse <= 0;
            counter <= 0;
        end else begin
            if(counter == DIVISOR-1) begin
                pulse <= 1;
                counter <= 0;
            end else begin
                pulse <= 0;
                counter <= counter + 1;
            end
        end
    end
    
endmodule
