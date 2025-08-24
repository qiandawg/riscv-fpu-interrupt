//Universal RAM
module uram(input clk,
            input we,
            input cs,
            input[31:0] addr,
            input[31:0] data_in,
            output reg[31:0] data_out);
            
parameter A_WIDTH = 0; //Address Bit width (gives 2^A_WIDTH 32 bit words)
parameter INIT_FILE = "";
parameter READ_DELAY = 0;

reg[31:0] ram[0:(1<<A_WIDTH)-1]; //Nx32 Instruction RAM
wire [31:0] addrmasked;
assign addrmasked = addr & 2**(A_WIDTH+2)-1;
//assign addrmasked = addr & (2**(A_WIDTH+2)-1); //NEW CODE
always @(posedge clk) begin
    if(we & cs) begin
        //Ram accesses in this system are word-oriented
        ram[addrmasked>>2] <= data_in;
    end
end

generate
    if(READ_DELAY == 0) begin
        //Asynchronous read (in the same clock cycle)
        always @(*) data_out = cs ? ram[addrmasked>>2] : 0;
    end else if(READ_DELAY == 1) begin
        //Registered read (ready by next clock cycle)
        always @(posedge clk) begin
            if(cs) data_out <= ram[addrmasked>>2];
            else data_out <= 0;
        end
    end
endgenerate

//Populate our program memory with the user-provided hex file
initial begin
    $readmemh(INIT_FILE, ram);
end

endmodule

