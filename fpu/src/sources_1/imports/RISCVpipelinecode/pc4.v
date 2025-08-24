 module pc4 (pc,halt, p4); // pc + 4
    input halt;
    input  [31:0] pc;
    output [31:0] p4;
    assign p4 = halt?32'b0:32'h4 + pc;
//    assign p4 = 32'h4 + pc;
endmodule
