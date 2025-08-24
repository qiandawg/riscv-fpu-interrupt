// aluc[3:0]:
// x 0 0 0  ADD 0, 8, add, addi, lw, sw
// x 0 0 1  SUB 1, 9, sub, beq, bne
// x 0 1 0  SLT 2, a, slt
// 1 0 1 1  XOR b,  , xor, xori
// x 1 0 0  OR  4, c, or, ori
// x 1 0 1  AND 5, d, and, andi
// 0 0 1 1  SLL 3,  , slli
// 0 1 1 1  SRL 7,  , srli
// 1 1 1 1  SRA f,  , srai
// x 1 1 0  LUI 6, e, lui
// no operation:      jalr, jal

module alu_ov (a,b,aluc,r,z,v);                             // 32-bit alu with a zero and overfloat flag
    input  [31:0] a, b;                                // inputs: a, b
    input   [3:0] aluc;                                // input:  alu control
    output [31:0] r;                                   // output: alu result
    output        z,v;                                   // output: zero, overflow flag
    assign r = calc(a,b,aluc);                         // call function
    assign z = ~|r;                                    // z = (r == 0)
    assign v =  ~aluc[2] & ~a[31] & ~b[31] &  r[31] & ~aluc[1] & ~aluc[0] |
                ~aluc[2] &  a[31] &  b[31] & ~r[31] & ~aluc[1] & ~aluc[0] |
                ~aluc[2] & ~a[31] &  b[31] &  r[31] & ~aluc[1] & aluc[0] |
                ~aluc[2] &  a[31] & ~b[31] & ~r[31] & ~aluc[1] & aluc[0];
    function  [31:0] calc;                             // function
        input [31:0] a,b;                              // input
        input  [3:0] aluc;                             // input
        reg   [31:0] sub;                              // local variable
        begin
            sub = a-b;                             // for SUB and SLT
            casex(aluc)                                // allowing don't care
                4'bx000: calc = a + b;                 // ADD, aluc: 0, 8
                4'bx001: calc = a - b;                    // SUB, aluc: 1, 9
                4'bx010: calc = {31'b0,sub[31]};       // SLT, aluc: 2, a
                4'b1011: calc = a ^ b;                 // XOR, aluc: b,
                4'bx100: calc = a | b;                 // OR,  aluc: 4, c
                4'bx101: calc = a & b;                 // AND, aluc: 5, d
                4'b0011: calc = a << b[4:0];           // SLL, aluc: 3,
                4'b0111: calc = a >> b[4:0];           //    SRL, aluc: 7,
                4'b1111: calc = $signed(a) >>> b[4:0]; // SRA, aluc: f,
                4'bx110: calc = b;                     // LUI, aluc: 6, e
                default: calc = 32'b0;  // default is 0   NEW
            endcase
        end
    endfunction
endmodule
