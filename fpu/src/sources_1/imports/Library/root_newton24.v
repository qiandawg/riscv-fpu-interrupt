/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module root_newton24 (d,fsqrt,ena,clk,clrn,q,busy,count,reg_x,stall);
    input  [23:0] d;                         // radicand: .1xx...x  .01x...x
    input         fsqrt;                     // ID stage: fsqrt = i_fsqrt
    input         clk, clrn;                 // clock and reset
    input         ena;                       // enable, save partial product
    output [31:0] q;                         // root: .1xxx...x
    output        busy;                      // cannot receive new div
    output        stall;                     // stall to save result
    output  [4:0] count;                     // for sim test only
    output [25:0] reg_x;                     // for sim test only 01.xx...x
    reg    [31:0] q;                         // root:     .1xxx...x
    reg    [23:0] reg_d;                     // 24-bit:   .xxxx...xx
    reg    [25:0] reg_x;                     // 26-bit: xx.1xxx...xx
    reg     [4:0] count;                     // 3 iterations * 7 cycles
    reg           busy;                      // cannot receive new fsqrt
    wire    [7:0] x0 = rom(d[23:19]);        // x0: from rom table
    wire   [51:0] x_2,x2d,x52;               // xxxx.xxxxx...x
    always @ (posedge clk or negedge clrn) begin
        if (!clrn) begin
            count <= 0;
            busy  <= 0;
            reg_x <= 0;
        end else begin
            if (fsqrt & (count == 0)) begin                // do once only
                count <= 5'b1;                             // set count
                busy  <= 1'b1;                             // set to busy
            end else begin                                 // 3 iterations
                if  (count == 5'h01) begin
                     reg_x <= {2'b1,x0,16'b0};             // 01.xxxx0...0
                     reg_d <= d;                           //   .1xxxx...x
                end
                if  (count != 0) count <= count + 5'b1;    // count++
                if  (count == 5'h15) busy  <= 0;           // ready for next
                if  (count == 5'h16) count <= 0;           // reset count
                if ((count == 5'h08) ||     // save result of 1st iteration
                    (count == 5'h0f) ||     // save result of 2nd iteration
                    (count == 5'h16))       // no need to save here actually
                     reg_x <= x52[50:25];   // /2 = xx.xxxxx...x
            end
        end
    end
    assign stall = fsqrt & (count == 0) | busy;
    // wallace_26x26_product (a,    b,    z);
    wallace_26x26_product x2 (reg_x,reg_x,x_2);           // xi(3-xi*xi*d)/2
    wallace_24x28_product xd (reg_d,x_2[51:24],x2d);
    wire   [25:0] b26 = 26'h3000000 - x2d[49:24];         // xx.xxxxx...x
    wallace_26x26_product xip1 (reg_x,b26,x52);
    reg    [25:0] reg_de_x; // pipeline register in between id and e1, x
    reg    [23:0] reg_de_d; // pipeline register in between id and e1, d
    wire   [49:0] m_s;      // sum:   41 + 8 = 49-bit
    wire   [49:8] m_c;      // carry: 42-bit
    // wallace_24x26 (a,       b,       x,        y,  z);
    wallace_24x26 wt (reg_de_d,reg_de_x,m_s[49:8],m_c,m_s[7:0]);    // d * x
    reg    [49:0] a_s;      // pipeline register in between e1 and e2, sum
    reg    [49:8] a_c;      // pipeline register in between e1 and e2, carry
    wire   [49:0] d_x = {1'b0,a_s} + {a_c,8'b0};             // 0x.xxxxx...x
    wire   [31:0] e2p = {d_x[47:17],|d_x[16:0]};             // sticky
    always @ (negedge clrn or posedge clk)
      if (!clrn) begin                                 // pipeline registers
          reg_de_x <= 0;          reg_de_d <= 0;                    // id-e1
          a_s      <= 0;          a_c      <= 0;                    // e1-e2
          q        <= 0;                                            // e2-e3
      end else if (ena) begin     // x52[50:25]: the result of 3rd iteration
          reg_de_x <= x52[50:25]; reg_de_d <= reg_d;                // id-e1
          a_s      <= m_s;        a_c      <= m_c;                  // e1-e2
          q        <= e2p;                                          // e2-e3
      end
    function  [7:0] rom;                           // a rom table: 1/d^{1/2}
        input [4:0] d;
        case (d)
            5'h08: rom = 8'hff;            5'h09: rom = 8'he1;
            5'h0a: rom = 8'hc7;            5'h0b: rom = 8'hb1;
            5'h0c: rom = 8'h9e;            5'h0d: rom = 8'h9e;
            5'h0e: rom = 8'h7f;            5'h0f: rom = 8'h72;
            5'h10: rom = 8'h66;            5'h11: rom = 8'h5b;
            5'h12: rom = 8'h51;            5'h13: rom = 8'h48;
            5'h14: rom = 8'h3f;            5'h15: rom = 8'h37;
            5'h16: rom = 8'h30;            5'h17: rom = 8'h29;
            5'h18: rom = 8'h23;            5'h19: rom = 8'h1d;
            5'h1a: rom = 8'h17;            5'h1b: rom = 8'h12;
            5'h1c: rom = 8'h0d;            5'h1d: rom = 8'h08;
            5'h1e: rom = 8'h04;            5'h1f: rom = 8'h00;
            default: rom = 8'hff;                  // 0 - 7: not be accessed
        endcase
    endfunction
endmodule
