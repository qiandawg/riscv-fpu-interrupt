/************************************************
  The Verilog HDL code example is from the book
  Computer Principles and Design in Verilog HDL
  by Yamin Li, published by A JOHN WILEY & SONS
************************************************/
module newton24 (a,b,fdiv,ena,clk,clrn,q,busy,count,reg_x,stall); 
    input  [23:0] a;                         // dividend: .1xxx...x
    input  [23:0] b;                         // divisor:  .1xxx...x
    input         fdiv;                      // ID stage: i_fdiv
    input         clk, clrn;                 // clock and reset
    input         ena;                       // enable, save partial product
    output [31:0] q;                         // quotient: x.xxxxx...x
    output reg    busy;                      // cannot receive new div
    output  [4:0] count;                     // counter
    output [25:0] reg_x;                     // for sim test only 01.xx...x
    output        stall;                     // for pipeline stall
    reg    [31:0] q;                         // 32-bit:  x.xxxxx...x
    reg    [25:0] reg_x;                     // 26-bit: xx.xxxxx...x
    reg    [23:0] reg_a;                     // 24-bit:   .1xxxx...x
    reg    [23:0] reg_b;                     // 24-bit:   .1xxxx...x
    reg     [4:0] count;                     // 3 iterations * 5 cycles
    wire   [49:0] bxi;                       //  xx.xxxxx...x
    wire   [51:0] x52;                       // xxx.xxxxx...x
    wire   [49:0] d_x;                       //  0x.xxxxx...x
    wire   [31:0] e2p;                       // sticky
    wire    [7:0] x0 = rom(b[22:19]);        // x0: from rom table
    always @ (posedge clk or negedge clrn) begin
        if (!clrn) begin
            busy  <= 0;
            count <= 0;
            reg_x <= 0;
        end else begin
            if (fdiv & (count == 0)) begin                 // do once only
                count <= 5'b1;                             // set count
                busy  <= 1'b1;                             // set to busy
            end else begin                                 // 3 iterations
                if  (count == 5'h01) begin
                     reg_a <= a;                           //   .1xxxx...x
                     reg_b <= b;                           //   .1xxxx...x
                     reg_x <= {2'b1,x0,16'b0};             // 01.xxxx0...0
                end
                if  (count != 0) count <= count + 5'b1;    // count++
                if  (count == 5'h0f) busy <= 0;            // ready for next
                if  (count == 5'h10) count <= 5'b0;        // reset count
                if ((count == 5'h06) ||     // save result of 1st iteration
                    (count == 5'h0b) ||     // save result of 2nd iteration
                    (count == 5'h10))       // no need to save here actually
                     reg_x <= x52[50:25];   // xx.xxxxx...x
            end
        end
    end
    assign stall = fdiv & (count == 0) | busy;
    // wallace_26x24_product   (a,    b,    z);
    wallace_26x24_product bxxi (reg_b,reg_x,bxi);       //           xi * b
    wire   [25:0] b26 = ~bxi[48:23] + 1'b1;             //       2 - xi * b
    wallace_26x26_product xip1 (reg_x,b26,x52);         // xi * (2 - xi * b)
    reg    [25:0] reg_de_x; // pipeline register in between id and e1, x
    reg    [23:0] reg_de_a; // pipeline register in between id and e1, a
    wire   [49:0] m_s;      // sum
    wire   [49:8] m_c;      // carry
    // wallace_24x26 (a,       b,       x,        y,  z);
    wallace_24x26 wt (reg_de_a,reg_de_x,m_s[49:8],m_c,m_s[7:0]); // a * xn
    reg    [49:0] a_s;      // pipeline register in between e1 and e2, sum
    reg    [49:8] a_c;      // pipeline register in between e1 and e2, carry
    assign d_x = {1'b0,a_s} + {a_c,8'b0};                    // 0x.xxxxx...x
    assign e2p = {d_x[48:18],|d_x[17:0]};                    // sticky
    always @ (negedge clrn or posedge clk)
      if (!clrn) begin                                 // pipeline registers
          reg_de_x <= 0;                      reg_de_a <= 0;        // id-e1
          a_s      <= 0;                      a_c      <= 0;        // e1-e2
          q        <= 0;                                            // e2-e3
      end else if (ena) begin     // x52[50:25]: the result of 3rd iteration
          reg_de_x <= x52[50:25];             reg_de_a <= reg_a;    // id-e1
          a_s      <= m_s;                    a_c      <= m_c;      // e1-e2
          q        <= e2p;                                          // e2-e3
      end
    function  [7:0] rom;                                      // a rom table
        input [3:0] b;
        case (b)
            4'h0: rom = 8'hff;            4'h1: rom = 8'hdf;
            4'h2: rom = 8'hc3;            4'h3: rom = 8'haa;
            4'h4: rom = 8'h93;            4'h5: rom = 8'h7f;
            4'h6: rom = 8'h6d;            4'h7: rom = 8'h5c;
            4'h8: rom = 8'h4d;            4'h9: rom = 8'h3f;
            4'ha: rom = 8'h33;            4'hb: rom = 8'h27;
            4'hc: rom = 8'h1c;            4'hd: rom = 8'h12;
            4'he: rom = 8'h08;            4'hf: rom = 8'h00;
        endcase
    endfunction
endmodule
