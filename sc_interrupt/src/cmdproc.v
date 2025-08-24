/* jtaglet.v
 *
 * Top module for the currently unnamed V0.1 of the UART debug monitor
 *
 *------------------------------------------------------------------------------
 *
 * Copyright 2021 Christopher Parish
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

module cmdproc #(
    parameter CLKS_PER_BIT = 217
    )
    (
    input clk,
    input rst_n,
    
    input serial_rx,
    output serial_tx,
    
    output reg[31:0] addr,
    output reg[31:0] data_out,
    input[31:0] data_in,
    output reg data_out_ready,
    input data_write_complete,
    input data_in_valid,
    output reg data_imem_p_dmem_n,
    
    output reg cpu_halt,
    output reg cpu_step,
    output reg cpu_reset_p
    
    );
    
    `include "ascii.vh"
    
    reg tx_valid;
    wire[7:0] tx_byte;
    wire tx_active;
    wire tx_done;
    
    wire rx_valid;
    wire[7:0] rx_byte;
    
    //The UART TX and RX IP modules from nandland
    UART_TX #(
            .CLKS_PER_BIT(CLKS_PER_BIT)
        ) uart_tx_1 (
            .i_Rst_L(rst_n),
            .i_Clock(clk),
            .i_TX_DV(tx_valid),
            .i_TX_Byte(tx_byte),
            .o_TX_Active(tx_active),
            .o_TX_Serial(serial_tx),
            .o_TX_Done(tx_done)
        );
        
    UART_RX #(
            .CLKS_PER_BIT(CLKS_PER_BIT)
        ) uart_rx_1 (
            .i_Rst_L(rst_n),
            .i_Clock(clk),
            .i_RX_Serial(serial_rx),
            .o_RX_DV(rx_valid),
            .o_RX_Byte(rx_byte)
        );

        
    function [3:0] CHAR_to_nibble;
        input [7:0] in;
        begin
            case(in)
            CHAR_A,CHAR_a: CHAR_to_nibble = 4'hA;
            CHAR_B,CHAR_b: CHAR_to_nibble = 4'hB;
            CHAR_C,CHAR_c: CHAR_to_nibble = 4'hC;
            CHAR_D,CHAR_d: CHAR_to_nibble = 4'hD;
            CHAR_E,CHAR_e: CHAR_to_nibble = 4'hE;
            CHAR_F,CHAR_f: CHAR_to_nibble = 4'hF;
            default: CHAR_to_nibble = in[3:0];
            endcase
        end
    endfunction


    function [7:0] nibble_to_CHAR;
        input [3:0] in;
        begin
            case(in)
            4'hA: nibble_to_CHAR = CHAR_A;
            4'hB: nibble_to_CHAR = CHAR_B;
            4'hC: nibble_to_CHAR = CHAR_C;
            4'hD: nibble_to_CHAR = CHAR_D;
            4'hE: nibble_to_CHAR = CHAR_E;
            4'hF: nibble_to_CHAR = CHAR_F;
            default: nibble_to_CHAR = {4'h3, in[3:0]};
            endcase
        end
    endfunction
  
    //Our overarching state machine organizing the command buffering, 
    //processing, and response
    localparam STATE_UART_RX = 0;
    localparam STATE_PROCESS = 1;
    localparam STATE_STRING_BUILD = 2;
    localparam STATE_UART_TX = 3;
    
    reg[1:0] state;
    reg in_buf_complete;
    reg process_complete;
    reg tx_complete;


    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) state <= STATE_UART_RX;
        else begin
            case(state)
            STATE_UART_RX: if(in_buf_complete) state <= STATE_PROCESS;
            STATE_PROCESS: if(process_complete) state <= STATE_STRING_BUILD;
            STATE_STRING_BUILD: state <= STATE_UART_TX;
            STATE_UART_TX: if(tx_complete) state <= STATE_UART_RX;
            endcase
        end
    end
    
    //UART RX buffering
    integer i;
    reg[7:0] in_buf[0:9];
    reg[4:0] in_buf_ptr;
    
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            in_buf_ptr <= 0;
            in_buf_complete <= 0;
            for(i=0; i<10; i=i+1) in_buf[i] <= 8'd0;
        end else begin
            in_buf_complete <= 0;
            if(tx_complete) in_buf_ptr <= 0;
            else if(state == STATE_UART_RX && rx_valid) begin
                if(in_buf_ptr < 10) begin
                    in_buf_ptr <= in_buf_ptr + 1;
                    in_buf[in_buf_ptr] <= rx_byte;
                end
                
                //We see a CR or LF, so the command should be issued
                if(rx_byte == CHAR_CR || rx_byte == CHAR_LF) begin
                    in_buf_complete <= 1;
                end
            end
        end
    end
    
    //Command Processing 
    reg[4:0] cur_command;

    localparam CMD_NONE = 0;
    localparam CMD_HALT = 1;
    localparam CMD_RESET = 2;
    localparam CMD_STEP = 3;
    localparam CMD_GO = 4;
    localparam CMD_ADDR = 5;
    localparam CMD_WRITE = 6;
    localparam CMD_READ = 7;
    localparam CMD_IMEM = 8;
    localparam CMD_DMEM = 9;
    localparam CMD_EXTRESET = 10;
    
    //CMD_HALT, CMD_GO
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) cpu_halt <= 0;
        else if (state == STATE_PROCESS) begin
            if(cur_command == CMD_HALT) cpu_halt <= 1;
            else if(cur_command == CMD_GO) cpu_halt <= 0;
        end
    end
    
    //CMD_IMEM, CMD_DMEM
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) data_imem_p_dmem_n <= 0;
        else if (state == STATE_PROCESS) begin
            if(cur_command == CMD_IMEM) data_imem_p_dmem_n <= 1;
            else if(cur_command == CMD_DMEM) data_imem_p_dmem_n <= 0;
        end
    end
    
    //CMD_RESET
    reg[7:0] cpu_reset_ctr;
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            cpu_reset_p <= 0;
            cpu_reset_ctr <= 0;
        end else if (state == STATE_PROCESS) begin
            if(cur_command == CMD_RESET) begin 
                cpu_reset_ctr <= cpu_reset_ctr + 1;
                cpu_reset_p <= |cpu_reset_ctr;
            end
        end
    end
    
    //CMD_STEP
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) cpu_step <= 0;
        else begin 
            cpu_step <= 0;
            if (state == STATE_PROCESS) begin
                if(cur_command == CMD_STEP) cpu_step <= 1;
            end
        end
    end
    
    //CMD_ADDR
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) addr <= 32'h00000000;
        else if (state == STATE_PROCESS) begin
            if(cur_command == CMD_ADDR) begin
                addr <= { CHAR_to_nibble(in_buf[2]),
                          CHAR_to_nibble(in_buf[3]),
                          CHAR_to_nibble(in_buf[4]),
                          CHAR_to_nibble(in_buf[5]),
                          CHAR_to_nibble(in_buf[6]),
                          CHAR_to_nibble(in_buf[7]),
                          CHAR_to_nibble(in_buf[8]),
                          CHAR_to_nibble(in_buf[9])
                          };
            end
        end
    end
    
    //CMD_READ
    reg[31:0] data_in_reg;
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) data_in_reg <= 0;
        else if (state == STATE_PROCESS) begin
            if(cur_command == CMD_READ) begin
                if(data_in_valid) data_in_reg <= data_in;
            end
        end
    end
    
    //CMD_WRITE
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            data_out <= 32'h00000000;
            data_out_ready <= 0;
        end else begin
            data_out_ready <= 0;
            if(state == STATE_PROCESS) begin
                if(cur_command == CMD_WRITE && !data_out_ready) begin
                    data_out_ready <= 1;
                    data_out <= { CHAR_to_nibble(in_buf[2]),
                                    CHAR_to_nibble(in_buf[3]),
                                    CHAR_to_nibble(in_buf[4]),
                                    CHAR_to_nibble(in_buf[5]),
                                    CHAR_to_nibble(in_buf[6]),
                                    CHAR_to_nibble(in_buf[7]),
                                    CHAR_to_nibble(in_buf[8]),
                                    CHAR_to_nibble(in_buf[9])
                                    };
                end
            end
        end
    end
    
    //Command decoder    
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) cur_command <= CMD_NONE;
        else begin
            cur_command <= CMD_NONE;
            case(in_buf[0])
            CHAR_H,CHAR_h: cur_command <= CMD_HALT;
            CHAR_S,CHAR_s: cur_command <= CMD_STEP;
            CHAR_E,CHAR_e: cur_command <= CMD_RESET;
            CHAR_G,CHAR_g: cur_command <= CMD_GO;
            CHAR_A,CHAR_a: cur_command <= CMD_ADDR;
            CHAR_R,CHAR_r: cur_command <= CMD_READ;
            CHAR_W,CHAR_w: cur_command <= CMD_WRITE;
            CHAR_I,CHAR_i: cur_command <= CMD_IMEM;
            CHAR_D,CHAR_d: cur_command <= CMD_DMEM;
            
            endcase
        end
    end
    
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) process_complete <= 0;
        else if(state == STATE_PROCESS) begin
            case(cur_command)
            CMD_HALT: process_complete <= 1;
            CMD_STEP: process_complete <= 1;
            CMD_RESET: process_complete <= &cpu_reset_ctr;
            CMD_GO: process_complete <= 1;
            CMD_ADDR: process_complete <= 1;
            CMD_READ: process_complete <= data_in_valid;
            CMD_WRITE: process_complete <= data_write_complete;
            CMD_IMEM: process_complete <= 1;
            CMD_DMEM: process_complete <= 1;
            default: process_complete <= 1;
            endcase
        end else process_complete <= 0;
    end
    
    //UART TX String Building
    reg[7:0] out_buf[15:0];
    
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            for(i=0; i<16; i=i+1) out_buf[i] <= 8'd0;
        end else if(state == STATE_PROCESS) begin
            case(cur_command)
            CMD_HALT,CMD_STEP,CMD_RESET,CMD_GO,CMD_ADDR,
            CMD_WRITE,CMD_IMEM,CMD_DMEM: begin
                out_buf[0] <= CHAR_O;
                out_buf[1] <= CHAR_K;
                out_buf[2] <= CHAR_CR;
                out_buf[3] <= CHAR_LF;
                out_buf[4] <= 0;
                end
            CMD_READ: begin
                out_buf[0] <= nibble_to_CHAR(data_in_reg[31:28]);
                out_buf[1] <= nibble_to_CHAR(data_in_reg[27:24]);
                out_buf[2] <= nibble_to_CHAR(data_in_reg[23:20]);
                out_buf[3] <= nibble_to_CHAR(data_in_reg[19:16]);
                out_buf[4] <= nibble_to_CHAR(data_in_reg[15:12]);
                out_buf[5] <= nibble_to_CHAR(data_in_reg[11:8]);
                out_buf[6] <= nibble_to_CHAR(data_in_reg[7:4]);
                out_buf[7] <= nibble_to_CHAR(data_in_reg[3:0]);
                out_buf[8] <= CHAR_CR;
                out_buf[9] <= CHAR_LF;
                out_buf[10] <= 0;
                end
            default: begin
                out_buf[0] <= CHAR_QM;
                out_buf[1] <= CHAR_CR;
                out_buf[2] <= CHAR_LF;
                out_buf[3] <= 0;
                end
            endcase
        end
    end
    
    //UART TX Transmit
    reg[4:0] out_buf_ptr;
    
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            out_buf_ptr <= 0;
            tx_valid <= 0;
        end else begin
            tx_complete <= 0;
            if(state == STATE_UART_TX) begin
                tx_valid <= 0;
                if(~tx_active & ~tx_valid) begin 
                    if(out_buf_ptr == 15 || out_buf[out_buf_ptr+1] == 0) tx_complete <= 1;
                    else if(~tx_complete) tx_valid <= 1;
                end
                
                if(tx_done & ~tx_valid) begin
                    out_buf_ptr <= out_buf_ptr + 1;
                end
            end else out_buf_ptr <= 0;
        end
    end
    
    assign tx_byte = out_buf[out_buf_ptr];

endmodule
