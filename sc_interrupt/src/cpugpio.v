`timescale 1ns / 1ps

// cpugpio.v
//
// General-purpose I/O module for Digilent's Nexys4-DDR board

`include "mfp_ahb_const.vh"

module cpugpio(
input              clk,
input              clrn,
output reg  [31:0] dataout,
output reg         dataout_ready,
input       [31:0] datain,
input       [5:0]  haddr,
input              we,
input              HSEL,

// memory-mapped I/O
input      [`MFP_N_SW-1:0]  IO_Switch,
input      [`MFP_N_PB-1:0]  IO_PB,
output reg [`MFP_N_LED-1:0] IO_LED,
output [7:0]            IO_7SEGEN_N,
output [6:0]            IO_7SEG_N,
output                  IO_BUZZ,                
output                  IO_RGB_SPI_MOSI,
output                  IO_RGB_SPI_SCK,
output reg              IO_RGB_SPI_CS,
output reg              IO_RGB_DC,
output reg              IO_RGB_RST,
output reg              IO_RGB_VCC_EN,
output reg              IO_RGB_PEN,
output                  IO_CS,
output                  IO_SCK,
input                   IO_SDO
);

// internal signals for I/Os: 7-segment displays, millisecond counter, buzzer
wire [ 6:0] segments;      // 7 segments value for enabled signal

reg  [ 7:0] SEGEN_N;       // 7-segment display enables
reg  [31:0] SEGDIGITS_N;   // value of 8 7-segment display digits
wire [31:0] millis;        // number of milliseconds since start of program
reg  [31:0] buzzerMicros;
reg  [ 7:0] SPIdata;       // 8-bit data to send from CPU
reg         SPIcmdb;       // cmd bar (1=data, 0 = command)
reg         SPIsend;       // CPU asserts SPIsend when data is ready to be sent
wire        SPIdone;       // SPI slave asserts SPIdone when done sending data 
wire  [15:0] PMOD_value;         // input from pmod device

reg  [5:0]  HADDR_d;
reg         HWRITE_d;
reg         HSEL_d;
reg  [1:0]  HTRANS_d;
reg  [31:0] datain_d;
wire        wes;            // write enable
reg     [3:0] IOwait_counter;
reg           IOready;
 
// delay HADDR, HWRITE, HSEL, and HTRANS to align with HWDATA for writing
always @ (posedge clk) begin
  datain_d <= datain;
  HADDR_d  <= haddr;
  HWRITE_d <= we;
  HSEL_d   <= HSEL;
end

//The CPU is capable of stalling for IO to finish reading before taking
//the value.  We do not use this ability and instead always signal completion
//in 1 clock cycle.  This means any future additions will either need to modify
//dataout_ready to wait appropriately or use the common paradigm of having 
//a flag in IO memory space that the software polls to check on read status.
always @(posedge clk or negedge clrn) begin
    if(~clrn) begin
        dataout_ready <= 0;
        IOwait_counter <= 4'b0;
        IOready <= 0;
     end 
     else if(HSEL) begin
        dataout_ready <= 1;
        IOready <= 1;                          // ready
        IOwait_counter <= 4'b0;
     end
     else begin
        dataout_ready <= 0;
        IOready <= 0;
        IOwait_counter <= 4'b0;
     end
end

// overall write enable signal
assign wes =  HSEL_d & HWRITE_d;

milliscounter milliscounter(
   .clk(clk),
   .resetn(clrn),
   .millis(millis));

buzzer buzzer(
   .clk(clk),
   .resetn(clrn),
   .numMicros(buzzerMicros),
   .buzz(IO_BUZZ));

SPI_Master #(
    .SPI_MODE(3),
    .CLKS_PER_HALF_BIT(4)
    ) rgb_spi (
    .i_Clk(clk),
    .i_Rst_L(clrn),
    .i_TX_Byte(SPIdata),
    .i_TX_DV(SPIsend),
    .o_TX_Ready(SPIdone),
    .o_RX_DV(),
    .o_RX_Byte(),
    .o_SPI_MOSI(IO_RGB_SPI_MOSI),
    .i_SPI_MISO(1'b0),
    .o_SPI_Clk(IO_RGB_SPI_SCK));

pmod_als_spi_receiver pmod_als_spi_receiver(
   .clock(clk),
   .reset_n(clrn),
   .cs(IO_CS),
   .sck(IO_SCK),
   .sdo(IO_SDO),
   .value(PMOD_value));

sevensegtimer sevensegtimer(
   .clk      (clk),    
   .resetn   (clrn),
   .EN       (SEGEN_N), 
   .DIGITS   (SEGDIGITS_N), 
   .DISPENOUT(IO_7SEGEN_N), 
   .DISPOUT  (IO_7SEG_N));

always @(posedge clk or negedge clrn)
   if (~clrn) begin
     IO_LED <= `MFP_N_LED'b0;  // turn LEDS off at reset
     // turn 7-segment displays off at reset
     SEGEN_N       <= 8'hff;          // 7-segment display enables
     SEGDIGITS_N   <= 32'hffffffff;   // 7-segment digit values      
     buzzerMicros  <= 32'b0;          // buzzer is off
     SPIdata       <= 8'b0;           // SPI data is 0
     SPIsend       <= 1'b0;           // CPU not sending SPI data
     IO_RGB_SPI_CS <= 1'b1;
     IO_RGB_DC     <= 1'b0;
     IO_RGB_RST    <= 1'b0;
     IO_RGB_VCC_EN <= 1'b0;
     IO_RGB_PEN    <= 1'b0;
   end else begin
     SPIsend <= 1'b0;
     if(wes) begin
        case (HADDR_d)
            `H_LED_IONUM: IO_LED <= datain_d[`MFP_N_LED-1:0];
            `H_7SEGEN_IONUM:     SEGEN_N      <= datain_d[7:0];
            `H_7SEGDIGITS_IONUM: SEGDIGITS_N  <= datain_d;
            `H_BUZZER_IONUM:     buzzerMicros <= datain_d;
            `H_RGB_SPI_DATA_IONUM: begin 
                               SPIdata   <= datain_d[7:0]; 
                               SPIsend   <= 1'b1;
                              end
            `H_RGB_SPI_CS_IONUM: IO_RGB_SPI_CS <= datain_d;
            `H_RGB_DC_IONUM:     IO_RGB_DC <= datain_d;
            `H_RGB_RST_IONUM:    IO_RGB_RST <= datain_d;
            `H_RGB_VCC_EN_IONUM: IO_RGB_VCC_EN <= datain_d;
            `H_RGB_PEN_IONUM:    IO_RGB_PEN <= datain_d;
         endcase
    end
  end

  always @(*) begin
      if(HSEL) begin
        case (haddr)
          `H_SW_IONUM: dataout = { {32 - `MFP_N_SW {1'b0}}, IO_Switch };
          `H_PB_IONUM: dataout = { {32 - `MFP_N_PB {1'b0}}, IO_PB };
          `H_MILLIS_IONUM:   dataout = millis;
          `H_RGB_SPI_DONE_IONUM: dataout = {31'b0, SPIdone};
          `H_LIGHTSENSOR_IONUM: dataout = {16'b0, PMOD_value};
        default:
            dataout = 32'h00000000;
         endcase
      end else begin
        dataout = 32'h00000000;
       end
      end

	
endmodule
