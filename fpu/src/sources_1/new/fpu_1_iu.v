

`include "mfp_ahb_const.vh"


module fpu_1_iu (
    input         SI_ClkIn, memclk, SI_Reset_N,              // clocks and reset
    output [31:0] pc, inst, eal, mal,   //wal,
    output [31:0] e3d, wd,
    output  [4:0] e1n, e2n, e3n, wn,
    output        ww, stl_lw, stl_fp, stl_lwc1, stl_swc1, stl,
    output        e,                // for multithreading CPU, not used here
    output  [4:0] cnt_div, cnt_sqrt,               // for testing
    input  [`MFP_N_SW-1 :0] IO_Switch,
    input  [`MFP_N_PB-1 :0] IO_PB,
    output [`MFP_N_LED-1:0] IO_LED,
    output [ 7          :0] IO_7SEGEN_N,
    output [ 6          :0] IO_7SEG_N,
    output                  IO_BUZZ,                  
    output                  IO_RGB_SPI_MOSI,
    output                  IO_RGB_SPI_SCK,
    output                  IO_RGB_SPI_CS,
    output                  IO_RGB_DC,
    output                  IO_RGB_RST,
    output                  IO_RGB_VCC_EN,
    output                  IO_RGB_PEN,
    output                  IO_CS,
    output                  IO_SCK,
    input                   IO_SDO,
    input                   UART_RX,
    inout [8:1] JB);
  // Press btnCpuReset to reset the processor. 


       

    wire   [31:0] qfa,qfb,fa,fb,dfa,dfb,mmo,wmo;   // for iu
    wire    [4:0] fs,ft,fd,wrn; 
    wire    [2:0] fc;
    wire    [1:0] e1c,e2c,e3c;                     // for fpu
    wire          fwdla,fwdlb,fwdfa,fwdfb,wf,fasmds,e1w,e2w,e3w,wwfpr;
//  wire [31:0] pc,inst,eal,mal,wres;
  pl_computer cpu( .SI_ClkIn(SI_ClkIn),
                    .SI_Reset_N(SI_Reset_N),                  
                    .pc(pc),
                    .inst(inst),
                    .eal(eal),
                    .mal(mal),
                    .wres(wres),
                    .IO_Switch(SW),
                    .IO_PB({BTNU, BTND, BTNL, BTNC, BTNR}),
                    .IO_LED(LED),
                    .IO_7SEGEN_N(AN),
                    .IO_7SEG_N({CA,CB,CC,CD,CE,CF,CG}), 
                    .IO_BUZZ(IO_BUZZ),
                    .IO_RGB_SPI_MOSI(IO_RGB_SPI_MOSI),
                    .IO_RGB_SPI_SCK(IO_RGB_SPI_SCK),
                    .IO_RGB_SPI_CS(IO_RGB_SPI_CS),
                    .IO_RGB_DC(IO_RGB_DC),
                    .IO_RGB_RST(IO_RGB_RST),
                    .IO_RGB_VCC_EN(IO_RGB_VCC_EN),
                    .IO_RGB_PEN(IO_RGB_PEN),
                    .IO_CS(IO_CS),
                    .IO_SCK(IO_SCK),
                    .IO_SDO(IO_SDO),
                    .UART_RX(UART_TXD_IN),
                    .JB(JB));                    

    regfile2w fpr (fs,ft,wd,wn,ww,wmo,wrn,wwfpr,~SI_ClkIn,SI_Reset_N,qfa,qfb);
    mux2x32 fwd_f_load_a (qfa,mmo,fwdla,fa);       // forward lwc1 to fp a
    mux2x32 fwd_f_load_b (qfb,mmo,fwdlb,fb);       // forward lwc1 to fp b
    mux2x32 fwd_f_res_a  (fa,e3d,fwdfa,dfa);       // forward fp res to fp a
    mux2x32 fwd_f_res_b  (fb,e3d,fwdfb,dfb);       // forward fp res to fp b
    fpu fp_unit (dfa,dfb,fc,wf,fd,1'b1,SI_ClkIn,SI_Reset_N,e3d,wd,wn,ww,
                 stl,e1n,e1w,e2n,e2w,e3n,e3w,
                 e1c,e2c,e3c,cnt_div,cnt_sqrt,e,1'b1);
                 
endmodule
