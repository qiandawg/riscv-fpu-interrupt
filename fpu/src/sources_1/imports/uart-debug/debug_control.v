module debug_control(
    input serial_rx,
    output serial_tx,

    input sys_rstn, //System reset. Should NOT be externally tied to our cpu_resetn_cpu output

    input cpu_clk,

    output[31:0] cpu_imem_addr,
    output[31:0] cpu_debug_to_imem_data,
    input[31:0] cpu_imem_to_debug_data,
    input cpu_imem_to_debug_data_ready,
    output cpu_imem_ce,
    output cpu_imem_we,

    output[31:0] cpu_dmem_addr,
    output[31:0] cpu_debug_to_dmem_data,
    input[31:0] cpu_dmem_to_debug_data,
    input cpu_dmem_to_debug_data_ready,
    output cpu_dmem_ce,
    output cpu_dmem_we,

    output cpu_halt_cpu,
    output cpu_resetn_cpu
    );

    // 50MHz / 115200 = 434 Clocks Per Bit.
    parameter CLKS_PER_BIT = 434;

    wire[31:0] addr;
    wire[31:0] data_out;
    wire[31:0] data_in;
    wire data_out_ready;
    wire data_in_valid;
    wire data_imem_p_dmem_n;
    wire cpu_reset_p;
    
    //The UART debug-monitor
    cmdproc #(.CLKS_PER_BIT(CLKS_PER_BIT)) debug_uart
        (.clk(cpu_clk), .rst_n(sys_rstn), .serial_rx(serial_rx), .serial_tx(serial_tx), 
         .addr(addr), .data_out(data_out), .data_in(data_in), .data_out_ready(data_out_ready),
         .data_write_complete(1'b1), .data_in_valid(data_in_valid),
         .data_imem_p_dmem_n(data_imem_p_dmem_n), .cpu_halt(cpu_halt_cpu), .cpu_step(), 
         .cpu_reset_p(cpu_reset_p));
        
    assign cpu_resetn_cpu = ~cpu_reset_p;
    
    assign cpu_imem_addr = addr;
    assign cpu_dmem_addr = addr;
    assign cpu_debug_to_imem_data = data_out;
    assign cpu_debug_to_dmem_data = data_out;
    
    assign data_in = data_imem_p_dmem_n ? cpu_imem_to_debug_data : cpu_dmem_to_debug_data;
    assign cpu_imem_we = data_imem_p_dmem_n ? data_out_ready : 0;
    assign cpu_dmem_we = data_imem_p_dmem_n ? 0 : data_out_ready;
    
    assign data_in_valid = data_imem_p_dmem_n ? cpu_imem_to_debug_data_ready : cpu_dmem_to_debug_data_ready;
    assign cpu_imem_ce = cpu_halt_cpu & data_imem_p_dmem_n;
    assign cpu_dmem_ce = cpu_halt_cpu & ~data_imem_p_dmem_n;

endmodule
