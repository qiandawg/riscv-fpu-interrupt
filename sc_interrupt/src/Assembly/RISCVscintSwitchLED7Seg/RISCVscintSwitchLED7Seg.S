/* RISC-V interrupt and exception handler */
/* Define symbols for CSR registers */
.equ mcause,  0x342
.equ mepc,    0x341  
.equ mstatus, 0x300
.equ mtvec,   0x305
 .equ mie,   0x304        # CSR address for Machine Interrupt Enable

.text
    j    start             # entry on reset
    nop                    

exc_base:                  # exception handler
    csrr t1, mcause        # read machine cause register
    andi t2, t1, 0xf      # get exception type (bits 7:4)
    la   t3, j_table       # load jump table base address
    add  t2, t2, t3        # calculate table entry address
    lw   t2, 0(t2)         # get handler address from table
    jr   t2                # jump to handler
    nop

int_entry:                 # 0. interrupt handler
    nop                    # deal with interrupt here
    mret                   # return from interrupt
    nop

sys_entry:                 # 1. syscall (ecall) handler  
    li   t5, 10            # t5 ← 10
    li   t6,  3            # t6 ←  3
    sub  t3, t5, t6        # t7 ← 10–3 = 7  <-- proof ecall handler ran
    nop
	
epc_plus4:
    csrr  t1, mepc         # get exception PC
    addi  t1, t1, 4        # epc + 4
    csrw  mepc, t1         # mepc <- mepc + 4
    mret                   # return from exception
    nop

uni_entry:                 # 2. unimplemented inst. handler
    nop                    # do something here
    j     epc_plus4        # return
    nop
    nop
    nop

ovf_entry:                 # 3. overflow handler
    nop                    # do something here  
    j     epc_plus4        # return
    nop

start:
	    # 1) Load a 1 in bit 11 into t0
    li   t0, 1 << 11         # t0 = 0x800
    csrrs x0, mie, t0        # MIE = MIE | t0
    li    t0, 0x08      # full absolute address (exc_base Must be below 12 bits)
    csrrw x0, mtvec, t0
    addi t3, zero, 0xf     # prepare status value
    csrw mstatus, t3       # enable exceptions/interrupts
    lw   t3, 0x48(zero)    # try overflow exception
    lw   t4, 0x4c(zero)    # caused by add
	nop
	
ov:
    add  t3, t3, t4        # overflow (if overflow detection enabled)
    nop

sys:
    ecall                  # environment call (was syscall)
    nop

unimpl:
    # Use an unimplemented instruction or reserved encoding
    .word 0x0000007f       # undefined instruction encoding
    nop

int:
    addi a0, zero, 0x50    # address of data[0]
    addi a1, zero, 4       # counter
    add  t3, zero, zero    # sum <- 0
	nop

loop:
    lw   t4, 0(a0)         # load data
    addi a0, a0, 4         # address + 4
    add  t3, t3, t4        # sum
    addi a1, a1, -1        # counter - 1
    bne  a1, zero, loop    # finish?
    nop

finish:
# t3 = I/O base
# t4 = cnt (value to write to 7-segment displays)
# t5 = 0 and digit values
# t6 = 4 (bit width of digits)
# t0 = temporary variable
# t1 = concatenated value of 8 7-segment display digits

main:
   li  a1, 0xbf800004            # Read the Switches
   li  a0, 0xbf800000
   addi     x9, x0, 7         # cnt = 7-segment displays
   addi  x8, zero, 0x0      # $10 = 0
   sw    x8, 0xc(a0)      # enable all 8 of the 7-segment displays
   nop
   
readIO: 
 
    lw  t0, 0(a1)

     #srl t0, t0, 16
    sw  t0, 0(a0)               # Write the LEDs
    
    sw  t0, 0x10(a0)            # write the switch values to the 7 segment display in hex

    j  readIO
	
	nop
.data
dataspace: .word 0,0,0,0,0,0,0,0
j_table:   .word int_entry     # 0x1 << 2 = 0x4 offset
           .word sys_entry     # 0x2 << 2 = 0x8 offset  
           .word uni_entry     # 0x3 << 2 = 0xc offset
           .word ovf_entry     # 0x4 << 2 = 0x10 offset
           .word 0,0,0,0,0,0
           .word 2
           .word 0x7fffffff
           .word 0xa3
           .word 0x27
           .word 0x79
           .word 0x115
           .word 0,0,0,0,0,0,0,0