/* RISC-V interrupt and exception handler */
/* Define symbols for CSR registers */
.equ mcause,  0x342
.equ mepc,    0x341  
.equ mstatus, 0x300
.equ mtvec,   0x305
.equ mie,     0x304        # CSR address for Machine Interrupt Enable

.text
    j    start             # entry on reset
    nop                    

exc_base:                  # exception handler
    csrr t1, mcause        # read machine cause register
    andi t2, t1, 0xf       # get exception type (bits 7:4)
    la   t3, j_table       # load jump table base address
    add  t2, t2, t3        # calculate table entry address
    lw   t2, 0(t2)         # get handler address from table
    jr   t2                # jump to handler
    nop

# ----------------------------------------------------------
# External interrupt handler
# ----------------------------------------------------------
int_entry:
    # Load current counter
    la   t0, int_counter
    lw   t1, 0(t0)
    addi t1, t1, 1          # increment counter
    sw   t1, 0(t0)

    # Write counter value to 7-seg display
    li   t2, 0xbf800000     # base address for LEDs/7-seg
    sw   t1, 0x10(t2)       # 0x10 offset: 7-segment display

    mret                   # return from interrupt
    nop

sys_entry:                 # 1. syscall (ecall) handler  
    li   t5, 10            
    li   t6,  3            
    sub  t3, t5, t6        
    nop
	
epc_plus4:
    csrr  t1, mepc         
    addi  t1, t1, 4        
    csrw  mepc, t1         
    mret                   
    nop

uni_entry:                 
    nop                    
    j     epc_plus4        
    nop
    nop
    nop

ovf_entry:                 
    nop                    
    j     epc_plus4        
    nop

# ----------------------------------------------------------
# Program entry point
# ----------------------------------------------------------
start:
    # Enable external interrupts in mie
    li   t0, (1 << 11)
    csrrs x0, mie, t0

    # Set trap vector
    li    t0, 0x08         
    csrrw x0, mtvec, t0

    # Enable global MIE in mstatus
    addi t3, zero, 0xf
    csrw mstatus, t3

    # Initialize LEDs / 7-seg
    li  a1, 0xbf800004     # switches
    li  a0, 0xbf800000     # LED / 7-seg base
    addi x9, x0, 7         
    addi x8, zero, 0x0
    sw   x8, 0xc(a0)       # enable all 8 of the 7-segment displays

# ----------------------------------------------------------
# Main loop
# ----------------------------------------------------------
readIO: 
    lw  t0, 0(a1)
    sw  t0, 0(a0)          # Switch value to LEDs
    j  readIO

# ----------------------------------------------------------
# Data section
# ----------------------------------------------------------
.data
int_counter: .word 0       # counter for external interrupts

dataspace: .word 0,0,0,0,0,0,0,0
j_table:   .word int_entry     # offset 0
           .word sys_entry     # offset 1
           .word uni_entry     # offset 2
           .word ovf_entry     # offset 3
           .word 0,0,0,0,0,0
           .word 2
           .word 0x7fffffff
           .word 0xa3
           .word 0x27
           .word 0x79
           .word 0x115
           .word 0,0,0,0,0,0,0,0
