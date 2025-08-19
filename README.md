# RISC-V RV32IMF Softcore with FPU and Interrupt/CSR Extensions

**FPGA implementation of a RISC-V RV32IMF softcore extended with IEEE-754 single-precision floating-point support and Trap/Break interrupts.**  
Developed as part of an independent study at Johns Hopkins University.

---

## 📖 Overview
This project presents the **design, implementation, and verification** of a teaching-oriented RISC-V softcore extended with:

- **IEEE-754 compliant FPU** (add, sub, mul, div, sqrt)  
- **Unified Trap/CSR controller** supporting both single-cycle and pipelined microarchitectures  
- **Precise exception/interrupt handling** with flush & replay logic in a 5-stage pipeline  
- **Machine-mode CSR support** (`mtvec`, `mstatus`, `mie`, `mepc`, `mcause`)  

Correctness was validated through **directed assembly tests** and **FPGA debugging** with a Xilinx Integrated Logic Analyzer (ILA) on the Nexys A7.

📄 Full paper: [RISC-V FPU and Interrupt CPU Extensions (PDF)](docs/RISC-V_FPU_and_Interrupt_CPU_Extensions.pdf)

---

## 🚀 Key Contributions
- ✅ **Unified Trap/CSR Controller** reusable across single-cycle and pipelined designs  
- ✅ **FPU integration** with hazard detection and rounding mode support  
- ✅ **Precise traps** in pipeline (sync exceptions > interrupts, flush younger instructions)  
- ✅ **FPGA Validation** with ILA captures showing exception handling and FPU results  

---

## 📂 Repository Contents
- `docs/` – Full PDF report and diagrams  
- `rtl/` – Verilog source files (CPU core, FPU, CSR unit)  
- `test/` – Directed assembly programs and ILA waveform captures  
- `fpga/` – Bitstream and board support for Nexys A7  

---

## 🛠️ Tools & Setup
- **Hardware**: Digilent Nexys A7 FPGA  
- **Tools**: Xilinx Vivado (synthesis, P&R, ILA debug)  
- **Languages**: Verilog HDL + RISC-V assembly  

To run on FPGA:  
```bash
# Synthesize and generate bitstream in Vivado
# Program Nexys A7 with generated .bit file
