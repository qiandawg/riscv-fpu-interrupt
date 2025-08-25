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

📄 Full paper: [RISC-V FPU and Interrupt CPU Extensions (PDF)](riscv-fpu-interrupts_paper.pdf)

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
- **Tools**: Xilinx Vivado 2024.2(synthesis, P&R, ILA debug)  
- **Languages**: Verilog HDL + RISC-V assembly  

## Running on FPGA (Windows)

1. Open either the **`sc-interrupt`** or **`fpu`** project folder, depending on which project you want to run.  

2. For `sc-interrupt` Open `src/sc_interrupt_sys.v` in your text editor of choice.
3. For `fpu` Open `"/fpu/riscv-pipelined-fpu-interrupt.srcs/sources_1/imports/RISCVpipelinecode/pl_stage_id.v"` for IMEM file path,  and `/fpu/riscv-pipelined-fpu-interrupt.srcs/sources_1/imports/RISCVpipelinecode/pl_stage_mem` for DMEM file path.

4. Edit the following lines to point to the path where your local repo lives:

   `
   parameter IMEM_FILE = "C:/riscv-fpu-interrupts/sc-interrupt/src/Assembly/RISCVscintSwitchLED7Seg/imem.mem";
   parameter DMEM_FILE = "C:/riscv-fpu-interrupts/sc-interrupt/src/Assembly/RISCVscintSwitchLED7Seg/dmem.mem";
   `

   If you clone the project, You should only need to modify the before 
   `/risc-fpu-interrupts` 
   
   for instance if your local repo is in, `C:/Downloads/` , your modification result will be
   `parameter IMEM_FILE = "C:/Downloads/riscv-fpu-interrupts/sc-interrupt/src/Assembly/RISCVscintSwitchLED7Seg/imem.mem";`

5. Save the file and exit your text editor.

6. For single-cycle Double-click make_project.bat located at:
	
	`
	riscv-fpu-interrupts/make_project.bat
	`
7. For fpu, open the .xpr file
