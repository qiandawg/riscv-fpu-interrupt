call c:\Xilinx\Vivado\2024.2\settings64.bat
call vivado -nolog -nojournal -mode=batch -source=./tcl/make_project.tcl
call vivado -nolog -nojournal -mode=batch -source=./tcl/impl.tcl
:: commenting out the vitis part since we will be testing in linux
::call xsct ./tcl/make_sw.tcl
::next line makes a .bit.bin file from the existing .bit file, for use with fpgautil
call bootgen -image doug.bif -arch zynq -process_bitstream bin -w on

