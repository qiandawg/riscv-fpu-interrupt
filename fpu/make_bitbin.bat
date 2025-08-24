: this file assumes you have a bitfile in the impl_1 directory, and turns it into a bit.bin file
: for fpgautil.  Note that you don't need to run this batch file if you've already run "make_project.bat"
: the use for this file is if you have generated a .bit file from the GUI and wanted to turn it into a .bit.bin
call c:\Xilinx\Vivado\2024.2\settings64.bat
call bootgen -image doug.bif -arch zynq -process_bitstream bin -w on

