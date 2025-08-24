# the first line constructs the vivado project, but doesn't synthesize or place+route
vivado -nolog -nojournal -mode batch -source ./tcl/make_project.tcl
# this next command runs the TCL script that makes the bitfile in vivado and exports the XSA
vivado -nolog -nojournal -mode batch -source ./tcl/impl.tcl
# this next line makes the .bit.bin file from the .bit file
bootgen -image doug.bif -arch zynq -process_bitstream bin -w on

# the next line makes the vitis workspace and builds the app thru boot.bin stage
# commenting out the Vitis section, since we will test this periph in Linux!
#xsct ./tcl/make_sw.tcl
