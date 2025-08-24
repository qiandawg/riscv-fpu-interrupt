set project_name "radio_periph_lab"
open_project ./vivado/${project_name}.xpr


# build the bitstream
update_compile_order -fileset sources_1
launch_runs impl_1 -to_step write_bitstream -jobs 7
wait_on_run impl_1

# export the XSA file for software development
write_hw_platform -fixed -include_bit -force -file ./vivado/${project_name}.xsa

close_project