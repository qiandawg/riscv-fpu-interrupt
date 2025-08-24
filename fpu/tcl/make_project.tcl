set project_name "radio_periph_lab"
create_project ${project_name} ./vivado -part xc7z020clg400-1 -force

set proj_dir [get_property directory [current_project]]
set obj [current_project]
 
add_files -fileset constrs_1 -norecurse ./src/toplevel.xdc
add_files -fileset sources_1 -norecurse ./src/toplevel.vhd

# setup IP repository path and a couple other project options 
set_property target_language VHDL [current_project]
set_property  ip_repo_paths  ./ip_repo [current_project]
update_ip_catalog
 
# make block design

source ./tcl/design_1.tcl 

close_project

 
