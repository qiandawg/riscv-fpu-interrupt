set project_name "radio_periph_lab"
setws ./vitis
platform create -name hwplat -hw ./vivado/${project_name}.xsa
domain create -name {standalone_ps7_cortexa9_0} -display-name {standalone_ps7_cortexa9_0} -os {standalone} -proc {ps7_cortexa9_0} -runtime {cpp} -arch {32-bit} -support-app {hello_world}
platform write
platform active {hwplat}
domain active {zynq_fsbl}
domain active {standalone_ps7_cortexa9_0}
platform generate -quick

platform active hwplat
app create -name my_app
importsources -name my_app -path ./src/proc_software -linker-script
app build my_app
sysproj build -name my_app_system
