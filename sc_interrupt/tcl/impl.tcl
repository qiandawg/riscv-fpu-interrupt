set project_name "riscv-singlecycle-int"
open_project ./vivado/${project_name}.xpr

# Ensure correct top
set_property top $top_entity [get_filesets sources_1]

# ===== BUILD (SYNTH → IMPL → BIT) =====
launch_runs synth_1 -jobs 8
wait_on_run synth_1

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# Optional: write debug probes if you used ILA
# open_run impl_1
# write_debug_probes -force "$proj_dir/$project_name.runs/impl_1/${top_entity}.ltx"

close_project



