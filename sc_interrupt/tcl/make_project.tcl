# ===== CONFIG =====
set project_name "riscv-singlecycle-int"
set top_entity   "mfp_nexys4_ddr"     ;# <-- your actual top module name
set part         "xc7a100tcsg324-1"   ;# Nexys A7-100T (use xc7a50tcsg324-1 for 50T)
set inc_dirs [list ./src]

# ===== PROJECT =====
create_project $project_name ./vivado -part $part -force
set proj_dir [get_property directory [current_project]]

# (Optional) language settings
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

# ===== SOURCES =====
# HDL
set verilog_files [glob -nocomplain ./src/*.v]
add_files -fileset sources_1 -norecurse $verilog_files

# Headers (.vh / .svh)
set header_files [concat \
  [glob -nocomplain ./src/*.vh] \
  [glob -nocomplain ./src/*.svh] \
]
if {[llength $header_files] > 0} {
  add_files -fileset sources_1 -norecurse $header_files
  # Mark them explicitly as headers
  set_property file_type {Verilog Header} [get_files $header_files]
  # Optional: include dirs if your code uses relative includes
  # add more paths (semicolon-separated) if you have subfolders
  
  set_property include_dirs $inc_dirs [get_filesets sources_1]
}

# Constraints
add_files -fileset constrs_1 -norecurse ./src/mfp_nexys4_ddr.xdc
# NEW: add the late XDC and force it to the end
add_files -fileset constrs_1 -norecurse ./src/late_xdc.xdc
set_property USED_IN {synthesis implementation} [get_files ./src/late_xdc.xdc]
# Try to set processing order to LATE (ignored if not supported), and push to back
catch { set_property PROCESSING_ORDER LATE [get_files ./src/late_xdc.xdc] }
reorder_files -fileset constrs_1 -back [get_files ./src/late_xdc.xdc]



# ===== (Optional) DRC severity overrides =====
# Prefer fixing the XDC; keep these only if you must unblock builds
#set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
#set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

# ===== IP: add existing .xci instances from ./ip_repo/*/* =====
# 1) Let Vivado know where your packaged IPs live (good hygiene)
set_property ip_repo_paths ./ip_repo [current_project]
update_ip_catalog

# 2) Add only the configured IP instances (.xci) — no recursion/junk
set ip_xci_list [concat \
  [glob -nocomplain ./ip_repo/*/*.xci] \
  [glob -nocomplain ./ip_repo/*/*/*.xci] \
]
if {[llength $ip_xci_list] > 0} {
  add_files -fileset sources_1 -norecurse $ip_xci_list
  # 3) Make sure they’re current and generate their outputs
  set ips [get_ips -quiet *]
  if {[llength $ips] > 0} {
    upgrade_ip    $ips
    generate_target all $ips
    # Optional but helpful in project mode:
    # export_ip_user_files -of_objects $ips -no_script -sync -force -quiet
  }
}



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
