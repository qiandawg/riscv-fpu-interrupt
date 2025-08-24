# constraints/late_xdc.xdc

# Adjust the pattern if your ILA instance name differs from "my_ila"
set ila_cells  [get_cells -hier -quiet -filter {NAME =~ *my_ila*}]
set ila_caps   [get_cells -hier -quiet -of_objects $ila_cells -filter {REF_NAME =~ SRL* || REF_NAME =~ FD*}]
set ila_d_pins [get_pins  -quiet -of_objects $ila_caps -filter {REF_PIN_NAME == "D"}]
if {[llength $ila_d_pins]} {
  set_false_path -to $ila_d_pins
}
