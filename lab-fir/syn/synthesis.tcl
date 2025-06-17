
sh rm -rf ./$TOP_DIR
sh rm -rf ./$RPT_DIR
sh rm -rf ./$NET_DIR
sh mkdir ./$TOP_DIR
sh mkdir ./$RPT_DIR
sh mkdir ./$NET_DIR

define_design_lib $TOPLEVEL -path ./$TOPLEVEL
													   
#Read Design File (add your files here)
analyze -library $TOPLEVEL -format verilog $verilog_files
elaborate $TOPLEVEL -architecture verilog -library $TOPLEVEL
uniquify

current_design $TOPLEVEL
link    

source -echo $sdc_files

set_fix_multiple_port_nets -feedthroughs -outputs -constants -buffer_constants
set_fix_hold [all_clocks]

####check design####
check_design  >  ./$RPT_DIR/check_design.log  
check_timing  >  ./$RPT_DIR/check_timing.log 

# clock gating
set_clock_gating_style -max_fanout 10

### Power Optimization
# set_leakage_optimization true
# set_dynamic_optimization true

#### Synthesis all design ####
# basic ------------
compile_ultra  
## find more command in user guide 

####--------------------------Netlist-related------------------------------------

write -format ddc     -hierarchy -output ./$NET_DIR/${TOPLEVEL}_syn.ddc
write -format verilog -hierarchy -output ./$NET_DIR/${TOPLEVEL}_syn.v
write_sdf -version 1.0  -context verilog ./$NET_DIR/${TOPLEVEL}_syn.sdf
write_sdc ./$NET_DIR/${TOPLEVEL}_syn.sdc
# write_saif -output ./$NET_DIR/${TOPLEVEL}_syn.saif 
##-------------------------------------------------------------------------


####---------------------------Syntheis result reports----------------------------
report_timing -delay max -max_paths 5 > ./$RPT_DIR/report_setup_${TOPLEVEL}.out
report_timing -delay min -max_paths 5 > ./$RPT_DIR/report_hold_${TOPLEVEL}.out
report_constraint -all_violators > ./$RPT_DIR/report_violation_${TOPLEVEL}.out
report_area -hier  > ./$RPT_DIR/report_area_${TOPLEVEL}.out
report_power -hier > ./$RPT_DIR/report_power_${TOPLEVEL}.out
report_timing -path full -delay max -max_paths 10 -nworst 1 > ./$RPT_DIR/report_time_${TOPLEVEL}.out

exit
