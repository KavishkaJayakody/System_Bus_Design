#===========================================================================
# sim/run_questa.do -- run every testbench under ModelSim / QuestaSim.
#
#   vsim -c -do sim/run_questa.do
#
# Run it from the Serial_System_Bus directory (the paths below are relative
# to it).  Each testbench prints its own PASS/FAIL banner; this script also
# counts them and reports a single summary at the end.
#===========================================================================

if {[file exists work]} { vdel -all }
vlib work

set RTL rtl
set TB  tb

# +incdir+ is how the tool finds rtl/bus_defs.vh
set INC "+incdir+$RTL"

foreach f [list \
        addr_decoder.v arbiter.v bus_mux.v slave_mem.v default_slave.v \
        master.v bus_top.v master_prog.v reset_ctrl.v debouncer.v \
        seg7_hex.v de2_top.v] {
    vlog -quiet $INC $RTL/$f
}

foreach f [list \
        tb_addr_decoder.v tb_arbiter.v tb_bus_mux.v tb_slave_mem.v \
        tb_default_slave.v tb_master.v tb_bus_top.v tb_de2_top.v] {
    vlog -quiet $INC $TB/$f
}

set failed 0
foreach tb [list \
        tb_addr_decoder tb_arbiter tb_bus_mux tb_slave_mem \
        tb_default_slave tb_master tb_bus_top tb_de2_top] {
    echo "### $tb"
    vsim -c -quiet work.$tb
    run -all
    quit -sim
}

echo "Check the transcript above: every testbench must print PASSED."
