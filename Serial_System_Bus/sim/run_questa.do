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
        shift_ser.v shift_deser.v \
        addr_decoder.v arbiter.v bus_mux.v default_slave.v system_bus.v \
        master.v uart_tx.v uart_rx.v bus_bridge.v slave.v bus_top.v \
        bus_issp_driver.v top_debug.v] {
    vlog -quiet $INC $RTL/$f
}

# altsource_probe_stub.v is SIMULATION ONLY - it stands in for the Altera
# megafunction and must never appear in the .qsf.
vlog -quiet $INC $TB/altsource_probe_stub.v

foreach f [list \
        tb_shift_ser.v tb_shift_deser.v \
        tb_addr_decoder.v tb_arbiter.v tb_bus_mux.v tb_default_slave.v \
        tb_system_bus.v tb_master.v tb_slave.v \
        tb_bus_issp_driver.v tb_integration.v tb_uart_remote.v \
        tb_top_debug.v] {
    vlog -quiet $INC $TB/$f
}

foreach tb [list \
        tb_shift_ser tb_shift_deser \
        tb_addr_decoder tb_arbiter tb_bus_mux tb_default_slave \
        tb_system_bus tb_master tb_slave \
        tb_bus_issp_driver tb_integration tb_uart_remote \
        tb_top_debug] {
    echo "### $tb"
    # -onfinish stop: a testbench's $finish must END THE SIMULATION, not the
    # simulator.  In batch mode the default is `exit', so without this the
    # first $finish kills vsim and the remaining testbenches never run.
    vsim -quiet -onfinish stop work.$tb
    run -all
    quit -sim
}

echo "Check the transcript above: every testbench must print PASSED."

# Batch runs must exit; a GUI run must NOT - `quit -f' there would close
# ModelSim on the user.
if {[batch_mode]} { quit -f }
