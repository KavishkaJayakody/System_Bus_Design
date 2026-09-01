#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_bus_test.tcl
#
#  Runs the tb_top_bus_system.v test cases against real hardware through the
#  In-System Sources & Probes instance "BUS0".
#
#  Usage:   quartus_stp -t issp_bus_test.tcl      <-- quartus_stp ONLY
#           (not quartus_sh, and not the Quartus GUI Tcl console)
#           quartus_stp -t issp_bus_test.tcl -gap    ;# + decode-gap hang demo
#
#  For an interactive session instead, use issp_console.tcl.
#  Requires: top_debug programmed onto the board, USB-Blaster connected, and
#  the In-System Sources & Probes Editor tab CLOSED.
#  Exits 0 if every case passes, 1 otherwise.
#
#  Bus plumbing and the source/probe bit map live in issp_bus_lib.tcl.
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set ERRORS 0
set TEST_DELAY_MS 1000 ;# hold after each test so the LEDs can be read

# The GUI Tcl console has no quartus(args); only quartus_stp passes them.
set RUN_GAP 0
if {[info exists quartus(args)]} {
    set RUN_GAP [expr {[lsearch $quartus(args) "-gap"] >= 0}]
}

# Hold between tests so the LED display is readable on the board.
proc pause {} { global TEST_DELAY_MS; if {$TEST_DELAY_MS > 0} { after $TEST_DELAY_MS } }

proc pass {msg} { puts "-> SUCCESS: $msg" }
proc fail {msg} { global ERRORS; incr ERRORS; puts "-> ERROR:   $msg" }

proc check {label got want lat} {
    if {$got == $want} {
        pass [format "%s = 0x%02X  (%d clks)" $label $got $lat]
    } else {
        fail [format "%s expected 0x%02X, got 0x%02X  (%d clks)" $label $want $got $lat]
    }
}

# ------------------------------------------------------------------ connect
puts "========================================================="
puts "   IN-SYSTEM BUS VERIFICATION  (ISSP over JTAG)"
puts "========================================================="
bus_connect
puts "Delay    : ${TEST_DELAY_MS} ms after each test"
puts ""

# ------------------------------------------------------------------- tests
puts "\[TEST 1\] M0 Fast RAM Write & Read (0x1000)..."
bus_cmd 0 1 0x1000 0xC5
lassign [bus_cmd 0 0 0x1000 0x00] ok rd lat
if {!$ok} { fail "M0 read from 0x1000 never completed (lat=$lat)" } \
     else { check "M0 read 0x1000" $rd 0xC5 $lat }

pause
puts "\n\[TEST 2\] M1 Fast RAM Write & Read (0x2000)..."
bus_cmd 1 1 0x2000 0x3B
lassign [bus_cmd 1 0 0x2000 0x00] ok rd lat
if {!$ok} { fail "M1 read from 0x2000 never completed (lat=$lat)" } \
     else { check "M1 read 0x2000" $rd 0x3B $lat }

pause
puts "\n\[TEST 3\] UART TX slave staging regs (0x3000-0x3002)..."
bus_cmd 0 1 0x3000 0xA1
bus_cmd 0 1 0x3001 0xB2
bus_cmd 0 1 0x3002 0xC3
foreach {a want nm} {0x3000 0xA1 byte0 0x3001 0xB2 byte1 0x3002 0xC3 byte2} {
    lassign [bus_cmd 0 0 $a 0x00] ok rd lat
    if {!$ok} { fail "read of $a never completed (lat=$lat)" } \
         else { check "staged $nm" $rd $want $lat }
}
# offset 3 reads back the transmitter busy flag; it must be idle here
lassign [bus_cmd 0 0 0x3003 0x00] ok rd lat
if {!$ok} { fail "UART status read never completed (lat=$lat)" } \
     else { check "UART tx idle (status)" $rd 0x00 $lat }

pause
puts "\n\[TEST 4\] Split Read & Arbitration Interleaving..."
bus_cmd 0 1 0x0010 0x7E
soft_reset
lassign [bus_cmd_pair 0 0x0010 0x00  0 0x1000 0x00] r0 r1
lassign $r0 ok0 rd0 lat0
lassign $r1 ok1 rd1 lat1
if {!$ok0} { fail "M0 split read never completed (lat=$lat0)" } \
     else { check "M0 split read 0x0010" $rd0 0x7E $lat0 }
if {!$ok1} { fail "M1 interleaved read never completed (lat=$lat1)" } \
     else { check "M1 interleaved read 0x1000" $rd1 0xC5 $lat1 }

set coll [bits [probe] 36 36]
if {$coll} {
    pass "collision flag set - both masters were in flight together"
} else {
    fail "collision flag clear - masters did not overlap, arbitration untested"
}
# Latency ordering here is NOT a correctness property: M0 has arbitration
# priority, so it is granted first and M1 absorbs the waiting. Report both,
# then measure the split overhead directly with an uncontended pair.
puts [format "   latency: M0 split=%d clks, M1 fast=%d clks" $lat0 $lat1]

lassign [bus_cmd 0 0 0x1000 0x00] okf rdf latf   ;# uncontended fast read
lassign [bus_cmd 0 0 0x0010 0x00] oks rds lats   ;# uncontended split read
if {$okf && $oks} {
    if {$lats > $latf} {
        pass "split overhead visible: 0x0010 took $lats clks vs $latf for 0x1000"
    } else {
        fail "split read ($lats clks) was not slower than a fast read ($latf clks) - did it split?"
    }
} else {
    fail "uncontended latency comparison did not complete"
}

pause

# --------------------------------------------------- optional: hang demo
# Off by default: this WEDGES the bus and only KEY[0] recovers it.
if {$RUN_GAP} {
    puts "\n\[EXTRA\] Decode-gap deadlock demo (0x2800)..."
    puts "  Addresses 0x2800-0x2FFF assert no slave select, so bus_ready"
    puts "  never rises and the master waits forever holding bus_req."
    lassign [bus_cmd 0 0 0x2800 0x00] ok rd lat
    if {$ok} {
        fail "expected a hang, but the transaction completed"
    } else {
        pass "M0 hung as predicted (latency saturated at $lat) - press KEY\[0\] to recover"
    }
}

pause

# ----------------------------------------------------------------- summary
puts "\n========================================================="
if {$ERRORS == 0} {
    puts ">> IN-SYSTEM TEST PASSED: All Interconnect Operations Correct! <<"
} else {
    puts ">> IN-SYSTEM TEST FAILED: $ERRORS error(s) detected! <<"
}
puts "========================================================="
puts "LEDs now show the last M0 read data."

bus_disconnect
script_exit [expr {$ERRORS ? 1 : 0}]
