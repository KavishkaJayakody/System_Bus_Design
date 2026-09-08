#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_bus_test.tcl
#
#  Runs the tb_bus_top.v cases against real hardware through the In-System
#  Sources & Probes instance "SBUS", plus the two checks that only matter on
#  a serial bus: the address reassembled off the wire, and the frame length.
#
#  Usage:   cd Serial_System_Bus
#           quartus_stp -t tcl/issp_bus_test.tcl     <-- quartus_stp ONLY
#
#  Requires de2_top programmed onto the board, USB-Blaster connected, and the
#  In-System Sources & Probes Editor tab CLOSED.
#  Exits 0 if every case passes, 1 otherwise.
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set ERRORS 0
set TEST_DELAY_MS 600 ;# hold after each test so the LEDs can be read

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
puts "   SERIAL BUS IN-SYSTEM VERIFICATION  (ISSP over JTAG)"
puts "========================================================="
bus_connect
puts "Delay    : ${TEST_DELAY_MS} ms after each test"
puts ""

# ------------------------------------------------------------------- tests
puts "\[TEST 1\] M0 write + read back, slave 1 (0x1ABC)..."
bus_cmd 0 1 0x1ABC 0xC5
lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat
if {!$ok} { fail "M0 read from 0x1ABC never completed (lat=$lat)" } \
     else { check "M0 read 0x1ABC" $rd 0xC5 $lat }
set RD_LAT $lat

pause
puts "\n\[TEST 2\] M1 write + read back, slave 2 (0x2345)..."
bus_cmd 1 1 0x2345 0x3B
lassign [bus_cmd 1 0 0x2345 0x00] ok rd rp lat
if {!$ok} { fail "M1 read from 0x2345 never completed (lat=$lat)" } \
     else { check "M1 read 0x2345" $rd 0x3B $lat }

pause
puts "\n\[TEST 3\] Serial framing: the address frame must be exactly 16 clocks..."
set p [probe]
set fl [bits $p 90 86]
set fb [bits $p 91 91]
if {$fb} {
    fail "frame_bad is SET - some address frame was not 16 clocks (last was $fl)"
} elseif {$fl != $ADDR_W} {
    fail "frame length reads $fl clocks, expected $ADDR_W"
} else {
    pass "every frame so far was $fl clocks - serial framing is correct on silicon"
}

puts "\n\[TEST 4\] Address transport: what the bus reassembled off ONE wire..."
foreach a {0x1ABC 0x2345 0x0A5C 0x8DEF} {
    bus_cmd 0 0 $a 0x00
    set got [bits [probe] 85 70]
    if {$got == $a} {
        pass [format "sent 0x%04X, bus reassembled 0x%04X" $a $got]
    } else {
        fail [format "sent 0x%04X, bus reassembled 0x%04X" $a $got]
    }
}

pause
puts "\n\[TEST 5\] Unmapped addresses must ANSWER, not hang..."
foreach a {0x2800 0x2FFF 0x3000 0x8000 0xFFFF} {
    lassign [bus_cmd 0 0 $a 0x00] ok rd rp lat
    if {!$ok} {
        fail [format "0x%04X never completed (lat=%d) - the bus is wedged" $a $lat]
    } elseif {$rp != 1} {
        fail [format "0x%04X answered %s, expected ERROR" $a [resp_name $rp]]
    } else {
        pass [format "0x%04X answered ERROR in %d clks" $a $lat]
    }
}
# ...and the bus is still healthy afterwards
lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat
if {$ok && $rd == 0xC5} {
    pass "the very next transfer returned correct data - the bus recovered"
} else {
    fail "the bus did not recover after the unmapped run"
}

pause
puts "\n\[TEST 6\] Split transaction on slave 0..."
set_split_en 0
bus_cmd 0 1 0x0A5C 0x7E
soft_reset
set_split_en 1
lassign [bus_cmd 0 0 0x0A5C 0x00] ok rd rp lat err splits
if {!$ok} {
    fail "M0 split read never completed (lat=$lat)"
} else {
    check "M0 split read 0x0A5C" $rd 0x7E $lat
    if {$splits > 0} {
        pass "split count = $splits - the slave really did defer the transfer"
    } else {
        fail "split count is 0 - the transfer never split"
    }
    if {$lat > $RD_LAT} {
        pass "split read cost $lat clks vs $RD_LAT for a plain read"
    } else {
        fail "split read ($lat clks) was not slower than a plain read ($RD_LAT)"
    }
}
set_split_en 0

pause
puts "\n\[TEST 7\] Arbitration: both masters on the same clock edge..."
soft_reset
lassign [bus_cmd_pair 1 0x1200 0xAA 1 0x2200 0x55] r0 r1
lassign $r0 ok0 rd0 rp0 lat0
lassign $r1 ok1 rd1 rp1 lat1
if {!$ok0 || !$ok1} {
    fail "one of the two masters never completed (M0 lat=$lat0, M1 lat=$lat1)"
} else {
    pass [format "both completed - M0 %d clks, M1 %d clks" $lat0 $lat1]
}
if {[bits [probe] 69 69]} {
    pass "collision flag set - both masters were in flight together"
} else {
    fail "collision flag clear - masters did not overlap, arbitration untested"
}
# Latency ordering is NOT a correctness property: M0 has priority, so it is
# granted first and M1 absorbs the wait. Check the DATA instead.
lassign [bus_cmd 0 0 0x1200 0x00] ok rd
if {$ok && $rd == 0xAA} { pass "M0's concurrent write landed" } \
                   else { fail "M0's concurrent write did not land" }
lassign [bus_cmd 1 0 0x2200 0x00] ok rd
if {$ok && $rd == 0x55} { pass "M1's concurrent write landed" } \
                   else { fail "M1's concurrent write did not land" }

pause

# ----------------------------------------------------------------- summary
puts "\n========================================================="
if {$ERRORS == 0} {
    puts ">> IN-SYSTEM TEST PASSED: the serial bus works on silicon <<"
} else {
    puts ">> IN-SYSTEM TEST FAILED: $ERRORS error(s) detected! <<"
}
puts "========================================================="
bus_status

bus_disconnect
script_exit [expr {$ERRORS ? 1 : 0}]
