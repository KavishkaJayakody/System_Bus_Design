#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_bus_test.tcl
#
#  Runs the tb_top_bus_system.v test cases against real hardware, driving the
#  bus through the In-System Sources & Probes instance "BUS0" in
#  bus_issp_driver.v.
#
#  Usage:   quartus_stp -t issp_bus_test.tcl      <-- quartus_stp ONLY
#           (not quartus_sh, and not the Quartus GUI Tcl console)
#           quartus_stp -t issp_bus_test.tcl -gap    ;# + decode-gap hang demo
#
#  Requires: top_debug programmed onto the board, USB-Blaster connected.
#  Exits 0 if every case passes, 1 otherwise.
#
#  Source map (50 bits)            Probe map (38 bits)
#    [0]      m0_go                  [7:0]    m0_rdata
#    [1]      m0_we                  [8]      m0_done   (sticky)
#    [15:2]   m0_addr                [9]      m0_busy
#    [23:16]  m0_wdata               [17:10]  m1_rdata
#    [24]     m1_go                  [18]     m1_done   (sticky)
#    [25]     m1_we                  [19]     m1_busy
#    [39:26]  m1_addr                [27:20]  m0_lat
#    [47:40]  m1_wdata               [35:28]  m1_lat
#    [48]     soft_rst               [36]     collision (sticky)
#    [49]     reserved               [37]     reserved
# ===========================================================================

set SRC        0      ;# shadow copy of the 50-bit source register
set ERRORS     0
set POLL_LIMIT 200    ;# probe reads before declaring a transaction hung
set TEST_DELAY_MS 1000 ;# hold after each test so the LEDs can be read
# The GUI Tcl console has no quartus(args); only quartus_stp passes them.
set RUN_GAP 0
if {[info exists quartus(args)]} {
    set RUN_GAP [expr {[lsearch $quartus(args) "-gap"] >= 0}]
}

# Detect the full Quartus GUI. There, a bare 'exit' would close the whole
# application, so unwind the script instead.
set IN_GUI 0
if {[info exists quartus(nameofexecutable)]} {
    set IN_GUI [expr {$quartus(nameofexecutable) eq "quartus"}]
}

# ---------------------------------------------------------------- utilities
proc script_exit {code} {
    global IN_GUI
    if {$IN_GUI} { return -code return } else { exit $code }
}

proc bits {v hi lo} {
    expr {($v >> $lo) & ((1 << ($hi - $lo + 1)) - 1)}
}

proc src_field {lo width val} {
    global SRC
    set mask [expr {((1 << $width) - 1) << $lo}]
    set SRC  [expr {($SRC & ~$mask) | (($val << $lo) & $mask)}]
}

proc src_flush {} {
    global SRC ISSP
    write_source_data -instance_index $ISSP -value [format %x $SRC] -value_in_hex
}

proc probe {} {
    global ISSP
    return [expr 0x[read_probe_data -instance_index $ISSP -value_in_hex]]
}

# Per-master probe field offsets: {rdata_lo done busy lat_lo}
proc pfields {m} {
    if {$m == 0} { return {0 8 9 20} } else { return {10 18 19 28} }
}

# Hold between tests so the LED display is readable on the board.
proc pause {} { global TEST_DELAY_MS; if {$TEST_DELAY_MS > 0} { after $TEST_DELAY_MS } }

proc pass {msg} { puts "-> SUCCESS: $msg" }
proc fail {msg} { global ERRORS; incr ERRORS; puts "-> ERROR:   $msg" }

# ------------------------------------------------------------ bus commands
# Arm one master's command fields without firing it.
proc arm {m we addr wdata} {
    set b [expr {$m * 24}]
    src_field $b            1 0          ;# go low - clears sticky done
    src_field [expr {$b+1}] 1 $we
    src_field [expr {$b+2}] 14 $addr
    src_field [expr {$b+16}] 8 $wdata
}

# Wait for a master to report done. Returns {ok rdata latency}.
proc await {m} {
    global POLL_LIMIT
    lassign [pfields $m] rlo dbit bbit llo
    for {set i 0} {$i < $POLL_LIMIT} {incr i} {
        set p [probe]
        if {[bits $p $dbit $dbit] == 1} {
            return [list 1 [bits $p [expr {$rlo+7}] $rlo] [bits $p [expr {$llo+7}] $llo]]
        }
    }
    set p [probe]
    return [list 0 [bits $p [expr {$rlo+7}] $rlo] [bits $p [expr {$llo+7}] $llo]]
}

# Single transaction on one master. Returns {ok rdata latency}.
proc bus_cmd {m we addr wdata} {
    arm $m $we $addr $wdata
    src_flush
    src_field [expr {$m * 24}] 1 1        ;# rising edge fires it
    src_flush
    set r [await $m]
    src_field [expr {$m * 24}] 1 0        ;# release, clears sticky done
    src_flush
    return $r
}

# Fire BOTH masters on the same clock edge. This is the only way to observe
# arbitration from JTAG: a single source write updates all 50 bits at once,
# so both go edges land together. Issuing them as two separate writes would
# leave milliseconds between them and the first would always finish first.
proc bus_cmd_pair {we0 a0 d0 we1 a1 d1} {
    arm 0 $we0 $a0 $d0
    arm 1 $we1 $a1 $d1
    src_flush
    src_field 0  1 1
    src_field 24 1 1
    src_flush                              ;# both masters launch together
    set r0 [await 0]
    set r1 [await 1]
    src_field 0  1 0
    src_field 24 1 0
    src_flush
    return [list $r0 $r1]
}

proc soft_reset {} {
    src_field 48 1 1 ; src_flush
    src_field 48 1 0 ; src_flush
}

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

# The JTAG / ISSP Tcl packages exist ONLY in quartus_stp. Verified on Quartus
# 24.1std: quartus_sh reports them as unavailable, and the GUI Tcl console does
# not provide them at all - no load_package or package require can help there.
if {[catch {load_package insystem_source_probe}] || [catch {load_package jtag}]} {
    puts "FATAL: the JTAG / In-System Sources & Probes Tcl packages are not"
    puts "       available in this interpreter."
    puts ""
    puts "       Run this script from a terminal with quartus_stp:"
    puts ""
    puts "           quartus_stp -t issp_bus_test.tcl"
    puts ""
    puts "       It cannot run in the Quartus GUI Tcl console, via Tools >"
    puts "       Tcl Scripts, or under quartus_sh."
    script_exit 1
}

if {[catch {set hwlist [get_hardware_names]} err]} {
    puts "FATAL: could not query programming hardware."
    puts "       $err"
    script_exit 1
}
if {[llength $hwlist] == 0} {
    puts "FATAL: no programming hardware found. Is the USB-Blaster plugged in?"
    script_exit 1
}

# Pick the first cable that actually has devices on it. get_hardware_names can
# also list unreachable remote servers, which must be skipped.
set hw ""; set dev ""
foreach cand $hwlist {
    if {[catch {set devs [get_device_names -hardware_name $cand]}]} { continue }
    if {[llength $devs] > 0} { set hw $cand; set dev [lindex $devs 0]; break }
    puts "  (skipping \"$cand\" - no devices)"
}
if {$hw eq ""} {
    puts "FATAL: none of these cables had a device on the chain:"
    foreach cand $hwlist { puts "         $cand" }
    script_exit 1
}
puts "Hardware : $hw"
puts "Delay    : ${TEST_DELAY_MS} ms after each test"
puts "Device   : $dev"

# Enumerate instances BEFORE opening a session: this query opens a transient
# session of its own, so calling it while one is active fails.
set ISSP -1
if {[catch {set insts [get_insystem_source_probe_instance_info \
                  -hardware_name $hw -device_name $dev]} err]} {
    puts "FATAL: could not enumerate ISSP instances."
    puts "       $err"
    puts ""
    puts "       If that mentions an active session, the Quartus GUI's In-System"
    puts "       Sources & Probes Editor has this device open. Close that tab"
    puts "       (the GUI itself can stay open) and re-run."
    script_exit 1
}
foreach inst $insts {
    lassign $inst idx swidth pwidth name
    puts "Instance : index $idx  \"$name\"  source=$swidth probe=$pwidth"
    if {$swidth == 50 && $pwidth == 38} { set ISSP $idx }
}
if {$ISSP < 0} {
    puts "FATAL: no 50-bit source / 38-bit probe instance found."
    puts "       Is the current top_debug actually programmed onto this device?"
    script_exit 1
}

if {[catch {start_insystem_source_probe \
              -hardware_name $hw -device_name $dev} err]} {
    puts "FATAL: could not open an ISSP session."
    puts "       $err"
    script_exit 1
}
puts ""

set SRC 0
src_flush
soft_reset

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
puts "\n\[TEST 3\] M0 Bridge Reg Read Default (0x3000)..."
lassign [bus_cmd 0 0 0x3000 0x00] ok rd lat
if {!$ok} { fail "M0 bridge read never completed (lat=$lat)" } \
     else { check "M0 bridge default" $rd 0xBE $lat }

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

end_insystem_source_probe
script_exit [expr {$ERRORS ? 1 : 0}]
