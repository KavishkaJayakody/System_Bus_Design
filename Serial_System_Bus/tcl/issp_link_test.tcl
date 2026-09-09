#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_link_test.tcl -- diagnose the board-to-board UART link
#
#  Usage:   cd Serial_System_Bus
#           quartus_stp -t tcl/issp_link_test.tcl            <-- quartus_stp ONLY
#           quartus_stp -t tcl/issp_link_test.tcl -loopback
#
#  issp_bus_test.tcl proves the BUS works.  This proves the LINK works, and
#  when it does not, says which end is at fault.
#
#  THE PROBLEM THIS SOLVES
#
#  A serial link between two boards fails silently.  "The remote read timed
#  out" is true of a missing ground, a baud mismatch, an unprogrammed far
#  board and a protocol disagreement alike.  The probe counters added for
#  this script tell those apart:
#
#      bytes sent = 0                  our transmitter is broken - our fault
#      bytes sent > 0, received = 0    cable, ground, or the far board
#      received > 0, no RESPONSE       connected, but we disagree on the
#                                      protocol or the baud rate
#
#  LOOPBACK MODE
#
#  With -loopback, jumper this board's rm_tx (AC15, JP5) straight to its own
#  rm_rx (AB22, JP5).  The board then answers its own remote requests out of
#  its own memory, which exercises the ENTIRE link path - client, framing,
#  the far-side parser, the server, the bus, the response - with no second
#  board involved.  A remote address then reads its own local mirror:
#  0x9ABC comes back with whatever is at local 0x1ABC.
#
#  That is the test to run FIRST.  If loopback fails, the problem is on this
#  board and no amount of cable-wiggling will help.
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set LOOPBACK 0
foreach a $argv {
    if {$a eq "-loopback" || $a eq "-lb"} { set LOOPBACK 1 }
}

set ERRORS 0
proc pass {msg} { puts "-> SUCCESS: $msg" }
proc fail {msg} { global ERRORS; incr ERRORS; puts "-> ERROR:   $msg" }
proc note {msg} { puts "   ..       $msg" }

puts "========================================================="
puts "   BOARD-TO-BOARD LINK DIAGNOSIS"
puts "========================================================="
bus_connect
if {$LOOPBACK} {
    puts "Mode     : LOOPBACK - expecting rm_tx (AC15) jumpered to rm_rx (AB22)"
} else {
    puts "Mode     : two-board - expecting a crossed cable and a common ground"
}
puts ""

# ---------------------------------------------------------------------------
puts "\[1\] The local bus first - if this fails, nothing else means anything."
bus_cmd 0 1 0x1ABC 0x5A
lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat err splits cmderr
if {$ok && $rd == 0x5A} {
    pass "local write/read round-trips (0x1ABC = 0x5A)"
} else {
    fail [format "local bus is broken: rd=0x%02X ok=%d - fix that first" $rd $ok]
    puts "\n>> ABORTING: the link cannot be debugged over a broken bus."
    script_exit 1
}

# Seed the three slaves so a remote read has something recognisable to find.
bus_cmd 0 1 0x05C3 0xD0        ;# slave 0, 2K, device id 0
bus_cmd 0 1 0x1ABC 0xD1        ;# slave 1, 4K, device id 1
bus_cmd 0 1 0x2567 0xD2        ;# slave 2, 4K, device id 2
pass "seeded local 0x05C3=0xD0, 0x1ABC=0xD1, 0x2567=0xD2"

# ---------------------------------------------------------------------------
puts "\n\[2\] Link state before we send anything."
soft_reset
link_status

# ---------------------------------------------------------------------------
puts "\n\[3\] One remote READ of 0x9ABC (= the far board's 0x1ABC)."
set t0 [clock milliseconds]
lassign [bus_cmd 0 0 0x9ABC 0x00] ok rd rp lat err splits cmderr
set dt [expr {[clock milliseconds] - $t0}]

if {!$ok} {
    fail "the master never completed at all - RESP_TIMEOUT did not fire"
    puts "     That is a fault in bus_bridge, not in the link."
    link_status
    script_exit 1
}

if {$cmderr} {
    puts "   ..       timed out after ${dt} ms, returned 0x[format %02X $rd]"
    set verdict [link_diagnose]
    if {$verdict eq "ok"} {
        fail "responses were parsed, yet this read still timed out"
    } else {
        fail "no answer from the far board (${verdict})"
    }
} else {
    pass [format "answered in %d ms with 0x%02X" $dt $rd]
    link_status
    if {$LOOPBACK} {
        if {$rd == 0xD1} {
            pass "loopback correct: 0x9ABC returned our own 0x1ABC (0xD1)"
            pass "the WHOLE link path works on this board - client, framing,"
            note "parser, server, bus and response are all good."
        } else {
            fail [format "loopback returned 0x%02X, expected 0xD1 (our 0x1ABC)" $rd]
            note "bytes flow but the command is being decoded wrong - suspect"
            note "byte order or the dev/offset packing."
        }
    } else {
        note "that byte came from the FAR board's 0x1ABC."
        note "If it is not what they seeded, check the device-id mapping:"
        note "  0x8xxx -> their slave 0 (2K)   0x9xxx -> slave 1 (4K)"
        note "  0xAxxx -> their slave 2 (4K)"
    }
}

# ---------------------------------------------------------------------------
puts "\n\[4\] Each device id in turn."
foreach {a what want} {0x85C3 "slave 0 (2K, id 0)" 0xD0
                       0x9ABC "slave 1 (4K, id 1)" 0xD1
                       0xA567 "slave 2 (4K, id 2)" 0xD2} {
    lassign [bus_cmd 0 0 $a 0x00] ok rd rp lat err splits cmderr
    if {$cmderr} {
        fail [format "%s: 0x%04X timed out" $what $a]
    } elseif {$LOOPBACK && $rd != $want} {
        fail [format "%s: 0x%04X = 0x%02X, loopback expected 0x%02X" $what $a $rd $want]
    } else {
        pass [format "%s: 0x%04X = 0x%02X" $what $a $rd]
    }
}

# ---------------------------------------------------------------------------
puts "\n\[5\] A remote WRITE is POSTED - it must retire fast and answer nothing."
set t0 [clock milliseconds]
lassign [bus_cmd 0 1 0x9ABC 0x3C] ok rd rp lat err splits cmderr
set dt [expr {[clock milliseconds] - $t0}]
if {$cmderr} {
    fail "the posted write reported an error - it should never wait for a reply"
} else {
    pass "posted write retired in ${dt} ms without waiting for a response"
}
after 50   ;# let the far side actually execute it before reading back

if {$LOOPBACK} {
    lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat err splits cmderr
    if {$rd == 0x3C} {
        pass "the posted write really landed (local 0x1ABC = 0x3C)"
    } else {
        fail [format "posted write did not land: 0x1ABC = 0x%02X, wanted 0x3C" $rd]
    }
}

# ---------------------------------------------------------------------------
puts "\n\[6\] The local bus must be untouched by all of that."
lassign [bus_cmd 0 0 0x2567 0x00] ok rd rp lat err splits cmderr
if {$ok && $rd == 0xD2 && !$cmderr} {
    pass "local 0x2567 still reads 0xD2 - the link cannot wedge the bus"
} else {
    fail [format "local bus disturbed: 0x2567 = 0x%02X cmd_error=%d" $rd $cmderr]
}

puts "\n========================================================="
if {$ERRORS == 0} {
    puts ">> LINK TEST PASSED <<"
} else {
    puts ">> LINK TEST FAILED: $ERRORS error(s) <<"
    puts ""
    puts "   Both boards must agree on ALL of:"
    puts "     115200 8N1                    (CLKS_PER_BIT 434 at 50 MHz)"
    puts "     slave sizes 2K / 4K / 4K, device ids 0 / 1 / 2"
    puts "     tags 0xA5 request / 0x5A response, little-endian"
    puts "     writes POSTED - no response frame for a write"
    puts "     10 ms response timeout"
    puts "     crossed cabling AND A COMMON GROUND"
}
puts "=========================================================\n"
script_exit [expr {$ERRORS == 0 ? 0 : 1}]
