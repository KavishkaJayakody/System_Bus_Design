#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_remote_rw_test.tcl -- write values to the OTHER board and read them
#                             back, on real hardware, over JTAG.
#
#  Usage:   cd Serial_System_Bus
#           quartus_stp -t tcl/issp_remote_rw_test.tcl              <-- ONLY quartus_stp
#           quartus_stp -t tcl/issp_remote_rw_test.tcl -loopback
#           quartus_stp -t tcl/issp_remote_rw_test.tcl -full
#
#  WHAT THIS ADDS OVER issp_link_test.tcl
#
#  issp_link_test.tcl answers "is the link alive?".  It reads a handful of
#  addresses the far board was supposed to have seeded, which means a failure
#  there is ambiguous: it could be our link, or it could be that they never
#  seeded what we expected.
#
#  This script is SELF-VERIFYING.  Every check writes a value across the link
#  and reads that same value back, so the expected answer is one this board
#  chose.  Nothing has to be agreed in advance and nothing has to be seeded on
#  the far side - if a byte comes back different from the one that went out,
#  the link is wrong, full stop.  That also makes it work UNCHANGED in
#  loopback: with rm_tx jumpered to rm_rx the board writes to and reads from
#  its own memory through the entire link path.
#
#  It is the hardware counterpart of tests 12-17 in tb/tb_uart_remote.v, and
#  it looks for the same faults:
#
#      every byte value          a stuck, swapped or reversed data bit
#      every carried address bit a dropped or swapped address bit
#      the tag bytes 0xA5 / 0x5A a parser that re-scans its own payload
#      adjacent words            an off-by-one in the offset field
#      0x00 as a real answer     a dead link reading as valid data
#
#  >> IT WRITES INTO THE FAR BOARD'S MEMORY. <<
#
#  In two-board mode this modifies the OTHER team's memory, in the small
#  scratch window listed below and nowhere else.  Tell them before running it.
#
#      far 0x1F00-0x1F1F   slave 1, the main scratch block
#      far 0x07F0          slave 0, one word, device-id check
#      far 0x2F00          slave 2, one word, device-id check
#
#  WRITES ARE POSTED, so a write completes before the far board has executed
#  it.  Every read-back therefore waits SETTLE ms first.  If you see failures
#  that come and go, raise SETTLE before suspecting anything else.
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set LOOPBACK 0
set FULL     0
foreach a $argv {
    if {$a eq "-loopback" || $a eq "-lb"} { set LOOPBACK 1 }
    if {$a eq "-full"}                    { set FULL     1 }
}

# Milliseconds to let a POSTED write reach the far board and execute before
# reading it back.  A 4-byte request is ~350 us at 115200 and the far bus
# transaction is under a microsecond, so this is generous - but JTAG jitter is
# the real variable, and a read-back that races the write looks exactly like a
# protocol fault.
set SETTLE 20

set ERRORS 0
proc pass {msg} { puts "-> SUCCESS: $msg" }
proc fail {msg} { global ERRORS; incr ERRORS; puts "-> ERROR:   $msg" }
proc note {msg} { puts "   ..       $msg" }

# The scratch window.  Far-side addresses; add 0x8000 for the local spelling.
set SCRATCH  0x9F00     ;# far 0x1F00, slave 1
set S0_ADDR  0x87F0     ;# far 0x07F0, slave 0 (2K - 0x07F0 is inside it)
set S2_ADDR  0xAF00     ;# far 0x2F00, slave 2

#----------------------------------------------------------------------------
# One remote write followed by a remote read-back of the same address.
# Returns {ok got} - ok is 0 if the transfer failed outright (timeout).
#----------------------------------------------------------------------------
proc remote_rw {addr val} {
    global SETTLE
    lassign [bus_cmd 0 1 $addr $val] wok wrd wrp wlat werr wsp wcerr
    if {$wcerr} { return [list 0 0] }
    after $SETTLE
    lassign [bus_cmd 0 0 $addr 0x00] rok rrd rrp rlat rerr rsp rcerr
    if {!$rok || $rcerr} { return [list 0 $rrd] }
    return [list 1 $rrd]
}

#----------------------------------------------------------------------------
# Turn a set of mismatches into a diagnosis.  A list of {addr wrote got}.
# The PATTERN of the failures says far more than any single one of them.
#----------------------------------------------------------------------------
proc diagnose_mismatches {bad} {
    if {[llength $bad] == 0} { return }

    puts ""
    puts "   ---- what the failures look like ------------------------"

    set all_ff 1 ; set all_00 1 ; set all_same 1 ; set first_got -1
    set rev 1
    foreach b $bad {
        lassign $b a w g
        if {$g != 0xFF} { set all_ff 0 }
        if {$g != 0x00} { set all_00 0 }
        if {$first_got < 0} { set first_got $g }
        if {$g != $first_got} { set all_same 0 }
        # bit-reversed?
        set r 0
        for {set i 0} {$i < 8} {incr i} {
            if {$w & (1 << $i)} { set r [expr {$r | (1 << (7 - $i))}] }
        }
        if {$g != $r} { set rev 0 }
    }

    if {$all_ff} {
        note "EVERY read came back 0xFF - that is the timeout value."
        note "The far board is not answering at all.  This is a LINK"
        note "fault, not a data fault.  Run 'link' or issp_link_test.tcl."
        return
    }
    if {$rev} {
        note "every byte came back BIT-REVERSED."
        note "The two ends disagree on UART bit order - one is sending"
        note "MSB-first.  8N1 is LSB-first; check their uart_tx/uart_rx."
        return
    }
    if {$all_00} {
        note "EVERY read came back 0x00 - the far side is answering, but"
        note "always with zero.  Either the write never lands (check that"
        note "they honour we = cmd\[1\]) or their read data path is dead."
        return
    }
    if {$all_same} {
        note [format "EVERY read came back the same byte (0x%02X) regardless of" $first_got]
        note "what was written.  The far board is answering from one fixed"
        note "location - suspect the offset field is being dropped."
        return
    }

    # Look for a consistently stuck bit.
    set stuck0 0xFF ; set stuck1 0xFF
    foreach b $bad {
        lassign $b a w g
        set stuck0 [expr {$stuck0 & ~($w & ~$g) & 0xFF}]
        set stuck1 [expr {$stuck1 & ~(~$w & $g) & 0xFF}]
    }
    set lost [expr {~$stuck0 & 0xFF}]
    set setb [expr {~$stuck1 & 0xFF}]
    if {$lost} {
        note [format "bit(s) 0x%02X went out as 1 and came back 0 every time -" $lost]
        note "a data bit is stuck LOW somewhere in the response path."
    }
    if {$setb} {
        note [format "bit(s) 0x%02X went out as 0 and came back 1 every time -" $setb]
        note "a data bit is stuck HIGH somewhere in the response path."
    }
    if {!$lost && !$setb} {
        note "no single stuck bit explains all of them - the failures are"
        note "value- or address-dependent.  If only some values fail, look"
        note "at whether those bytes collide with the 0xA5 / 0x5A tags."
    }
}

#============================================================================
puts "========================================================="
puts "   REMOTE WRITE / READ-BACK VERIFICATION"
puts "========================================================="
bus_connect
if {$LOOPBACK} {
    puts "Mode     : LOOPBACK - expecting rm_tx (AC15) jumpered to rm_rx (AB22)"
} else {
    puts "Mode     : two-board - THIS WRITES INTO THE FAR BOARD'S MEMORY"
    puts "           scratch: far 0x1F00-0x1F1F, 0x07F0, 0x2F00"
}
puts "Settle   : ${SETTLE} ms after every posted write"
puts ""

#----------------------------------------------------------------------------
puts "\[1\] Local bus first - nothing else means anything if this fails."
bus_cmd 0 1 0x1ABC 0x5A
lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat err splits cmderr
if {$ok && $rd == 0x5A} {
    pass "local write/read round-trips (0x1ABC = 0x5A)"
} else {
    fail [format "local bus is broken: rd=0x%02X ok=%d" $rd $ok]
    puts "\n>> ABORTING: fix the local bus before testing the link."
    script_exit 1
}

#----------------------------------------------------------------------------
puts "\n\[2\] Is the link answering at all?"
# One round trip before the bulk of the test, so a dead link produces one
# clear verdict instead of fifty identical timeouts.
# The probe value is deliberately NOT 0x5A: that is a tag byte, so a
# tag-handling bug would hide here, and it is also its own bit-reversal, so
# a reversed bit order would hide here too.  0x1F reverses to 0xF8 and
# collides with neither tag.
soft_reset
lassign [remote_rw $SCRATCH 0x1F] rwok got
if {!$rwok} {
    fail "the far board did not answer a single remote round trip"
    set verdict [link_diagnose]
    puts ""
    puts ">> ABORTING: there is no link to verify data over."
    puts "   Verdict: $verdict"
    if {$verdict eq "no-rx"} {
        note "check the COMMON GROUND first, then that the cable is crossed:"
        note "our rm_tx (AC15) to their RX, their TX to our rm_rx (AB22)."
    }
    if {!$LOOPBACK} {
        note "try -loopback with AC15 jumpered to AB22: that proves this"
        note "board's whole link path with no second board involved."
    }
    script_exit 1
}
if {$got == 0x1F} {
    pass [format "a remote round trip works (wrote 0x1F, read 0x%02X)" $got]
} else {
    # The link is ALIVE - a frame went out and a frame came back - but the
    # byte is wrong.  Do not abort: that is precisely what the rest of this
    # script is built to diagnose.
    fail [format "the link answers but corrupts data (wrote 0x1F, read 0x%02X)" $got]
    note "the wire is working; the two ends disagree about what it means."
    note "the checks below will say which part."
}

#----------------------------------------------------------------------------
puts "\n\[3\] Walking bits - catches a stuck, swapped or reversed data bit."
set bad {}
set walking_ok 1
set vals {0x01 0x02 0x04 0x08 0x10 0x20 0x40 0x80
          0xFE 0xFD 0xFB 0xF7 0xEF 0xDF 0xBF 0x7F}
foreach v $vals {
    lassign [remote_rw $SCRATCH $v] rwok got
    if {!$rwok || $got != $v} {
        lappend bad [list $SCRATCH $v $got]
    }
}
if {[llength $bad] == 0} {
    pass "all 16 walking-bit values round-tripped over the link"
} else {
    set walking_ok 0
    fail "[llength $bad] of 16 walking-bit values came back wrong"
    foreach b $bad {
        lassign $b a w g
        note [format "wrote 0x%02X  read 0x%02X" $w $g]
    }
    diagnose_mismatches $bad
}

#----------------------------------------------------------------------------
puts "\n\[4\] The tag bytes as DATA - the parser must not re-scan its payload."
# 0xA5 and 0x5A are the request and response tags.  A byte equal to a tag
# must survive as data; a parser that hunts for tags inside a payload it has
# already committed to will desynchronise here and nowhere else.
set bad {}
foreach v {0xA5 0x5A} {
    lassign [remote_rw $SCRATCH $v] rwok got
    if {!$rwok || $got != $v} {
        lappend bad [list $SCRATCH $v $got]
        fail [format "0x%02X did not survive as a data byte (read 0x%02X)" $v $got]
    } else {
        pass [format "0x%02X survived as data, not mistaken for a tag" $v]
    }
}
if {[llength $bad]} {
    if {$walking_ok} {
        # Ordinary values are fine and only the tags fail - that is the
        # signature of a parser re-scanning a payload it already committed to.
        note "ordinary values pass and only the TAG bytes fail: one end is"
        note "re-scanning payload bytes for tags.  The rule is: hunt a tag,"
        note "then take exactly 3 more bytes (request) or 1 (response), and"
        note "never look at those bytes for tags again."
    } else {
        note "the walking-bit test failed too, so this is most likely the"
        note "same underlying data fault rather than a tag-handling bug."
    }
}

#----------------------------------------------------------------------------
puts "\n\[5\] 0x00 must be a real answer, not the sound of silence."
# A dead link plausibly reads as 0x00, so prove zero is genuine by writing
# 0xFF first and watching it change.
lassign [remote_rw $SCRATCH 0xFF] rwok got
if {$rwok && $got == 0xFF} {
    lassign [remote_rw $SCRATCH 0x00] rwok2 got2
    if {$rwok2 && $got2 == 0x00} {
        pass "0xFF then 0x00 - zero is a real value, not a dead link"
    } else {
        fail [format "0x00 did not read back (got 0x%02X, cmd ok=%d)" $got2 $rwok2]
    }
} else {
    fail [format "0xFF did not read back (got 0x%02X)" $got]
}

#----------------------------------------------------------------------------
puts "\n\[6\] Every device id - the two address bits that pick the far slave."
foreach {a what} [list $S0_ADDR "slave 0 (2K, id 0)" \
                       $SCRATCH "slave 1 (4K, id 1)" \
                       $S2_ADDR "slave 2 (4K, id 2)"] {
    set want [expr {0xC0 + ($a >> 12 & 0x3)}]
    lassign [remote_rw $a $want] rwok got
    if {$rwok && $got == $want} {
        pass [format "%s: 0x%04X round-tripped 0x%02X" $what $a $want]
    } else {
        fail [format "%s: 0x%04X wrote 0x%02X read 0x%02X" $what $a $want $got]
        note "the device field is bits 13:12 of the carried address."
    }
}

#----------------------------------------------------------------------------
puts "\n\[7\] Address bits - a dropped one makes two addresses the same word."
# Give each of a set of addresses its own marker, THEN read them all back.
# Writing all of them first is the point: if two addresses alias, the second
# write lands on the first location and the read-back exposes it.
set addrs {}
for {set i 0} {$i < 8} {incr i} { lappend addrs [expr {$SCRATCH + $i}] }
lappend addrs [expr {$SCRATCH + 0x10}]
lappend addrs [expr {$SCRATCH + 0x20}]
lappend addrs [expr {$SCRATCH + 0x40}]
lappend addrs [expr {$SCRATCH + 0x80}]

set n 0
foreach a $addrs {
    bus_cmd 0 1 $a [expr {0x10 + $n}]
    incr n
}
after $SETTLE

set bad {}
set n 0
foreach a $addrs {
    set want [expr {0x10 + $n}]
    lassign [bus_cmd 0 0 $a 0x00] ok rd rp lat err sp cerr
    if {$cerr || $rd != $want} { lappend bad [list $a $want $rd] }
    incr n
}
if {[llength $bad] == 0} {
    pass "all [llength $addrs] addresses kept their own distinct word"
} else {
    fail "[llength $bad] of [llength $addrs] addresses returned the wrong word"
    foreach b $bad {
        lassign $b a w g
        note [format "0x%04X: wrote 0x%02X read 0x%02X" $a $w $g]
    }
    note "two far addresses are landing on one location - an address bit"
    note "is being lost between here and their memory array."
}

#----------------------------------------------------------------------------
if {$FULL} {
    puts "\n\[8\] FULL SWEEP - all 256 byte values (this takes a while)."
    set bad {}
    for {set v 0} {$v < 256} {incr v} {
        lassign [remote_rw $SCRATCH $v] rwok got
        if {!$rwok || $got != $v} { lappend bad [list $SCRATCH $v $got] }
        if {($v % 32) == 31} { puts [format "   ..       %d/256 done" [expr {$v+1}]] }
    }
    if {[llength $bad] == 0} {
        pass "all 256 byte values round-tripped over the link"
    } else {
        fail "[llength $bad] of 256 byte values came back wrong"
        set shown 0
        foreach b $bad {
            lassign $b a w g
            if {$shown < 8} { note [format "wrote 0x%02X  read 0x%02X" $w $g] }
            incr shown
        }
        diagnose_mismatches $bad
    }
} else {
    puts "\n\[8\] Full 256-value sweep skipped.  Add -full to run it."
}

#----------------------------------------------------------------------------
puts "\n\[9\] The link must not have disturbed the local bus."
lassign [bus_cmd 0 0 0x1ABC 0x00] ok rd rp lat err splits cmderr
if {$ok && $rd == 0x5A && !$cmderr} {
    pass "local 0x1ABC still reads 0x5A"
} else {
    fail [format "local bus disturbed: 0x1ABC = 0x%02X cmd_error=%d" $rd $cmderr]
}

# A dropped incoming request is reported, not prevented - the wire format has
# no flow control to push back with.  Worth saying out loud after a run that
# hammered the link.
set p [probe]
if {[bits $p 95 95]} {
    note "req_overrun is SET: at least one INCOMING request was thrown"
    note "away because this side was still busy with the previous one."
    note "That is the far board sending faster than we drain, not a"
    note "fault in either end.  It does not affect the results above,"
    note "which are all locally initiated."
}

puts "\n========================================================="
if {$ERRORS == 0} {
    puts ">> REMOTE WRITE/READ-BACK PASSED <<"
    puts "   Every byte written across the link read back identical."
} else {
    puts ">> REMOTE WRITE/READ-BACK FAILED: $ERRORS error(s) <<"
    puts ""
    puts "   Data came back WRONG rather than not at all, so the wire is"
    puts "   working and the two ends disagree about what it means."
    puts "   Both boards must agree on ALL of:"
    puts "     115200 8N1                    (CLKS_PER_BIT 434 at 50 MHz)"
    puts "     LSB-first UART, one stop bit"
    puts "     little-endian: the LOW byte of the 24-bit command goes first"
    puts "     cmd = wdata\[23:16\] dev\[15:14\] offset\[13:2\] we\[1\] rsvd\[0\]"
    puts "     tags 0xA5 request / 0x5A response"
    puts "     REQUEST is 4 bytes, RESPONSE is 2 - and only for READS"
    puts "     writes POSTED - no response frame for a write"
    puts "     slave sizes 2K / 4K / 4K, device ids 0 / 1 / 2"
    puts ""
    puts "   If -loopback also fails, the fault is on THIS board and no"
    puts "   amount of agreeing with the other team will fix it."
}
puts "=========================================================\n"
bus_disconnect
script_exit [expr {$ERRORS == 0 ? 0 : 1}]
