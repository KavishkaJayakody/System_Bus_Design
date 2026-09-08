# ===========================================================================
#  issp_bus_lib.tcl -- shared plumbing for driving the SERIAL system bus over
#                      JTAG through the In-System Sources & Probes instance
#                      "SBUS".
#
#  Sourced by issp_console.tcl (interactive) and issp_bus_test.tcl (scripted).
#  Keeping the bit map in one place stops the two drifting.
#
#  This must agree with the header of rtl/bus_issp_driver.v and with the probe
#  assembly at the bottom of that file - three places, one bit map.
#
#  SOURCE map (56 bits)              PROBE map (128 bits)
#    per master, base = m*26           per master, base = m*30
#      [b+0]      go                     [b+7:b+0]    rdata
#      [b+1]      we                     [b+8]        done   (sticky)
#      [b+17:b+2] addr   (16)            [b+9]        busy
#      [b+25:b+18] wdata (8)             [b+11:b+10]  resp
#    m0 = src[25:0]  m1 = src[51:26]     [b+12]       err    (sticky)
#                                        [b+20:b+13]  lat
#    [52] soft_rst                       [b+28:b+21]  splits
#    [53] issp_mode                      [b+29]       cmd_error (sticky,
#    [54] split_en                                    remote timeout, M0 only)
#    [55] spare       (remote is      m0 = prb[29:0]  m1 = prb[59:30]
#         chosen by the ADDRESS,
#         not a command bit)            [61:60]  gnt
#                                        [63:62]  split_mask
#                                        [67:64]  sel_q {def,s2,s1,s0}
#                                        [68]     split_busy
#                                        [69]     collision (sticky)
#                                        [85:70]  bus_addr  (reassembled)
#                                        [90:86]  frame_len (must be 16)
#                                        [91]     frame_bad (sticky)
#                                        [92]     remote_busy
#                                        [93]     srv_busy
#                                        [94]     rx_active (line not idle)
#                                        [95]     req_overrun (sticky: an
#                                                 incoming REQUEST was thrown
#                                                 away - no flow control)
#  LINK DIAGNOSTICS (probe is 128 bits)
#    [103:96]  rx_last     last byte framed
#    [111:104] rx_count    bytes received (wraps)
#    [119:112] tx_count    bytes sent (wraps)
#    [121:120] rx_state    0=hunting 1=in REQ 2=in RESP
#    [122]     req_seen    sticky: parsed a whole REQUEST
#    [123]     resp_seen   sticky: parsed a whole RESPONSE
# ===========================================================================

set SRC        0      ;# shadow copy of the 56-bit source register
set ISSP       -1
# How long to wait for one transaction before declaring the bus hung.
#
# This is a WALL-CLOCK budget, not a count of probe reads.  A count is not a
# time: how long N reads take depends on the USB-Blaster's clock and the host,
# and the split slave's SPLIT_LATENCY is 10,000,000 clocks - 0.2 s at 50 MHz - so a
# split read legitimately outlasts a few hundred JTAG reads on a fast cable.
# Counting reads made TEST 6 fail on hardware for no reason but cable speed.
set POLL_MS 2000      ;# ms to wait for `done' before declaring a hang
set ADDR_W     16

# Detect the full Quartus GUI: a bare 'exit' there closes the application.
set IN_GUI 0
if {[info exists quartus(nameofexecutable)]} {
    set IN_GUI [expr {$quartus(nameofexecutable) eq "quartus"}]
}

proc script_exit {code} {
    global IN_GUI
    if {$IN_GUI} { return -code return } else { exit $code }
}

# ---------------------------------------------------------------- bit twiddling
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

# ------------------------------------------------------------- link diagnosis
# What is actually happening on the wire.  Read this FIRST when a remote
# transaction fails: it separates "nothing is arriving" from "something is
# arriving that we cannot parse", which are completely different faults.
proc link_status {} {
    set p [probe]
    set rxc  [bits $p 111 104]
    set txc  [bits $p 119 112]
    set last [bits $p 103 96]
    set st   [bits $p 121 120]
    set reqs [bits $p 122 122]
    set rsps [bits $p 123 123]
    set act  [bits $p 94 94]
    set ovr  [bits $p 95 95]
    set rb   [bits $p 92 92]
    set sb   [bits $p 93 93]
    set names {"hunting for a tag" "collecting a REQUEST" "collecting a RESPONSE" "?"}

    puts ""
    puts "  ---- board-to-board link ----------------------------------"
    puts [format "  bytes sent      %3d   (wraps at 256)" $txc]
    puts [format "  bytes received  %3d   last byte 0x%02X" $rxc $last]
    puts [format "  parser state    %s" [lindex $names $st]]
    puts [format "  parsed a REQUEST  %s      parsed a RESPONSE  %s" \
              [expr {$reqs ? "yes" : "no "}] [expr {$rsps ? "yes" : "no "}]]
    puts [format "  rx line         %s" \
              [expr {$act ? "LOW - someone is driving it right now" : "idle high"}]]
    puts [format "  remote_busy %d   srv_busy %d" $rb $sb]
    if {$ovr} {
        puts "  ** REQUEST OVERRUN - a request was thrown away because this"
        puts "     side was still busy with the previous one.  The link has"
        puts "     no flow control; slow the far board down."
    }
    puts ""
    return [list $txc $rxc $last $st $reqs $rsps $ovr]
}

# Turn the counters into a diagnosis.  Call after attempting a remote read.
proc link_diagnose {} {
    lassign [link_status] txc rxc last st reqs rsps ovr
    if {$txc == 0} {
        puts "  >> WE ARE NOT TRANSMITTING.  No byte ever reached uart_tx."
        puts "     That is a fault on THIS board, not the cable."
        return "no-tx"
    }
    if {$rxc == 0} {
        puts "  >> NOTHING IS ARRIVING.  We transmit, they say nothing."
        puts "     In order of likelihood:"
        puts "       - cable not crossed: our rm_tx (AC15, JP5) must reach"
        puts "         THEIR rm_rx, and theirs must reach our rm_rx (AB22)"
        puts "       - NO COMMON GROUND between the boards"
        puts "       - the far board is not programmed or is held in reset"
        puts "       - baud so far off that no start bit ever frames"
        return "no-rx"
    }
    if {!$rsps} {
        puts "  >> BYTES ARE ARRIVING BUT NEVER PARSE INTO A RESPONSE."
        puts [format "     %d bytes in, last 0x%02X.  The wire is connected;" $rxc $last]
        puts "     the disagreement is in the protocol or the baud rate:"
        puts "       - baud slightly off (we are 115200 8N1, CLKS_PER_BIT 434)"
        puts "       - they answer writes too (spec says writes are POSTED)"
        puts "       - tag mismatch: we expect 0xA5 request / 0x5A response"
        puts "       - byte order: low byte of the command goes FIRST"
        if {$last == 0xA5 || $last == 0x5A} {
            puts "     NOTE: the last byte IS a tag, so framing is probably fine"
            puts "     and the payload length is what disagrees."
        }
        return "no-parse"
    }
    if {$ovr} {
        puts "  >> The link works, but at least one REQUEST was DROPPED."
        puts "     The far board sent faster than this side could drain."
        return "overrun"
    }
    puts "  >> The link is alive: whole RESPONSE frames are being parsed."
    return "ok"
}

# ---------------------------------------------------------------- remote link
# There is no "remote" source bit: the ADDRESS selects the far board.  Any
# address with bit 15 set leaves over the UART and is executed on the other
# board, at (address - 0x8000).  Writes are posted; a read with no cable
# returns 0xFF and sets cmd_error (prb[b+29]) after about 10 ms.
#
#   0x8000-0x87FF -> far slave 0 (2K, id 0)
#   0x9000-0x9FFF -> far slave 1 (4K, id 1)
#   0xA000-0xAFFF -> far slave 2 (4K, id 2)

# ---------------------------------------------------------------- bus commands
proc arm {m we addr wdata} {
    set b [expr {$m * 26}]
    src_field $b              1 0        ;# go low - clears the sticky done
    src_field [expr {$b+1}]   1 $we
    src_field [expr {$b+2}]  16 $addr
    src_field [expr {$b+18}]  8 $wdata
}

# Wait for a master to report done.
# Returns {ok rdata resp lat err splits cmd_error}.
proc await {m} {
    global POLL_MS
    set b [expr {$m * 30}]
    set deadline [expr {[clock milliseconds] + $POLL_MS}]
    while {[clock milliseconds] < $deadline} {
        set p [probe]
        if {[bits $p [expr {$b+8}] [expr {$b+8}]] == 1} {
            return [list 1 \
                [bits $p [expr {$b+7}]  $b]            \
                [bits $p [expr {$b+11}] [expr {$b+10}]] \
                [bits $p [expr {$b+20}] [expr {$b+13}]] \
                [bits $p [expr {$b+12}] [expr {$b+12}]] \
                [bits $p [expr {$b+28}] [expr {$b+21}]] \
                [bits $p [expr {$b+29}] [expr {$b+29}]]]
        }
    }
    set p [probe]
    return [list 0 [bits $p [expr {$b+7}] $b] 0 \
                   [bits $p [expr {$b+20}] [expr {$b+13}]] 0 \
                   [bits $p [expr {$b+28}] [expr {$b+21}]] 0]
}

# One transaction on one master.
# Returns {ok rdata resp lat err splits cmd_error}.
proc bus_cmd {m we addr wdata} {
    arm $m $we $addr $wdata
    src_flush
    src_field [expr {$m * 26}] 1 1        ;# rising edge fires it
    src_flush
    set r [await $m]
    src_field [expr {$m * 26}] 1 0        ;# release, re-arms for the next one
    src_flush
    return $r
}

# Fire BOTH masters on the same clock edge. A single source write updates all
# 56 bits at once, so both go edges land together; two separate writes would
# leave milliseconds between them and never overlap.
proc bus_cmd_pair {we0 a0 d0 we1 a1 d1} {
    arm 0 $we0 $a0 $d0
    arm 1 $we1 $a1 $d1
    src_flush
    src_field 0  1 1
    src_field 26 1 1
    src_flush
    set r0 [await 0]
    set r1 [await 1]
    src_field 0  1 0
    src_field 26 1 0
    src_flush
    return [list $r0 $r1]
}

proc soft_reset {} {
    src_field 52 1 1 ; src_flush
    src_field 52 1 0 ; src_flush
}

proc set_split_en {on} { src_field 54 1 $on ; src_flush }

proc resp_name {r} {
    switch -- $r {
        0 { return "OKAY"  }
        1 { return "ERROR" }
        2 { return "SPLIT" }
        default { return "rsvd" }
    }
}

proc sel_name {s} {
    switch -- $s {
        1 { return "slave 0" }
        2 { return "slave 1" }
        4 { return "slave 2" }
        8 { return "default" }
        0 { return "none"    }
        default { return [format "0b%04b??" $s] }
    }
}

# Snapshot of the bus-side probes - the part that is about the BUS rather
# than about one master.
proc bus_status {} {
    global ADDR_W
    set p [probe]
    puts ""
    puts [format "  grant        %02b        split mask  %02b" \
              [bits $p 61 60] [bits $p 63 62]]
    puts [format "  responder    %s   (sel_q %04b)" \
              [sel_name [bits $p 67 64]] [bits $p 67 64]]
    puts [format "  split slave  %s" \
              [expr {[bits $p 68 68] ? "BUSY - a split is in flight" : "idle"}]]
    puts [format "  collision    %d         (sticky: both masters in flight at once)" \
              [bits $p 69 69]]
    puts [format "  last address 0x%04X    reassembled off the serial wire" \
              [bits $p 85 70]]
    set fl [bits $p 90 86]
    set fb [bits $p 91 91]
    if {$fb} {
        puts [format "  frame length %d clocks  ** frame_bad SET - some frame was not %d **" \
                  $fl $ADDR_W]
    } else {
        puts [format "  frame length %d clocks  (expected %d - serial framing is correct)" \
                  $fl $ADDR_W]
    }
    puts [format "  uart link    %s%s" \
              [expr {[bits $p 92 92] ? "remote transaction OUTSTANDING" : "idle"}] \
              [expr {[bits $p 93 93] ? ", SERVING the other board" : ""}]]
    puts ""
    puts [format "  m0  rdata 0x%02X  resp %-5s  lat %3d  err %d  splits %d  cmd_error %d" \
              [bits $p 7 0] [resp_name [bits $p 11 10]] [bits $p 20 13] \
              [bits $p 12 12] [bits $p 28 21] [bits $p 29 29]]
    puts [format "  m1  rdata 0x%02X  resp %-5s  lat %3d  err %d  splits %d" \
              [bits $p 37 30] [resp_name [bits $p 41 40]] [bits $p 50 43] \
              [bits $p 42 42] [bits $p 58 51]]
    puts ""
}

# ---------------------------------------------------------------- connection
# Opens the ISSP session, sets ::ISSP and takes the command ports.
proc bus_connect {} {
    global ISSP SRC

    # These packages exist ONLY in quartus_stp. Verified on Quartus 24.1std:
    # quartus_sh rejects them and the GUI Tcl console lacks them entirely.
    if {[catch {load_package insystem_source_probe}] || [catch {load_package jtag}]} {
        puts "FATAL: the JTAG / In-System Sources & Probes Tcl packages are not"
        puts "       available in this interpreter."
        puts ""
        puts "       Run from a terminal with quartus_stp, e.g."
        puts "           quartus_stp -t tcl/issp_console.tcl"
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

    # Pick the first cable that actually has devices; get_hardware_names can
    # also list unreachable remote servers.
    set hw ""; set dev ""
    foreach cand $hwlist {
        if {[catch {set devs [get_device_names -hardware_name $cand]}]} { continue }
        if {[llength $devs] > 0} { set hw $cand; set dev [lindex $devs 0]; break }
    }
    if {$hw eq ""} {
        puts "FATAL: none of these cables had a device on the chain:"
        foreach cand $hwlist { puts "         $cand" }
        script_exit 1
    }
    puts "Hardware : $hw"
    puts "Device   : $dev"

    # Enumerate BEFORE opening a session: this query opens a transient session
    # of its own and fails if one is already active.
    if {[catch {set insts [get_insystem_source_probe_instance_info \
                      -hardware_name $hw -device_name $dev]} err]} {
        puts "FATAL: could not enumerate ISSP instances."
        puts "       $err"
        puts ""
        puts "       If that mentions an active session, the Quartus GUI's"
        puts "       In-System Sources & Probes Editor has this device open."
        puts "       Close that tab and re-run."
        script_exit 1
    }
    set ISSP -1
    foreach inst $insts {
        lassign $inst idx swidth pwidth name
        puts "Instance : index $idx  \"$name\"  source=$swidth probe=$pwidth"
        if {$swidth == 56 && $pwidth == 128} { set ISSP $idx }
    }
    if {$ISSP < 0} {
        puts "FATAL: no 56-bit source / 128-bit probe instance found."
        puts "       Is the current top_debug programmed onto this device?"
        puts "       (The earlier System_Bus_Final design has a 50/38 instance"
        puts "        named BUS0 - that is a different bus, use its own scripts.)"
        script_exit 1
    }

    if {[catch {start_insystem_source_probe \
                  -hardware_name $hw -device_name $dev} err]} {
        puts "FATAL: could not open an ISSP session."
        puts "       $err"
        script_exit 1
    }

    set SRC 0
    src_flush
    soft_reset

    # issp_mode is a leftover from when the board had scenario sequencers to
    # take the command ports away from.  top_debug has none - the ISSP driver
    # is the only command source - so the bit is unconnected in the RTL and
    # setting it changes nothing.  It is written for the benefit of any older
    # bitstream that still honours it.
    src_field 53 1 1
    src_flush
    puts "Mode     : the JTAG host owns both master command ports"
}

# Leave the source register in a quiet state and release the JTAG session.
proc bus_disconnect {} {
    catch {
        src_field 53 1 0          ;# issp_mode
        src_field 54 1 0          ;# split_en - do not leave the slave splitting
        src_flush
    }
    catch {end_insystem_source_probe}
}
