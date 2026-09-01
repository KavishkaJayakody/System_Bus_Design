# ===========================================================================
#  issp_bus_lib.tcl -- shared plumbing for driving the system bus over JTAG
#                      through the In-System Sources & Probes instance "BUS0".
#
#  Sourced by issp_bus_test.tcl (scripted regression) and issp_console.tcl
#  (interactive). Keeping the bit map in one place stops the two drifting.
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
set ISSP       -1
set POLL_LIMIT 200    ;# probe reads before declaring a transaction hung

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

# Per-master probe field offsets: {rdata_lo done busy lat_lo}
proc pfields {m} {
    if {$m == 0} { return {0 8 9 20} } else { return {10 18 19 28} }
}

# ---------------------------------------------------------------- bus commands
proc arm {m we addr wdata} {
    set b [expr {$m * 24}]
    src_field $b             1 0          ;# go low - clears sticky done
    src_field [expr {$b+1}]  1 $we
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

# One transaction on one master. Returns {ok rdata latency}.
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

# Fire BOTH masters on the same clock edge. A single source write updates all
# 50 bits at once, so both go edges land together; two separate writes would
# leave milliseconds between them and never overlap.
proc bus_cmd_pair {we0 a0 d0 we1 a1 d1} {
    arm 0 $we0 $a0 $d0
    arm 1 $we1 $a1 $d1
    src_flush
    src_field 0  1 1
    src_field 24 1 1
    src_flush
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

# ---------------------------------------------------------------- connection
# Opens the ISSP session and sets ::ISSP. Bails out on any failure.
proc bus_connect {} {
    global ISSP SRC

    # These packages exist ONLY in quartus_stp. Verified on Quartus 24.1std:
    # quartus_sh rejects them and the GUI Tcl console lacks them entirely.
    if {[catch {load_package insystem_source_probe}] || [catch {load_package jtag}]} {
        puts "FATAL: the JTAG / In-System Sources & Probes Tcl packages are not"
        puts "       available in this interpreter."
        puts ""
        puts "       Run from a terminal with quartus_stp, e.g."
        puts "           quartus_stp -t issp_console.tcl"
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
        if {$swidth == 50 && $pwidth == 38} { set ISSP $idx }
    }
    if {$ISSP < 0} {
        puts "FATAL: no 50-bit source / 38-bit probe instance found."
        puts "       Is the current top_debug programmed onto this device?"
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
}

proc bus_disconnect {} {
    catch {end_insystem_source_probe}
}
