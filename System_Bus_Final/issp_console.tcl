#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_console.tcl -- interactive system-bus console over JTAG.
#
#  Pick a slave, an address and a byte; the console writes it, reads it back
#  and tells you whether the bus returned what you put in.
#
#  Usage:  quartus_stp -t issp_console.tcl        <-- quartus_stp ONLY
#
#  Requires top_debug programmed onto the board and the In-System Sources &
#  Probes Editor tab CLOSED (an open editor holds the JTAG session).
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set MASTER 0

# ---------------------------------------------------------------- slave table
# index -> {name base size note}
array set SLAVES {
    0 {"Split RAM"    0x0000 0x1000 "every read splits (slow path)"}
    1 {"Fast RAM"     0x1000 0x1000 "single cycle"}
    2 {"Fast RAM"     0x2000 0x0800 "single cycle"}
    3 {"UART TX"      0x3000 0x1000 "0-2 = cmd bytes, 3 = trigger / busy"}
}

proc show_map {} {
    global SLAVES MASTER
    puts ""
    puts "  Slave  Name        Range            Notes"
    puts "  -----  ----------  ---------------  ------------------------------"
    foreach i {0 1 2 3} {
        lassign $SLAVES($i) nm base size note
        puts [format "    %d    %-10s  0x%04X-0x%04X    %s" \
                  $i $nm $base [expr {$base + $size - 1}] $note]
    }
    puts "         (gap)       0x2800-0x2FFF    unmapped - access DEADLOCKS the bus"
    puts ""
    puts "  Current master: M$MASTER"
    puts ""
}

# Which physical storage a slave offset actually lands on. The RAMs hold only
# 64 words and index on the UPPER address bits, so offsets alias in blocks.
proc where {sl off} {
    switch -- $sl {
        0 - 1   { return [list [expr {($off & 0xFFF) >> 6}] 64 0x40] }
        2       { return [list [expr {($off & 0x7FF) >> 5}] 64 0x20] }
        default { return [list [expr {$off & 3}] 4 0x1] }
    }
}

proc parse_hex {s} {
    set s [string trim $s]
    if {$s eq ""} { return -1 }
    regsub -nocase {^0x} $s "" s
    if {![regexp {^[0-9a-fA-F]+$} $s]} { return -1 }
    return [expr 0x$s]
}

proc ask {prompt} {
    puts -nonewline $prompt
    flush stdout
    if {[gets stdin line] < 0} { return "" }
    return [string trim $line]
}

# Absolute address from a slave + offset, with the two traps guarded.
proc resolve {sl off} {
    global SLAVES
    if {![info exists SLAVES($sl)]} {
        puts "  ! slave must be 0-3"
        return -1
    }
    lassign $SLAVES($sl) nm base size note
    if {$off >= $size} {
        puts [format "  ! offset 0x%X is outside slave %d (max 0x%X)" $off $sl [expr {$size-1}]]
        return -1
    }
    set a [expr {$base + $off}]
    if {$a >= 0x2800 && $a <= 0x2FFF} {
        puts "  ! 0x[format %04X $a] is in the unmapped gap - it would deadlock the bus"
        puts "    (no slave select asserts, so bus_ready never rises; KEY\[0\] to recover)"
        return -1
    }
    return $a
}

proc do_write {sl off data {verify 1}} {
    global MASTER SLAVES
    set a [resolve $sl $off]
    if {$a < 0} { return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl $off] idx nwords blk

    puts [format "  M%d write 0x%02X -> 0x%04X   (slave %d %s, word %d of %d)" \
              $MASTER $data $a $sl $nm $idx $nwords]

    lassign [bus_cmd $MASTER 1 $a $data] ok rd lat
    if {!$ok} {
        puts "  -> HUNG: write never completed (latency saturated at $lat)"
        puts "     press KEY\[0\] to recover the bus"
        return
    }
    puts [format "  -> write done (%d clks)" $lat]

    if {!$verify} { return }
    if {$sl == 3 && ($off & 3) == 3} {
        puts "  -> offset 3 is the UART trigger; nothing to read back"
        return
    }

    lassign [bus_cmd $MASTER 0 $a 0x00] ok2 rd2 lat2
    if {!$ok2} {
        puts "  -> HUNG: read-back never completed (latency saturated at $lat2)"
        return
    }
    if {$rd2 == $data} {
        puts [format "  -> read back 0x%02X  (%d clks)   PASS" $rd2 $lat2]
    } else {
        puts [format "  -> read back 0x%02X  (%d clks)   MISMATCH, expected 0x%02X" \
                  $rd2 $lat2 $data]
        if {$blk > 1} {
            puts [format "     note: this slave holds %d words; offsets alias in blocks of 0x%X" \
                      $nwords $blk]
        }
    }
}

proc do_read {sl off} {
    global MASTER SLAVES
    set a [resolve $sl $off]
    if {$a < 0} { return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl $off] idx nwords blk

    lassign [bus_cmd $MASTER 0 $a 0x00] ok rd lat
    if {!$ok} {
        puts "  -> HUNG: read never completed (latency saturated at $lat)"
        puts "     press KEY\[0\] to recover the bus"
        return
    }
    puts [format "  M%d read 0x%04X = 0x%02X   (%d clks, slave %d %s, word %d)" \
              $MASTER $a $rd $lat $sl $nm $idx]
}

# Write a distinct value to every distinct word of a slave, then read them all
# back. This is the "is the bus actually working" sweep.
proc do_sweep {sl} {
    global MASTER SLAVES
    if {![info exists SLAVES($sl)]} { puts "  ! slave must be 0-3"; return }
    if {$sl == 3} { puts "  ! sweep is for the RAM slaves (0-2)"; return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl 0] _ nwords blk

    # Keep it quick: 8 words spread across the slave.
    set n 8
    set step [expr {($nwords / $n) * $blk}]
    puts "  Sweeping slave $sl ($nm): $n words, stride 0x[format %X $step]"

    set bad 0
    for {set i 0} {$i < $n} {incr i} {
        set off [expr {$i * $step}]
        set a   [expr {$base + $off}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 1 $a $val] ok rd lat
        if {!$ok} { puts "  -> HUNG writing 0x[format %04X $a]"; return }
    }
    for {set i 0} {$i < $n} {incr i} {
        set off [expr {$i * $step}]
        set a   [expr {$base + $off}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 0 $a 0x00] ok rd lat
        if {!$ok} { puts "  -> HUNG reading 0x[format %04X $a]"; return }
        if {$rd == $val} {
            puts [format "    0x%04X  wrote 0x%02X  read 0x%02X  ok   (%d clks)" $a $val $rd $lat]
        } else {
            puts [format "    0x%04X  wrote 0x%02X  read 0x%02X  BAD  (%d clks)" $a $val $rd $lat]
            incr bad
        }
    }
    if {$bad == 0} {
        puts "  -> sweep PASSED: $n/$n words verified"
    } else {
        puts "  -> sweep FAILED: $bad of $n words wrong"
    }
}

proc show_help {} {
    puts ""
    puts "  w                    guided write+verify (asks slave, offset, data)"
    puts "  w <slave> <off> <d>  write and verify, e.g.  w 1 40 5A"
    puts "  r                    guided read"
    puts "  r <slave> <off>      read, e.g.  r 1 40"
    puts "  sweep <slave>        write+read 8 words across a slave (0-2)"
    puts "  m <0|1>              choose which master issues commands"
    puts "  map                  show the address map"
    puts "  h                    this help"
    puts "  q                    quit"
    puts ""
    puts "  Offsets are hex and relative to the slave base. All values hex."
    puts ""
}

# ---------------------------------------------------------------- main
puts "========================================================="
puts "   INTERACTIVE SYSTEM BUS CONSOLE  (ISSP over JTAG)"
puts "========================================================="
bus_connect
show_map
show_help

while {1} {
    puts -nonewline "bus> "
    flush stdout
    if {[gets stdin line] < 0} break
    set line [string trim $line]
    if {$line eq ""} { continue }
    set argv [split $line]
    set cmd  [string tolower [lindex $argv 0]]

    switch -- $cmd {
        q - quit - exit { break }
        h - help - "?"  { show_help }
        map             { show_map }

        m {
            set v [lindex $argv 1]
            if {$v eq "0" || $v eq "1"} {
                set MASTER $v
                puts "  master = M$MASTER"
            } else {
                puts "  ! usage: m 0   or   m 1"
            }
        }

        sweep {
            set sl [lindex $argv 1]
            if {![string is integer -strict $sl]} { set sl [ask "  Slave \[0-2\]: "] }
            if {[string is integer -strict $sl]} { do_sweep $sl } else { puts "  ! slave must be 0-2" }
        }

        w {
            if {[llength $argv] >= 4} {
                set sl   [lindex $argv 1]
                set off  [parse_hex [lindex $argv 2]]
                set data [parse_hex [lindex $argv 3]]
            } else {
                set sl   [ask "  Slave \[0-3\]: "]
                set off  [parse_hex [ask "  Offset (hex): "]]
                set data [parse_hex [ask "  Data (hex):   "]]
            }
            if {![string is integer -strict $sl] || $off < 0 || $data < 0} {
                puts "  ! need a slave 0-3 plus hex offset and hex data"
            } elseif {$data > 0xFF} {
                puts "  ! data is one byte (0x00-0xFF)"
            } else {
                do_write $sl $off $data
            }
        }

        r {
            if {[llength $argv] >= 3} {
                set sl  [lindex $argv 1]
                set off [parse_hex [lindex $argv 2]]
            } else {
                set sl  [ask "  Slave \[0-3\]: "]
                set off [parse_hex [ask "  Offset (hex): "]]
            }
            if {![string is integer -strict $sl] || $off < 0} {
                puts "  ! need a slave 0-3 and a hex offset"
            } else {
                do_read $sl $off
            }
        }

        default { puts "  ! unknown command \"$cmd\" - type h for help" }
    }
}

puts "\nclosing session."
bus_disconnect
script_exit 0
