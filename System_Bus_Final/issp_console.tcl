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
set REMOTE 0      ;# 1 = send M0 commands to the OTHER board over UART

# ---------------------------------------------------------------- slave table
# index -> {name base size note}
array set SLAVES {
    0 {"Split RAM"    0x0000 0x1000 "every read splits (slow path)"}
    1 {"Fast RAM"     0x1000 0x1000 "single cycle"}
    2 {"Fast RAM"     0x2000 0x0800 "single cycle"}
    3 {"(unmapped)"   0x3000 0x1000 "UART moved into M0; reads 0x00"}
}

proc show_map {} {
    global SLAVES MASTER REMOTE
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
    puts "  Current master: M$MASTER[expr {$REMOTE ? "   REMOTE mode: commands go to the OTHER board" : ""}]"
    puts ""
}

# Which physical word a slave offset lands on. The RAMs are full-depth
# altsyncram blocks now (4K / 4K / 2K), so every offset is its own word and
# nothing aliases. The third element is the alias block size, 1 = none.
proc where {sl off} {
    switch -- $sl {
        0 - 1   { return [list [expr {$off & 0xFFF}] 4096 1] }
        2       { return [list [expr {$off & 0x7FF}] 2048 1] }
        default { return [list 0 0 1] }
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

proc do_write {sl off data} {
    global MASTER SLAVES REMOTE
    set a [resolve $sl $off]
    if {$a < 0} { return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl $off] idx nwords blk

    puts [format "  M%d %swrite 0x%02X -> 0x%04X   (slave %d %s, word %d of %d)" \
              $MASTER [expr {$REMOTE ? "REMOTE " : ""}] $data $a $sl $nm $idx $nwords]

    lassign [bus_cmd $MASTER 1 $a $data $REMOTE] ok rd lat err
    if {$err} {
        puts "  -> REMOTE TIMEOUT: the other board did not answer."
        puts "     check the UART wiring (our C3 -> their D3, their C3 -> our D3, common ground)"
        return
    }
    if {!$ok} {
        puts "  -> HUNG: write never completed (latency saturated at $lat)"
        puts "     press KEY\[0\] to recover the bus"
        return
    }
    puts [format "  -> write done (%d clks).  Use 'r %d %X' to read it back." \
              $lat $sl $off]
    if {$sl == 3 && ($off & 3) == 3} {
        puts "  -> that was the UART trigger: 24-bit command sent to the far board"
    }
}

proc do_read {sl off} {
    global MASTER SLAVES REMOTE
    set a [resolve $sl $off]
    if {$a < 0} { return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl $off] idx nwords blk

    lassign [bus_cmd $MASTER 0 $a 0x00 $REMOTE] ok rd lat err
    if {!$ok} {
        puts "  -> HUNG: read never completed (latency saturated at $lat)"
        puts "     press KEY\[0\] to recover the bus"
        return
    }
    if {$err} {
        puts "  -> REMOTE TIMEOUT: the other board did not answer."
        puts "     check the UART wiring (our C3 -> their D3, their C3 -> our D3, common ground)"
        return
    }
    puts [format "  M%d %sread 0x%04X = 0x%02X   (%d clks, slave %d %s, word %d)" \
              $MASTER [expr {$REMOTE ? "REMOTE " : ""}] $a $rd $lat $sl $nm $idx]
}

# Fire a write from BOTH masters on the same clock edge. bus_cmd_pair sets both
# go bits in one source write, so the two requests reach the arbiter together -
# issuing them as two separate JTAG writes would leave milliseconds between them
# and never overlap. The sticky collision flag proves they really did contend.
proc do_both {sl0 off0 d0 sl1 off1 d1} {
    global SLAVES REMOTE
    if {$REMOTE} {
        puts "  ! 'both' is a local-bus demonstration and is disabled in REMOTE mode."
        puts "    A remote M0 command travels over UART and never touches our own"
        puts "    bus, so the two masters could not contend and the collision flag"
        puts "    would stay clear. Turn it off with 'rem off' first."
        return
    }
    set a0 [resolve $sl0 $off0]
    if {$a0 < 0} { return }
    set a1 [resolve $sl1 $off1]
    if {$a1 < 0} { return }

    lassign $SLAVES($sl0) nm0 b0 sz0 n0
    lassign $SLAVES($sl1) nm1 b1 sz1 n1

    soft_reset          ;# clear collision so it reflects THIS transfer only

    puts [format "  M0 write 0x%02X -> 0x%04X   (slave %d %s)" $d0 $a0 $sl0 $nm0]
    puts [format "  M1 write 0x%02X -> 0x%04X   (slave %d %s)" $d1 $a1 $sl1 $nm1]
    puts "  firing both masters on the same clock edge..."

    lassign [bus_cmd_pair 1 $a0 $d0 1 $a1 $d1] r0 r1
    lassign $r0 ok0 rd0 lat0
    lassign $r1 ok1 rd1 lat1

    if {$ok0} { puts [format "  -> M0 write done (%d clks)" $lat0] } \
         else { puts "  -> M0 HUNG (latency saturated at $lat0)" }
    if {$ok1} { puts [format "  -> M1 write done (%d clks)" $lat1] } \
         else { puts "  -> M1 HUNG (latency saturated at $lat1)" }

    if {[bits [probe] 36 36]} {
        puts "  -> collision flag SET: both masters were in flight together,"
        puts "     so the arbiter really did serialise them onto the bus."
    } else {
        puts "  -> collision flag clear: they did not overlap this time."
    }
    puts [format "     read them back with:  r %d %X    and    m 1 ; r %d %X" \
              $sl0 $off0 $sl1 $off1]
}

# Write a distinct value to every distinct word of a slave, then read them all
# back. This is the "is the bus actually working" sweep.
proc do_sweep {sl} {
    global MASTER SLAVES REMOTE
    if {![info exists SLAVES($sl)]} { puts "  ! slave must be 0-3"; return }
    if {$sl == 3} { puts "  ! sweep is for the RAM slaves (0-2)"; return }
    lassign $SLAVES($sl) nm base size note
    lassign [where $sl 0] _ nwords blk

    # Keep it quick: 8 words spread across the slave.
    set n 8
    set step [expr {($nwords / $n) * $blk}]
    puts "  Sweeping [expr {$REMOTE ? {the OTHER board's} : {our}}] slave $sl ($nm): $n words, stride 0x[format %X $step]"

    set bad 0
    for {set i 0} {$i < $n} {incr i} {
        set off [expr {$i * $step}]
        set a   [expr {$base + $off}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 1 $a $val $REMOTE] ok rd lat err
        if {!$ok} { puts "  -> HUNG writing 0x[format %04X $a]"; return }
        if {$err} { puts "  -> REMOTE TIMEOUT writing 0x[format %04X $a] - link down?"; return }
    }
    for {set i 0} {$i < $n} {incr i} {
        set off [expr {$i * $step}]
        set a   [expr {$base + $off}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 0 $a 0x00 $REMOTE] ok rd lat err
        if {!$ok} { puts "  -> HUNG reading 0x[format %04X $a]"; return }
        if {$err} { puts "  -> REMOTE TIMEOUT reading 0x[format %04X $a] - link down?"; return }
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
    puts "  w                    guided write (asks slave, offset, data)"
    puts "  w <slave> <off> <d>  write, e.g.  w 1 40 5A"
    puts "  r                    guided read"
    puts "  r <slave> <off>      read, e.g.  r 1 40"
    puts "  both                 guided: BOTH masters write on the same clock"
    puts "  both <s0> <o0> <d0> <s1> <o1> <d1>"
    puts "                       e.g.  both 1 40 AA 2 20 55"
    puts "  sweep <slave>        write+read 8 words across a slave (0-2)"
    puts "  m <0|1>              choose which master issues commands"
    puts "  rem <on|off>         send M0 commands to the OTHER board over UART"
    puts "                       (applies to w, r and sweep; 'both' is local only)"
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
    puts -nonewline [expr {$REMOTE ? "bus(remote)> " : "bus> "}]
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

        rem {
            set v [string tolower [lindex $argv 1]]
            if {$v eq "on" || $v eq "1"} {
                if {$MASTER != 0} {
                    puts "  ! only Master 0 has the UART link - switch with 'm 0' first"
                } else {
                    set REMOTE 1
                    puts "  REMOTE mode ON: w/r now target the OTHER board's bus"
                }
            } elseif {$v eq "off" || $v eq "0"} {
                set REMOTE 0
                puts "  REMOTE mode OFF: w/r target our own bus"
            } else {
                puts "  ! usage: rem on   or   rem off"
            }
        }

        m {
            set v [lindex $argv 1]
            if {$v eq "0" || $v eq "1"} {
                set MASTER $v
                if {$MASTER != 0 && $REMOTE} {
                    set REMOTE 0
                    puts "  REMOTE mode OFF (only Master 0 has the UART link)"
                }
                puts "  master = M$MASTER"
            } else {
                puts "  ! usage: m 0   or   m 1"
            }
        }

        both {
            if {[llength $argv] >= 7} {
                set sl0 [lindex $argv 1]
                set of0 [parse_hex [lindex $argv 2]]
                set dd0 [parse_hex [lindex $argv 3]]
                set sl1 [lindex $argv 4]
                set of1 [parse_hex [lindex $argv 5]]
                set dd1 [parse_hex [lindex $argv 6]]
            } else {
                puts "  -- Master 0 --"
                set sl0 [ask "  Slave \[0-3\]: "]
                set of0 [parse_hex [ask "  Offset (hex): "]]
                set dd0 [parse_hex [ask "  Data (hex):   "]]
                puts "  -- Master 1 --"
                set sl1 [ask "  Slave \[0-3\]: "]
                set of1 [parse_hex [ask "  Offset (hex): "]]
                set dd1 [parse_hex [ask "  Data (hex):   "]]
            }
            if {![string is integer -strict $sl0] || ![string is integer -strict $sl1]
                || $of0 < 0 || $of1 < 0 || $dd0 < 0 || $dd1 < 0} {
                puts "  ! need slave/offset/data for each master"
            } elseif {$dd0 > 0xFF || $dd1 > 0xFF} {
                puts "  ! data is one byte (0x00-0xFF)"
            } else {
                do_both $sl0 $of0 $dd0 $sl1 $of1 $dd1
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
