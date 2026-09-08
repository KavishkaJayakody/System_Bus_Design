#!/usr/bin/env quartus_stp -t
# ===========================================================================
#  issp_console.tcl -- interactive console for the SERIAL system bus.
#
#  Drives either master's command port over JTAG and reports what the bus
#  did, including the two things only this design has: the address it
#  reassembled off the single serial wire, and how long the address frame
#  actually was.
#
#  Usage:  cd Serial_System_Bus
#          quartus_stp -t tcl/issp_console.tcl      <-- quartus_stp ONLY
#
#  Requires de2_top programmed onto the board and the In-System Sources &
#  Probes Editor tab CLOSED (an open editor holds the JTAG session).
#
#  The board switches do not need touching: connecting sets issp_mode, and
#  the command-port mux in de2_top is unconditional, so the on-board
#  scenario sequencers are disconnected for as long as this console is up.
# ===========================================================================

source [file join [file dirname [file normalize [info script]]] issp_bus_lib.tcl]

set MASTER 0
set SPLIT  0

# ---------------------------------------------------------------- slave table
# index -> {name base size note}
array set SLAVES {
    0 {"Split RAM" 0x0000 0x1000 "split capable - see 'split on'"}
    1 {"RAM"       0x1000 0x1000 "plain"}
    2 {"RAM"       0x2000 0x0800 "plain"}
}

proc show_map {} {
    global SLAVES MASTER SPLIT
    puts ""
    puts "  Slave  Name        Range            Notes"
    puts "  -----  ----------  ---------------  ------------------------------"
    foreach i {0 1 2} {
        lassign $SLAVES($i) nm base size note
        puts [format "    %d    %-10s  0x%04X-0x%04X    %s" \
                  $i $nm $base [expr {$base + $size - 1}] $note]
    }
    puts "         (hole)      0x2800-0x2FFF    unmapped -> ERROR"
    puts "         (unmapped)  0x3000-0x7FFF    unmapped -> ERROR"
    puts "         (reserved)  0x8000-0xFFFF    phase-2 remote window -> ERROR"
    puts ""
    puts "  Unmapped addresses ANSWER; they do not hang the bus. Try one with"
    puts "  'ra 2800' - the default responder inside system_bus is what makes"
    puts "  that safe, and proving it is the point of the exercise."
    puts ""
    puts "  Current master: M$MASTER   slave-0 split: [expr {$SPLIT ? {ON} : {off}}]"
    puts ""
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

# Absolute address from a slave + offset.
proc resolve {sl off} {
    global SLAVES
    if {![info exists SLAVES($sl)]} { puts "  ! slave must be 0-2"; return -1 }
    lassign $SLAVES($sl) nm base size note
    if {$off >= $size} {
        puts [format "  ! offset 0x%X is outside slave %d (max 0x%X)" \
                  $off $sl [expr {$size-1}]]
        return -1
    }
    return [expr {$base + $off}]
}

# Report one transaction result.
proc report {tag a r} {
    lassign $r ok rd rp lat err splits
    if {!$ok} {
        puts [format "  -> NO ANSWER for 0x%04X (latency saturated at %d)" $a $lat]
        puts "     On this bus that should be impossible - every unmapped address"
        puts "     is answered ERROR by the default responder. If you see this,"
        puts "     the bus is genuinely wedged. Press KEY\[0\]."
        return
    }
    puts [format "  -> %s 0x%04X = 0x%02X   resp %s   %d clks   splits %d" \
              $tag $a $rd [resp_name $rp] $lat $splits]
    if {$rp == 1} {
        puts "     ERROR means nothing is mapped there - the bus answered and"
        puts "     released the grant, which is the designed behaviour."
    }
}

proc do_write {a data} {
    global MASTER
    puts [format "  M%d write 0x%02X -> 0x%04X" $MASTER $data $a]
    report "write" $a [bus_cmd $MASTER 1 $a $data]
}

proc do_read {a} {
    global MASTER
    report "M[set MASTER] read" $a [bus_cmd $MASTER 0 $a 0x00]
}

# Fire a write from BOTH masters on the same clock edge.
proc do_both {a0 d0 a1 d1} {
    soft_reset          ;# clear collision so it reflects THIS transfer only
    puts [format "  M0 write 0x%02X -> 0x%04X" $d0 $a0]
    puts [format "  M1 write 0x%02X -> 0x%04X" $d1 $a1]
    puts "  firing both masters on the same clock edge..."

    lassign [bus_cmd_pair 1 $a0 $d0 1 $a1 $d1] r0 r1
    lassign $r0 ok0 rd0 rp0 lat0
    lassign $r1 ok1 rd1 rp1 lat1

    if {$ok0} { puts [format "  -> M0 done, %s, %d clks" [resp_name $rp0] $lat0] } \
         else { puts "  -> M0 NO ANSWER (lat $lat0)" }
    if {$ok1} { puts [format "  -> M1 done, %s, %d clks" [resp_name $rp1] $lat1] } \
         else { puts "  -> M1 NO ANSWER (lat $lat1)" }

    if {[bits [probe] 69 69]} {
        puts "  -> collision flag SET: both masters were in flight together, so"
        puts "     the arbiter really did serialise them onto the one bus."
        puts [format "     M0 got %d clks, M1 got %d - the loser absorbs the wait." \
                  $lat0 $lat1]
    } else {
        puts "  -> collision flag clear: they did not overlap this time."
    }
}

# Write a distinct value to 8 words across a slave, then read them all back.
proc do_sweep {sl} {
    global MASTER SLAVES
    if {![info exists SLAVES($sl)]} { puts "  ! slave must be 0-2"; return }
    lassign $SLAVES($sl) nm base size note

    set n 8
    set step [expr {$size / $n}]
    puts "  Sweeping slave $sl ($nm): $n words, stride 0x[format %X $step]"

    for {set i 0} {$i < $n} {incr i} {
        set a   [expr {$base + $i * $step}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 1 $a $val] ok
        if {!$ok} { puts "  -> NO ANSWER writing 0x[format %04X $a]"; return }
    }
    set bad 0
    for {set i 0} {$i < $n} {incr i} {
        set a   [expr {$base + $i * $step}]
        set val [expr {(0xA0 + $i) & 0xFF}]
        lassign [bus_cmd $MASTER 0 $a 0x00] ok rd rp lat
        if {!$ok} { puts "  -> NO ANSWER reading 0x[format %04X $a]"; return }
        if {$rd == $val} {
            puts [format "    0x%04X  wrote 0x%02X  read 0x%02X  ok   (%d clks)" \
                      $a $val $rd $lat]
        } else {
            puts [format "    0x%04X  wrote 0x%02X  read 0x%02X  BAD  (%d clks)" \
                      $a $val $rd $lat]
            incr bad
        }
    }
    if {$bad == 0} { puts "  -> sweep PASSED: $n/$n words verified" } \
                else { puts "  -> sweep FAILED: $bad of $n words wrong" }
}

proc show_help {} {
    puts ""
    puts "  w <slave> <off> <d>  write via slave+offset,  e.g.  w 1 ABC 5A"
    puts "  r <slave> <off>      read  via slave+offset,  e.g.  r 1 ABC"
    puts "  wa <addr> <d>        write an ABSOLUTE address, e.g. wa 1ABC 5A"
    puts "  ra <addr>            read  an ABSOLUTE address, e.g. ra 2800"
    puts "                       (absolute lets you hit the unmapped ranges)"
    puts "  both <a0> <d0> <a1> <d1>"
    puts "                       both masters write on the same clock edge"
    puts "  sweep <slave>        write+read 8 words across a slave (0-2)"
    puts "  m <0|1>              choose which master issues commands"
    puts "  split <on|off>       make slave 0 answer SPLIT"
    puts "  status               dump the bus-side probes"
    puts "  map                  show the address map"
    puts "  h                    this help"
    puts "  q                    quit (hands the bus back to the switches)"
    puts ""
    puts "  All values hex. Offsets are relative to the slave base."
    puts ""
}

# ---------------------------------------------------------------- main
puts "========================================================="
puts "   SERIAL SYSTEM BUS CONSOLE  (ISSP over JTAG)"
puts "========================================================="
bus_connect
show_map
show_help

while {1} {
    puts -nonewline "bus\[M$MASTER[expr {$SPLIT ? {,split} : {}}]\]> "
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
        status          { bus_status }

        m {
            set v [lindex $argv 1]
            if {$v eq "0" || $v eq "1"} {
                set MASTER $v
                puts "  master = M$MASTER"
            } else { puts "  ! usage: m 0   or   m 1" }
        }

        split {
            set v [string tolower [lindex $argv 1]]
            if {$v eq "on" || $v eq "1"} {
                set SPLIT 1 ; set_split_en 1
                puts "  slave 0 will now answer SPLIT on a fresh access."
                puts "  A read of slave 0 should now cost visibly more clocks,"
                puts "  and 'status' will show the split mask and the count."
            } elseif {$v eq "off" || $v eq "0"} {
                set SPLIT 0 ; set_split_en 0
                puts "  slave 0 split disabled."
            } else { puts "  ! usage: split on   or   split off" }
        }

        both {
            if {[llength $argv] >= 5} {
                set a0 [parse_hex [lindex $argv 1]]
                set d0 [parse_hex [lindex $argv 2]]
                set a1 [parse_hex [lindex $argv 3]]
                set d1 [parse_hex [lindex $argv 4]]
            } else {
                set a0 [parse_hex [ask "  M0 address (hex): "]]
                set d0 [parse_hex [ask "  M0 data    (hex): "]]
                set a1 [parse_hex [ask "  M1 address (hex): "]]
                set d1 [parse_hex [ask "  M1 data    (hex): "]]
            }
            if {$a0 < 0 || $a1 < 0 || $d0 < 0 || $d1 < 0} {
                puts "  ! need an address and a byte for each master"
            } elseif {$d0 > 0xFF || $d1 > 0xFF} {
                puts "  ! data is one byte (0x00-0xFF)"
            } elseif {$a0 > 0xFFFF || $a1 > 0xFFFF} {
                puts "  ! address is 16 bits (0x0000-0xFFFF)"
            } else { do_both $a0 $d0 $a1 $d1 }
        }

        sweep {
            set sl [lindex $argv 1]
            if {![string is integer -strict $sl]} { set sl [ask "  Slave \[0-2\]: "] }
            if {[string is integer -strict $sl]} { do_sweep $sl } \
                                            else { puts "  ! slave must be 0-2" }
        }

        wa {
            set a [parse_hex [lindex $argv 1]]
            set d [parse_hex [lindex $argv 2]]
            if {$a < 0 || $d < 0}          { puts "  ! usage: wa <addr> <data>" } \
            elseif {$a > 0xFFFF}           { puts "  ! address is 16 bits" } \
            elseif {$d > 0xFF}             { puts "  ! data is one byte" } \
            else                           { do_write $a $d }
        }

        ra {
            set a [parse_hex [lindex $argv 1]]
            if {$a < 0}          { puts "  ! usage: ra <addr>" } \
            elseif {$a > 0xFFFF} { puts "  ! address is 16 bits" } \
            else                 { do_read $a }
        }

        w {
            if {[llength $argv] >= 4} {
                set sl [lindex $argv 1]
                set of [parse_hex [lindex $argv 2]]
                set dd [parse_hex [lindex $argv 3]]
            } else {
                set sl [ask "  Slave \[0-2\]: "]
                set of [parse_hex [ask "  Offset (hex): "]]
                set dd [parse_hex [ask "  Data (hex):   "]]
            }
            if {![string is integer -strict $sl] || $of < 0 || $dd < 0} {
                puts "  ! need a slave 0-2 plus hex offset and hex data"
            } elseif {$dd > 0xFF} {
                puts "  ! data is one byte (0x00-0xFF)"
            } else {
                set a [resolve $sl $of]
                if {$a >= 0} { do_write $a $dd }
            }
        }

        r {
            if {[llength $argv] >= 3} {
                set sl [lindex $argv 1]
                set of [parse_hex [lindex $argv 2]]
            } else {
                set sl [ask "  Slave \[0-2\]: "]
                set of [parse_hex [ask "  Offset (hex): "]]
            }
            if {![string is integer -strict $sl] || $of < 0} {
                puts "  ! need a slave 0-2 and a hex offset"
            } else {
                set a [resolve $sl $of]
                if {$a >= 0} { do_read $a }
            }
        }

        default { puts "  ! unknown command \"$cmd\" - type h for help" }
    }
}

puts "\nhanding the bus back to the on-board sequencers."
bus_disconnect
script_exit 0
