# ===========================================================================
#  run_issp_test.tcl  --  launch the ISSP bus test FROM the Quartus GUI
#
#  The Quartus GUI's Tcl interpreter does not provide ::quartus::jtag or
#  ::quartus::insystem_source_probe, so issp_bus_test.tcl cannot run directly
#  in the GUI console. This wrapper shells out to quartus_stp (which does have
#  them) and echoes the result back into the console.
#
#  Run it from the GUI in either of these ways:
#     Tools > Tcl Scripts...      -> select run_issp_test.tcl -> Run
#     View > Utility Windows > Tcl Console:
#         source {.../System_Bus_Final/run_issp_test.tcl}
#
#  Set RUN_GAP to 1 below to also run the decode-gap deadlock demo.
#  NOTE: close the In-System Sources & Probes Editor tab first - an open
#        editor holds the JTAG session and the test cannot claim it.
# ===========================================================================

set RUN_GAP 0

set here   [file dirname [file normalize [info script]]]
set target [file join $here issp_bus_test.tcl]

if {![file exists $target]} {
    puts "ERROR: cannot find issp_bus_test.tcl next to this wrapper."
    puts "       looked for: $target"
    return
}

# Locate quartus_stp: prefer the running installation, fall back to PATH.
set stp ""
set cands {}
if {[info exists quartus(binpath)]} {
    lappend cands [file join $quartus(binpath) quartus_stp]
}
if {[info exists quartus(quartus_rootpath)]} {
    lappend cands [file join $quartus(quartus_rootpath) bin quartus_stp]
}
foreach c $cands {
    if {[file executable $c]} { set stp $c; break }
}
if {$stp eq ""} {
    if {[catch {set stp [exec which quartus_stp]}]} {
        puts "ERROR: could not locate the quartus_stp executable."
        puts "       Run the test from a terminal instead:"
        puts "           quartus_stp -t issp_bus_test.tcl"
        return
    }
}

set cmd [list $stp -t $target]
if {$RUN_GAP} { lappend cmd -gap }

puts "Launching: $cmd"
puts "(this blocks the GUI for a few seconds while the test runs)\n"

# exec raises on a non-zero exit status, which the test uses to report
# failures - so capture output either way rather than letting it throw.
set failed [catch {eval exec $cmd 2>@1} out]

foreach line [split $out "\n"] {
    # drop the quartus_stp licence/banner preamble
    if {[string match "    Info:*" $line]} { continue }
    if {[string match "Info: Command:*" $line]} { continue }
    # Tcl appends this to the captured output when exec sees a non-zero exit
    if {$line eq "child process exited abnormally"} { continue }
    puts $line
}

if {$failed} {
    puts "\n>> quartus_stp reported a non-zero exit status (test failures above)."
} else {
    puts "\n>> quartus_stp completed successfully."
}
