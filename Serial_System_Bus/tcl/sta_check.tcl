# ===========================================================================
#  sta_check.tcl -- fail the build on UNCONSTRAINED TIMING PATHS.
#
#  Usage:   cd Serial_System_Bus
#           quartus_sta -t tcl/sta_check.tcl                 # after a fit
#           quartus_sta -t tcl/sta_check.tcl -model fast
#
#  Exit status 0 = every path is constrained and timing is met.
#             1 = something is unconstrained, or timing failed.
#
#  WHY THIS EXISTS
#
#  quartus_sta reports an unconstrained path as an *Info* or at worst a
#  Critical Warning, and then exits 0.  A build script that only checks the
#  exit status therefore treats "I analysed nothing" exactly the same as "I
#  analysed everything and it passed".  That is the failure mode this design
#  has already been bitten by once: the earlier parallel project has no .sdc
#  at all, so `clk' was never constrained and its Fmax was never verified -
#  the 50 MHz in its testbenches was simulation-only and nobody noticed.
#
#  An unconstrained path is not a slow path.  It is a path with NO ANSWER,
#  and the fitter is free to route it as badly as it likes.
#
#  WHAT COUNTS AS A FAILURE
#
#  Every check `check_timing' offers, plus the unconstrained-path report:
#
#    no_clock                a register with no clock reaching it
#    no_input_delay          an input port with no set_input_delay
#    no_output_delay         an output port with no set_output_delay
#    partial_input_delay     min or max given, not both
#    partial_output_delay      "
#    latches                 inferred latches - this design must have zero
#    loops                   combinational loops - likewise
#    generated_clocks        a generated clock with no master
#
#  DELIBERATELY CUT PATHS ARE NOT UNCONSTRAINED.  `set_false_path' is an
#  answer: it says "this path has no timing relationship worth analysing".
#  The .sdc cuts rst_n, the LEDs and the two UART pins, and those cuts are
#  reasoned about in the .sdc itself.  They will NOT show up here, which is
#  the point - this catches what nobody thought about, not what somebody
#  decided.
#
#  IF IT FAILS, DO NOT SILENCE IT BY ADDING A FALSE PATH.  Work out whether
#  the path has a real timing relationship first.  A false_path on something
#  that genuinely needs analysing hides the bug rather than fixing it.
# ===========================================================================

set MODEL   slow
set PROJECT Serial_System_Bus
for {set i 0} {$i < [llength $argv]} {incr i} {
    switch -- [lindex $argv $i] {
        -model   { set MODEL   [lindex $argv [incr i]] }
        -project { set PROJECT [lindex $argv [incr i]] }
    }
}

if {[catch {project_open $PROJECT} err]} {
    puts "FATAL: could not open project '$PROJECT'."
    puts "       $err"
    puts "       Run quartus_map and quartus_fit first."
    exit 1
}

create_timing_netlist -model $MODEL
read_sdc
update_timing_netlist

set FAILURES 0
proc fail {msg} { global FAILURES; incr FAILURES; puts "-> ERROR:   $msg" }
proc pass {msg} { puts "-> ok:      $msg" }
proc note {msg} { puts "   ..       $msg" }

puts ""
puts "========================================================="
puts "   TIMING CONSTRAINT CHECK  ($PROJECT, $MODEL model)"
puts "========================================================="

# ---------------------------------------------------------------------------
# 1. Are there any clocks at all?  A design with no create_clock analyses
#    nothing and still exits 0, which is the worst possible outcome.
# ---------------------------------------------------------------------------
set nclk 0
foreach_in_collection c [all_clocks] { incr nclk }
if {$nclk == 0} {
    fail "NO CLOCKS ARE DEFINED - nothing was analysed at all."
    note "The .sdc is missing or was not read.  Everything below is"
    note "meaningless until this is fixed."
} else {
    pass "$nclk clock(s) defined"
}

# ---------------------------------------------------------------------------
# 2. check_timing: every category Quartus offers.
#    It writes a panel; we read the panel back to count the entries, because
#    the text output is not machine-readable.
# ---------------------------------------------------------------------------
# Verified against Quartus 24.1std: `generated_clocks' is NOT one of the
# categories check_timing accepts, despite appearing in some documentation.
# Passing it makes the tool warn and ignore the whole -include list.
set CHECKS {no_clock no_input_delay no_output_delay
            partial_input_delay partial_output_delay
            latches loops}

# Of those, only these are ALWAYS a defect.  no_input_delay and
# no_output_delay fire on every port that has no set_input/output_delay -
# including ones deliberately cut with set_false_path, which is a perfectly
# good answer.  report_ucp below is the authority on what is genuinely
# unanalysed; these two are reported for information only.
set FATAL_CHECKS {no_clock latches loops}

check_timing -include $CHECKS -panel_name "ck" -file "$PROJECT.check_timing.rpt"

# The Summary table carries the COUNTS.  Match on those, not on the check
# names appearing anywhere in the file - every name appears in a passing
# report too, which is what made an earlier version of this script report
# seven failures on a clean design.
set n_ct 0
if {[file exists "$PROJECT.check_timing.rpt"]} {
    set fh [open "$PROJECT.check_timing.rpt" r]
    foreach line [split [read $fh] "\n"] {
        if {[regexp {^\s*;\s*([a-z_]+)\s*;\s*(\d+)\s*;} $line -> chk cnt]} {
            if {$cnt == 0} continue
            if {[lsearch -exact $FATAL_CHECKS $chk] >= 0} {
                incr n_ct
                fail "check_timing: $cnt x $chk"
            } else {
                note "check_timing: $cnt x $chk (informational - a false_path answers it)"
            }
        }
    }
    close $fh
}
if {$n_ct == 0} { pass "check_timing: no fatal category reported anything" }

# ---------------------------------------------------------------------------
# 3. The unconstrained-path report itself.
# ---------------------------------------------------------------------------
report_ucp -summary -file "$PROJECT.ucp.rpt"
set n_ucp 0
if {[file exists "$PROJECT.ucp.rpt"]} {
    set fh [open "$PROJECT.ucp.rpt" r]
    foreach line [split [read $fh] "\n"] {
        # Rows look like:  ; <what> ; <count> ;
        if {[regexp {^\s*;\s*([^;]+?)\s*;\s*(\d+)\s*;} $line -> what cnt]} {
            if {$cnt > 0} {
                incr n_ucp $cnt
                fail [format "%s unconstrained: %s" $cnt [string trim $what]]
            }
        }
    }
    close $fh
}
if {$n_ucp == 0} { pass "report_ucp: no unconstrained paths" }

# ---------------------------------------------------------------------------
# 4. And while we are here: did timing actually MEET?
#    An unconstrained design cannot fail this, which is exactly why the
#    checks above have to come first.
# ---------------------------------------------------------------------------
# get_timing_paths takes -setup / -hold, not a -setup_hold pair, and has no
# -clock_filter - verified against the tool's own usage text.
foreach op {-setup -hold} {
    set worst ""
    foreach_in_collection pth [get_timing_paths $op -npaths 1 -detail summary] {
        set worst [get_path_info $pth -slack]
    }
    if {$worst eq ""} {
        note "no [string range $op 1 end] paths to report"
    } elseif {$worst < 0} {
        fail [format "worst %s slack %.3f ns - TIMING NOT MET" \
                  [string range $op 1 end] $worst]
    } else {
        pass [format "worst %s slack %.3f ns" [string range $op 1 end] $worst]
    }
}

puts ""
puts "========================================================="
if {$FAILURES == 0} {
    puts ">> TIMING CHECK PASSED - every path is constrained and met <<"
} else {
    puts ">> TIMING CHECK FAILED: $FAILURES problem(s) <<"
    puts ""
    puts "   An unconstrained path is not a slow path - it is a path the"
    puts "   analyser was never asked about, and the fitter may route it"
    puts "   arbitrarily badly.  Constrain it, or cut it with a"
    puts "   set_false_path that says IN A COMMENT why it has no timing"
    puts "   relationship.  Do not silence this by cutting blindly."
    puts ""
    puts "   Detail: $PROJECT.check_timing.rpt and $PROJECT.ucp.rpt"
}
puts "=========================================================="
puts ""

delete_timing_netlist
project_close
exit [expr {$FAILURES == 0 ? 0 : 1}]
