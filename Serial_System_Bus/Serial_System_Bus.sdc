#===========================================================================
# Serial_System_Bus.sdc -- timing constraints for the DE2-115 build
#
# One clock, one domain.  CLOCK_50 is the DE2-115's 50 MHz oscillator on the
# dedicated clock pin Y2; everything in the design is clocked by it.
#
# Without this file Quartus reports "no clocks defined", analyses nothing,
# and Fmax is never checked - so the 50 MHz in the testbenches would be an
# assumption rather than a verified result.
#
# The JTAG hub adds a second domain, `altera_reserved_tck', when the ISSP
# instance is present.  Quartus constrains that itself; nothing here needs to
# mention it.
#===========================================================================

create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]

# Model the small amount of jitter/skew the fitter should budget for.
derive_clock_uncertainty

#---------------------------------------------------------------------------
# `rst_n' is KEY[0], pressed by a person, and the LEDs are looked at by one.
# Neither has a setup or hold relationship worth constraining, so cut those
# paths and let the analyser concentrate on the internal bus logic.
#
# NOTE: there is no reset synchroniser - `reset_ctrl' went with the board
# layer - so rst_n reaches every flop's asynchronous clear directly and its
# RELEASE is unsynchronised.  Cutting it here means recovery/removal is not
# analysed either.  That is acceptable for a button on a lab board (the worst
# case is one metastable release, cured by pressing KEY[0] again) but it is a
# deliberate shortcut, not a verified path.
#---------------------------------------------------------------------------
set_false_path -from [get_ports {rst_n}] -to [all_registers]
set_false_path -from * -to [get_ports {led[*]}]

#---------------------------------------------------------------------------
# The board-to-board link.  `rm_rx' is driven by the far
# board's oscillator, so it is asynchronous by definition and there is no
# meaningful setup or hold relationship to constrain - uart_rx double-flops
# it, which is the correct fix and the only one available.  `rm_tx'
# changes once per bit period, thousands of clocks apart.
#---------------------------------------------------------------------------
set_false_path -from [get_ports {rm_rx}] -to [all_registers]
set_false_path -from * -to [get_ports {rm_tx}]

#---------------------------------------------------------------------------
# The JTAG pins.
#
# `top_debug' instantiates the In-System Sources & Probes megafunction, which
# drags in the JTAG hub and with it altera_reserved_tdi / tms / tdo.  Quartus
# constrains the altera_reserved_tck DOMAIN itself, but it does NOT constrain
# these three PORTS - so without the cuts below they are the only genuinely
# unconstrained paths in the design.  Verified with report_ucp: two
# unconstrained input ports (tdi, tms) and one output port (tdo), with 40
# unconstrained input port paths behind them.
#
# Cutting them is correct rather than merely convenient.  JTAG is driven by
# the USB-Blaster at its own pace, asynchronously to CLOCK_50, and nothing in
# the bus has a timing relationship with it; the hub is Altera's and is not
# ours to constrain.  There is no arrival time that would mean anything.
#
# `tcl/sta_check.tcl' fails the build if anything else ever becomes
# unconstrained, so this cut cannot quietly grow to cover a real path.
#---------------------------------------------------------------------------
set_false_path -from [get_ports {altera_reserved_tdi}] -to [all_registers]
set_false_path -from [get_ports {altera_reserved_tms}] -to [all_registers]
set_false_path -from * -to [get_ports {altera_reserved_tdo}]
