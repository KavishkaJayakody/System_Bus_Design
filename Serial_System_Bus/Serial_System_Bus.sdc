#===========================================================================
# Serial_System_Bus.sdc -- timing constraints for the DE2-115 build
#
# One clock, one domain.  CLOCK_50 is the DE2-115's 50 MHz oscillator on the
# dedicated clock pin Y2; everything in the design is clocked by it.
#
# Without this file Quartus reports "no clocks defined", analyses nothing,
# and Fmax is never checked - so the 50 MHz in the testbenches would be an
# assumption rather than a verified result.
#===========================================================================

create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]

# Model the small amount of jitter/skew the fitter should budget for.
derive_clock_uncertainty

#---------------------------------------------------------------------------
# All I/O on this board is asynchronous to the bus: slide switches and
# pushbuttons are set by a person and are synchronised inside the design
# (reset_ctrl, debouncer and the SW synchroniser in de2_top); LEDs and the
# seven-segment displays are looked at by a person.  None of it has a setup
# or hold relationship worth constraining, so cut those paths and let the
# analyser concentrate on the internal bus logic.
#---------------------------------------------------------------------------
set_false_path -from [get_ports {KEY[*]}] -to [all_registers]
set_false_path -from [get_ports {SW[*]}]  -to [all_registers]

set_false_path -from * -to [get_ports {LEDR[*]}]
set_false_path -from * -to [get_ports {LEDG[*]}]
set_false_path -from * -to [get_ports {HEX0[*]}]
set_false_path -from * -to [get_ports {HEX1[*]}]
set_false_path -from * -to [get_ports {HEX2[*]}]
set_false_path -from * -to [get_ports {HEX3[*]}]
set_false_path -from * -to [get_ports {HEX4[*]}]
set_false_path -from * -to [get_ports {HEX5[*]}]
set_false_path -from * -to [get_ports {HEX6[*]}]
set_false_path -from * -to [get_ports {HEX7[*]}]
