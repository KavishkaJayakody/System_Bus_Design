create_clock -name clk -period 20.000 [get_ports {clk}]
derive_pll_clocks
derive_clock_uncertainty
set_false_path -from [get_ports {rst_n}]
set_false_path -from [get_ports {ext_rx_serial}]
set_false_path -to   [get_ports {ext_tx_serial}]
set_false_path -to   [get_ports {led[*]}]
