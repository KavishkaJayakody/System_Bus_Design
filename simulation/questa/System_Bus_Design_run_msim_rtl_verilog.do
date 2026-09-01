transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/bus_pkg.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/top_level.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/arbiter.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/addr_decoder.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/bus_mux.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/slave1_split.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/slave_simple.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/master.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/master1_stimulus.sv}
vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/master2_stimulus.sv}

vlog -sv -work work +incdir+C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design {C:/Users/amoda/OneDrive/Documents/GitHub/System_Bus_Design/tb_arbiter.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  tb_arbiter

add wave *
view structure
view signals
run -all
