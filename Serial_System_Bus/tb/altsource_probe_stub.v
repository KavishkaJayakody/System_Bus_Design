//==========================================================================
// altsource_probe_stub.v -- SIMULATION ONLY.  Not in the .qsf.
//
// Behavioural stand-in for the Altera altsource_probe megafunction, so
// bus_issp_driver can be elaborated and tested under Icarus.  Quartus's own
// altera_mf.v does not parse under iverilog, and the real megafunction is
// a JTAG endpoint with no simulation behaviour worth having anyway.
//
// A testbench drives the source exactly as the Tcl host does, by assigning
// to this module's `source' register through a hierarchical reference:
//
//     dut.u_issp.source = 56'h...;
//
// and reads the probe vector back the same way.  That makes the testbench a
// faithful stand-in for issp_bus_lib.tcl: if a sequence works here, the same
// sequence works over JTAG.
//
// Port and parameter names match the real megafunction so the instantiation
// in bus_issp_driver.v needs no `ifdef.
//==========================================================================
`timescale 1ns/1ps

module altsource_probe #(
    parameter sld_auto_instance_index = "YES",
    parameter instance_id             = "NONE",
    parameter source_width            = 1,
    parameter probe_width             = 1,
    parameter source_initial_value    = "0",
    parameter enable_metastability     = "NO"
) (
    output reg  [source_width-1:0] source,
    input  wire [probe_width-1:0]  probe,
    input  wire                    source_clk,
    input  wire                    source_ena
);

    initial source = {source_width{1'b0}};

endmodule
