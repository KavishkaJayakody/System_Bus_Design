#!/usr/bin/env bash
#===========================================================================
# sim/run_icarus.sh -- run every testbench under Icarus Verilog.
#
#   ./sim/run_icarus.sh            run all testbenches
#   ./sim/run_icarus.sh arbiter    run only tb_arbiter
#
# The testbenches print their own PASS/FAIL banner and always exit 0, so this
# script greps the output and sets the exit status itself: 0 = everything
# passed, 1 = at least one testbench failed.  Use it in that spirit - do not
# trust vvp's exit code.
#===========================================================================
set -u

cd "$(dirname "$0")/.." || exit 1
RTL=rtl
TB=tb
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# Source groups, mirroring the module split: the serial primitives, the bus,
# the peripherals, the board layer.
SER="$RTL/shift_ser.v $RTL/shift_deser.v"

# Everything inside system_bus - this is what tb_system_bus compiles, with
# no master and no memory anywhere in the list.
BUS="$RTL/system_bus.v $RTL/arbiter.v $RTL/addr_decoder.v $RTL/bus_mux.v \
     $RTL/default_slave.v $RTL/shift_deser.v"

# The whole system: masters + bus + slaves, composed once in bus_top.v.
# top_debug and both integration testbenches instantiate that one wrapper.
COMMON="$RTL/bus_top.v $RTL/master.v $RTL/bus_bridge.v $RTL/slave.v \
        $RTL/uart_tx.v $RTL/uart_rx.v $BUS $RTL/shift_ser.v"

# The JTAG debug layer.  altsource_probe_stub.v is SIMULATION ONLY - it
# stands in for the Altera megafunction, which iverilog cannot elaborate.
DBG="$RTL/bus_issp_driver.v $TB/altsource_probe_stub.v"

# name : sources
run_one () {
    local name=$1; shift
    printf '\n### %s\n' "$name"
    if ! iverilog -g2005 -I "$RTL" -o "$OUT/$name.vvp" "$TB/tb_$name.v" "$@" 2>&1; then
        echo "  COMPILE FAILED"
        return 1
    fi
    local log="$OUT/$name.log"
    vvp "$OUT/$name.vvp" | tee "$log"
    if grep -qE 'FAILED' "$log"; then return 1; fi
    if ! grep -qE 'PASSED' "$log"; then
        echo "  no PASS banner found"
        return 1
    fi
    return 0
}

FAIL=0
want=${1:-all}

try () {
    local n=$1; shift
    if [ "$want" = all ] || [ "$want" = "$n" ]; then
        run_one "$n" "$@" || FAIL=1
    fi
}

# --- serial primitives ---------------------------------------------------
try shift_ser     "$RTL/shift_ser.v"
try shift_deser   "$RTL/shift_deser.v"

# --- inside the bus ------------------------------------------------------
try addr_decoder  "$RTL/addr_decoder.v"
try arbiter       "$RTL/arbiter.v"
try bus_mux       "$RTL/bus_mux.v"
try default_slave "$RTL/default_slave.v"

# --- the bus itself, with no master and no memory attached ---------------
try system_bus    $BUS

# --- the peripherals -----------------------------------------------------
try master        "$RTL/master.v" $SER
try slave         "$RTL/slave.v" "$RTL/shift_deser.v"

# --- JTAG debug front-end ------------------------------------------------
try bus_issp_driver $DBG $COMMON

# --- integration ---------------------------------------------------------
try integration   $COMMON

# --- two boards, crossed UART link ---------------------------------------
try uart_remote   $COMMON

# --- the synthesis top level, elaborated for real ------------------------
try top_debug     "$RTL/top_debug.v" $DBG $COMMON

echo
echo "======================================================"
if [ "$FAIL" -eq 0 ]; then
    echo " ALL TESTBENCHES PASSED"
else
    echo " REGRESSION FAILED"
fi
echo "======================================================"
exit $FAIL
