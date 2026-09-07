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

COMMON="$RTL/addr_decoder.v $RTL/arbiter.v $RTL/bus_mux.v $RTL/slave_mem.v \
        $RTL/default_slave.v $RTL/master.v $RTL/bus_top.v"
BOARD="$RTL/de2_top.v $RTL/master_prog.v $RTL/reset_ctrl.v $RTL/debouncer.v \
       $RTL/seg7_hex.v"

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

try addr_decoder  "$RTL/addr_decoder.v"
try arbiter       "$RTL/arbiter.v"
try bus_mux       "$RTL/bus_mux.v"
try slave_mem     "$RTL/slave_mem.v"
try default_slave "$RTL/default_slave.v"
try master        "$RTL/master.v"
try bus_top       $COMMON
try de2_top       $COMMON $BOARD

echo
echo "======================================================"
if [ "$FAIL" -eq 0 ]; then
    echo " ALL TESTBENCHES PASSED"
else
    echo " REGRESSION FAILED"
fi
echo "======================================================"
exit $FAIL
