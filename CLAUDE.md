# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout — two separate designs

This repo holds **two independent implementations** of the same system bus. They do not share files.

| Location | Language | Status |
|---|---|---|
| `System_Bus_Final/` | Verilog-2001 (`.v`) | **Active.** The real project. All work goes here. |
| Repo root (`*.sv`), `src/`, `tb/` | SystemVerilog (`.sv`) + earlier `.v` copies | Legacy. Superseded; do not extend. |

`src/` and `tb/` contain earlier `.v` copies of the `System_Bus_Final/` modules. They have **diverged** — `src/slave_fast_ram.v` and `src/slave_0_split_4k.v` still use full-depth memories with a reset loop over all 4096 words (which forces LUT/FF memory instead of M9K). Treat `System_Bus_Final/` as the only source of truth.

The root `System_Bus_Design.qsf` targets `EP4CE115F29C7`; `System_Bus_Final/System_Bus_Final.qsf` targets **`EP4CE22F17C6`** (a DE0-Nano) with top **`top_debug`**. Only the latter matters.

## Critical: `System_Bus_Final/` is gitignored

`.gitignore:6` contains `System_Bus_Final/`. Files there are tracked only because they were committed *before* that rule was added. **Any new file added to that directory is silently ignored by git** — it will not show in `git status` and will never be committed.

Currently untracked-and-invisible: `bus_issp_driver.v`, `top_debug.v`, `bus_interconnect.v`, `tb_slave_fast_ram.v`, `issp_bus_test.tcl`, `issp_bus_lib.tcl`, `issp_console.tcl`, `run_issp_test.tcl`, `uart_tx.v`, `uart_rx.v`, `master_node_uart.v`, `tb_uart_remote.v`.

To add a file there, either `git add -f <path>`, or fix the root cause by narrowing the rule to build artifacts:

```
System_Bus_Final/db/
System_Bus_Final/incremental_db/
System_Bus_Final/output_files/
System_Bus_Final/simulation/
```

The tracked tree already includes ~100 Quartus `db/`, `incremental_db/` and `output_files/` blobs that churn on every compile — that is why most of `git status` is noise.

## Commands

No build script or Makefile — invoke tools directly. **`bus_interconnect.v` must be in every RTL compile** since the refactor.

Full integration testbench:

```bash
cd System_Bus_Final
iverilog -g2005 -o /tmp/top.vvp tb_top_bus_system.v top_bus_system.v bus_interconnect.v \
  address_decoder.v arbiter_2m_split.v master_node.v slave_0_split_4k.v \
  slave_fast_ram.v master_node_uart.v uart_tx.v uart_rx.v
vvp /tmp/top.vvp
```

Two-board remote access (crossed UART between two bus systems):

```bash
iverilog -g2005 -o /tmp/rem.vvp tb_uart_remote.v top_bus_system.v bus_interconnect.v \
  address_decoder.v arbiter_2m_split.v master_node.v master_node_uart.v \
  slave_0_split_4k.v slave_fast_ram.v uart_tx.v uart_rx.v
vvp /tmp/rem.vvp
```

The slaves instantiate `altsyncram`, which Quartus's own `altera_mf.v` cannot
provide to iverilog (it does not parse). Use a behavioural stub for it.

Single unit testbenches (each prints its own banner and error count):

```bash
iverilog -g2005 -o /tmp/dec.vvp tb_address_decoder.v  address_decoder.v   && vvp /tmp/dec.vvp
iverilog -g2005 -o /tmp/arb.vvp tb_arbiter_2m_split.v arbiter_2m_split.v  && vvp /tmp/arb.vvp
iverilog -g2005 -o /tmp/mst.vvp tb_master_node.v      master_node.v       && vvp /tmp/mst.vvp
iverilog -g2005 -o /tmp/s0.vvp  tb_slave_0_split_4k.v slave_0_split_4k.v  && vvp /tmp/s0.vvp
iverilog -g2005 -o /tmp/sfr.vvp tb_slave_fast_ram.v   slave_fast_ram.v    && vvp /tmp/sfr.vvp
iverilog -g2005 -o /tmp/ram.vvp tb_ram_4k.v           ram_4k.v            && vvp /tmp/ram.vvp
```

Testbenches exit 0 regardless of result — **grep the output**, don't trust the exit code:

```bash
vvp /tmp/top.vvp | grep -E "ERROR|FAILED|PASSED"
```

Synthesis / fit / bitstream (Quartus Prime Lite 24.1std at `~/programs/intelFPGA_lite/24.1std/`):

```bash
quartus_map System_Bus_Final --part=EP4CE22F17C6
quartus_fit System_Bus_Final
quartus_asm System_Bus_Final
```

Run these on a **copy** in a scratch directory. Compiling in place rewrites the ~100 tracked `db/` and `output_files/` blobs and floods `git status`.

`top_debug` cannot be elaborated by `iverilog` as-is — it instantiates the Altera `altsource_probe` megafunction. Use a behavioural stub, or compile against `~/programs/intelFPGA_lite/24.1std/quartus/eda/sim_lib/altera_mf.v`.

## Hardware test over JTAG

`issp_bus_test.tcl` runs the `tb_top_bus_system.v` cases against the real board through the In-System Sources & Probes instance `BUS0`. It exits 0 on pass, 1 on failure.

```bash
cd System_Bus_Final
quartus_stp -t issp_bus_test.tcl          # quartus_stp ONLY
quartus_stp -t issp_bus_test.tcl -gap     # + decode-gap deadlock demo
```

**`quartus_stp` is the only interpreter that works.** Verified on 24.1std: `::quartus::jtag` and `::quartus::insystem_source_probe` are rejected outright by `quartus_sh` and absent entirely from the Quartus GUI's Tcl console — no `load_package` or `package require` can load them there. The script gates on this and fails fast with that message.

`issp_console.tcl` is the interactive counterpart: `w` / `r` / `both` / `sweep`
against any slave+offset. `both` fires both masters from one source write so
they reach the arbiter on the same clock. Connection and the bit map are shared
via `issp_bus_lib.tcl` — change the bit layout there, not in the two callers.

To launch it *from* the GUI, use `run_issp_test.tcl` (Tools > Tcl Scripts, or `source` it in the console). The GUI interpreter has `exec` even though it lacks the ISSP packages, so that wrapper shells out to `quartus_stp` and echoes the output back into the console.

**Close the In-System Sources & Probes Editor tab before any run** — an open editor holds the JTAG session and the test cannot claim it.

Two ordering constraints inside the script, both easy to reintroduce:

- `get_insystem_source_probe_instance_info` opens a **transient session of its own**, so it must be called *before* `start_insystem_source_probe`, never after.
- Bit 0 of each 24-bit source slice is the `go` level and is deliberately excluded from the command payload concat (`src[23:1]`, not `src[23:0]`). Widening it back over bit 0 aliases `we` onto `go` and makes reads impossible.

## Architecture

Single-master-at-a-time shared bus: 14-bit address, 8-bit data, 2 masters, 4 slaves, with a **split-transaction** protocol.

```
top_debug                     synthesis top; only clk, rst_n, led[7:0] and the
 |                            two bridge pins leave the device
 +- bus_issp_driver           JTAG source/probe front-end (instance "BUS0")
 +- top_bus_system
     +- master_node_uart m0   command FSM + UART client/server (remote access)
     +- master_node      m1   command FSM, local only
     +- bus_interconnect      arbiter + decoder + both muxes
     |    +- arbiter_2m_split
     |    +- address_decoder
     +- slave_0_split_4k      0x0000-0x0FFF  splits every read
     +- slave_fast_ram #12    0x1000-0x1FFF  single cycle
     +- slave_fast_ram #11    0x2000-0x27FF  single cycle
     (0x3000-0x3FFF unmapped: acknowledged with 0x00, never hangs)
```


`0x2800-0x2FFF` is an unmapped gap. `bus_interconnect` owns the granted-master mux onto `bus_addr/bus_wdata/bus_we`, the one-hot slave selects, and the `bus_rdata`/`bus_ready` return mux; `top_bus_system` keeps the masters and slaves and wires them together.

**Transaction flow.** `master_node` latches the command, raises `bus_req`, waits for `bus_gnt`, then drives `m_addr/m_wdata/m_we/m_valid`. FSM: `IDLE -> REQ -> DRIVE -> WAIT -> DONE`, plus `SPLIT_W`.

**Split protocol.** On a read, `slave_0_split_4k` pulses `split_req`, saves the address, and counts 3 cycles in the background. The arbiter sees `split_req` while in `GRANT_M0`, raises `m0_split_notify`, and releases the bus to M1. The master drops `m_valid` but *keeps* `bus_req` high and parks in `SPLIT_W`. When the slave pulses `split_done`, the arbiter latches `split_data_ready` and re-grants M0, which re-issues the read; the slave matches the saved address and returns the data.

**Arbitration** is fixed priority M0 > M1, with one override: an M0 split whose data has arrived jumps the queue. Because M0 is granted first, it often *finishes* first even on the slow split slave while M1 absorbs the wait — latency ordering between the two masters is an arbitration artifact, not a correctness property. Do not assert on it.

Measured on hardware: fast read 6 clocks, uncontended split read 12 clocks.

## Remote access over UART

`uart_cmd_inject` and `slave_uart_tx` are **deleted**. Master 0 is now
`master_node_uart`, which wraps the ordinary `master_node` core and adds a UART
client and server. Master 1 stays a plain `master_node`.

A transaction is local (`cmd_remote = 0`) or remote (`cmd_remote = 1`). A remote
one is carried to the other board and executed on ITS bus; reads bring data
back, writes bring an acknowledgement, so `cmd_done` means the far side really
did it. The same block is also the server for the other board's requests, which
it runs through the same core — so one core serves both, and `srv_active` muxes
the command inputs.

Wire format, 8N1, tagged so the two frame kinds cannot be confused:

```
REQUEST   0xA5, b0, b1, b2      b2:b1:b0 = the 24-bit command
RESPONSE  0x5A, data            read data, or 0x00 acknowledging a write
```

24-bit command: `cmd[23:16]`=wdata, `cmd[15:2]`=addr, `cmd[1]`=we, `cmd[0]` reserved.

**Responses take priority over requests** in the shared transmitter. Without
that, two boards issuing remote commands on the same instant would both sit in
`C_WAIT` and neither would answer.

**`RESP_TIMEOUT`** (default 500000 clocks, ~10 ms) completes a remote
transaction with `cmd_error` set if no answer arrives, so an unplugged cable
reports a failure instead of hanging the bus. `CLKS_PER_BIT` (default 434) and
`RESP_TIMEOUT` are parameters on `top_debug`, `top_bus_system` and
`master_node_uart` so testbenches can shrink both.

ISSP wiring: `src[49]` drives `cmd_remote`, `prb[37]` reports the sticky
`cmd_error`. Widths stay 50/38.

Board-to-board: `ext_tx_serial` (`C3`) of each to `ext_rx_serial` (`D3`) of the
other, plus common ground.

`tb_uart_remote.v` instantiates two complete bus systems with a crossed link and
checks remote write, remote read, both directions, a remote read of the split
slave, and that cutting the link produces `cmd_error` rather than a hang.

## Known quirks

Deliberate or known-broken.

- **Memories are full-depth `altsyncram` blocks** (4K / 4K / 2K) inferred into M9K. An earlier revision used 64-word arrays indexed on the upper address bits, so offsets aliased in blocks of 0x40 — that is gone; every offset is now its own word.
- **A decode-gap access deadlocks the bus.** `decode_err` is driven by `address_decoder` but left unconnected in `bus_interconnect`. An access to `0x2800-0x2FFF` asserts no slave select, so `bus_ready` never rises and the master waits forever holding `bus_req`. There is no timeout, and nothing but `rst_n` recovers it — `soft_rst` (`src[48]`) only clears the ISSP driver's status flags, not the fabric.
- **Slave 0 is effectively M0-only.** `m1_split_notify` is declared but never assigned 1 (`arbiter_2m_split.v:61` is its only write) — the arbiter reacts to `s0_split_req` solely in the `GRANT_M0` state. M1 reading `0x0000-0x0FFF` will split and hang. The slave also tracks a single outstanding split with no master ID.
- **`ram_4k.v` is dead code** — nothing instantiates it, and it is not in the `.qsf` file list (though `tb_ram_4k.v` is).
- **No `.sdc` exists**, so `clk` has no `create_clock` and Fmax is never verified. The 50 MHz in the testbenches is simulation-only.

### Fixed, worth not reintroducing

`slave_0_split_4k` used to start a **phantom split** after every split read: the master holds `m_valid` (hence `sel`) through both `DRIVE` and `WAIT`, so the cycle after the slave served the split data and cleared `data_ready_for_fetch`, the `else if` fired again and opened a second split for the same address. A repeat read of that address was then served in 5 clks without splitting, and a read of any *different* slave-0 address matched neither branch — no `ready`, no `split_req` — and deadlocked the bus permanently. The `!sel_d` guard at `slave_0_split_4k.v:62` restricts split-start to the first cycle of an access. Simulation missed this for a long time because the testbench only ever reads `0x0010`.

## Board bring-up (DE0-Nano)

Pin assignments live in the `.qsf`. `STRATIX_DEVICE_IO_STANDARD` is `3.3-V LVTTL` (Quartus otherwise defaults the banks to 2.5 V), and `RESERVE_ALL_UNUSED_PINS` is `AS INPUT TRI-STATED` — the board wires SDRAM, EPCS, ADC and the accelerometer to unused pins, and Quartus' default of "as output driving ground" would fight those devices.

`led[7:0]` shows `m0_cmd_rdata`. `master_node` captures that **only on reads** (`master_node.v:89` and `:108` are gated on `!reg_we`), so writes leave the display unchanged. Without that gate the slaves' `dout <= din` write echo would clobber it; the bridge would additionally lag by one because it does `dout <= dummy_reg` while `dummy_reg <= din`.

The full source/probe bit map is in `README.md` and in the header comment of `bus_issp_driver.v` — keep those in sync when changing the bit layout.
