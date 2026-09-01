# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout — two separate designs

This repo holds **two independent implementations** of the same system bus. They do not share files.

| Location | Language | Status |
|---|---|---|
| `System_Bus_Final/` | Verilog-2001 (`.v`) | **Active.** The real project. All work goes here. |
| Repo root (`*.sv`), `src/`, `tb/` | SystemVerilog (`.sv`) + earlier `.v` copies | Legacy. Superseded; do not extend. |

`src/` and `tb/` contain earlier `.v` copies of the `System_Bus_Final/` modules. They have **diverged** — `src/slave_fast_ram.v` and `src/slave_0_split_4k.v` still use full-depth memories with a reset loop over all 4096 words (which forces LUT/FF memory instead of M9K). Treat `System_Bus_Final/` as the only source of truth.

The root `System_Bus_Design.qsf` targets `EP4CE115F29C7` with top `tb_arbiter`; `System_Bus_Final/System_Bus_Final.qsf` targets **`EP4CE22F17C6`** with top **`top_debug`**. Only the latter matters.

## Critical: `System_Bus_Final/` is gitignored

`.gitignore:6` contains `System_Bus_Final/`. The existing files there are tracked only because they were committed *before* that rule was added. **Any new file added to `System_Bus_Final/` is silently ignored by git** — it will not show in `git status` and will never be committed.

Currently untracked-and-invisible: `bus_issp_driver.v`, `top_debug.v`, `tb_slave_fast_ram.v`.

To add a file there, either `git add -f <path>`, or fix the root cause by narrowing the rule to build artifacts:

```
System_Bus_Final/db/
System_Bus_Final/incremental_db/
System_Bus_Final/output_files/
System_Bus_Final/simulation/
```

Note the tracked tree already includes ~100 Quartus `db/`, `incremental_db/` and `output_files/` blobs that churn on every compile — that is why most of `git status` is noise.

## Commands

All simulation uses Icarus Verilog (`iverilog`/`vvp`), run from `System_Bus_Final/`. There is no build script or Makefile — invoke directly.

Run a single unit testbench (each prints its own PASS/FAIL banner and an error count):

```bash
cd System_Bus_Final
iverilog -g2005 -o /tmp/dec.vvp tb_address_decoder.v  address_decoder.v      && vvp /tmp/dec.vvp
iverilog -g2005 -o /tmp/arb.vvp tb_arbiter_2m_split.v arbiter_2m_split.v     && vvp /tmp/arb.vvp
iverilog -g2005 -o /tmp/mst.vvp tb_master_node.v      master_node.v          && vvp /tmp/mst.vvp
iverilog -g2005 -o /tmp/s0.vvp  tb_slave_0_split_4k.v slave_0_split_4k.v     && vvp /tmp/s0.vvp
iverilog -g2005 -o /tmp/sfr.vvp tb_slave_fast_ram.v   slave_fast_ram.v       && vvp /tmp/sfr.vvp
iverilog -g2005 -o /tmp/ram.vvp tb_ram_4k.v           ram_4k.v               && vvp /tmp/ram.vvp
```

Full integration testbench:

```bash
cd System_Bus_Final
iverilog -g2005 -o /tmp/top.vvp tb_top_bus_system.v top_bus_system.v address_decoder.v \
  arbiter_2m_split.v master_node.v slave_0_split_4k.v slave_fast_ram.v cross_fpga_bridge_dummy.v
vvp /tmp/top.vvp
```

Testbenches exit 0 regardless of result — **grep the output**, don't trust the exit code:

```bash
vvp /tmp/top.vvp | grep -E "ERROR|FAILED|PASSED"
```

Synthesis check (Quartus Prime Lite 24.1std at `~/programs/intelFPGA_lite/24.1std/`):

```bash
~/programs/intelFPGA_lite/24.1std/quartus/bin/quartus_map System_Bus_Final --part=EP4CE22F17C6
```

Run this on a **copy** of the project in a scratch directory. Running it in place rewrites the ~100 tracked `db/` and `output_files/` blobs and floods `git status`.

`top_debug` cannot be elaborated by `iverilog` as-is — it instantiates the Altera `altsource_probe` megafunction. Either use a behavioural stub for it, or compile against `~/programs/intelFPGA_lite/24.1std/quartus/eda/sim_lib/altera_mf.v`.

## Architecture

Single-master-at-a-time shared bus: 14-bit address, 8-bit data, 2 masters, 4 slaves, with a **split-transaction** protocol. Synthesis top is `top_debug` → `bus_issp_driver` (JTAG control) + `top_bus_system` (the bus).

```
m0_cmd_* ─▶ master_node m0 ─┐          ┌─▶ slave_0_split_4k   0x0000–0x0FFF  (splits every read)
m1_cmd_* ─▶ master_node m1 ─┤          ├─▶ slave_fast_ram #12 0x1000–0x1FFF  (1 cycle)
                   ▲        │ gnt-MUX  ├─▶ slave_fast_ram #11 0x2000–0x27FF  (1 cycle)
              arbiter_2m_split ────────┼─▶ cross_fpga_bridge  0x3000–0x3FFF  (dummy, reads 0xBE)
                                       └─ address_decoder → sel_s0/s1/s2/sel_bridge
                    rdata/ready return MUX, priority-ordered on sel_*
```

`0x2800–0x2FFF` is an unmapped gap.

**Transaction flow.** `master_node` latches the command, raises `bus_req`, waits for `bus_gnt`, then drives `m_addr/m_wdata/m_we/m_valid`. `top_bus_system` MUXes the granted master onto the bus; `address_decoder` turns `bus_addr` into one-hot slave selects; the selected slave returns `rdata`/`ready`. Master FSM: `IDLE → REQ → DRIVE → WAIT → DONE`, plus `SPLIT_W`.

**Split protocol.** On a read, `slave_0_split_4k` pulses `split_req`, saves the address, and counts 3 cycles in the background. The arbiter sees `split_req` while in `GRANT_M0`, raises `m0_split_notify`, and releases the bus to M1. The master drops `m_valid` but *keeps* `bus_req` high and parks in `SPLIT_W`. When the slave pulses `split_done`, the arbiter latches `split_data_ready` and re-grants M0, which re-issues the read; the slave matches the saved address and returns the data.

**Arbitration** is fixed priority M0 > M1, with one override: an M0 split whose data has arrived jumps the queue.

## Known design quirks

These are deliberate or known-broken. Do not "fix" the first one without asking — it is load-bearing for M9K inference.

- **Memories are 64 words, not 4K/2K, and they alias.** Both slave types index with the *upper* 6 address bits (`slave_fast_ram.v:19`, `slave_0_split_4k.v:46`), so `0x1000` and `0x1001` are the same storage word — only every 64th address is distinct. Module names and the address map still advertise the full sizes. Testbenches are written to respect this.
- **A decode-gap access deadlocks the bus.** `decode_err` is driven by `address_decoder` but left unconnected at `top_bus_system.v:118`. An access to `0x2800–0x2FFF` asserts no slave select, so `bus_ready` never rises and the master waits forever holding `bus_req`. There is no timeout anywhere.
- **Slave 0 is effectively M0-only.** `m1_split_notify` is declared but never assigned 1 — the arbiter only reacts to `s0_split_req` in the `GRANT_M0` state (`arbiter_2m_split.v:81`). M1 reading `0x0000–0x0FFF` will split and hang. The slave also tracks a single outstanding split with no master ID.
- **`ram_4k.v` is dead code** — nothing instantiates it, and it is not in the `.qsf` file list (though `tb_ram_4k.v` is).

## Hardware bring-up

`bus_issp_driver` exposes both master command ports over JTAG via `altsource_probe` (instance ID `BUS0`), so the design is driven from the Quartus In-System Sources & Probes Editor with no pins. The full source/probe bit map is in `README.md` and in the header comment of `System_Bus_Final/bus_issp_driver.v` — keep those two in sync when changing the bit layout.

Bit 0 of each 24-bit source slice is the `go` level and is deliberately *excluded* from the command payload concat (`src[23:1]`, not `src[23:0]`). Widening the payload back over bit 0 aliases `we` onto `go` and makes reads impossible.
