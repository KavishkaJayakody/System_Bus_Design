# System Bus Design

A shared system bus for the Cyclone IV E (`EP4CE22F17C6`, DE0-Nano), written in Verilog-2001: 14-bit address, 8-bit data, two masters, four slaves, arbitration with a **split-transaction** protocol. The whole design is driven and observed over JTAG — no pins, no board wiring — using Altera In-System Sources & Probes.

**Verified on hardware.** All four integration cases pass on the board, including split-transaction interleaving.

The active project is [`System_Bus_Final/`](System_Bus_Final/). The `.sv` files at the repo root, plus `src/` and `tb/`, are an earlier superseded attempt.

## Architecture

```
top_debug                     synthesis top; only clk, rst_n, led[7:0] and the
 |                            two bridge pins leave the device
 +- bus_issp_driver           JTAG source/probe front-end (instance "BUS0")
 +- top_bus_system
     +- master_node_uart m0   command FSM + UART client/server
     +- master_node      m1   command FSM, local only
     +- bus_interconnect      arbiter + address decoder + both muxes
     |    +- arbiter_2m_split
     |    +- address_decoder
     +- slave_0_split_4k      0x0000-0x0FFF
     +- slave_fast_ram #12    0x1000-0x1FFF
     +- slave_fast_ram #11    0x2000-0x27FF
     (0x3000-0x3FFF unmapped, acknowledged with 0x00)
```


`bus_interconnect` owns the granted-master mux, the one-hot slave selects and the response return mux. `top_bus_system` holds the masters and slaves and wires them together.

| Range | Slave | Behaviour |
|---|---|---|
| `0x0000–0x0FFF` | `slave_0_split_4k` | Writes 1 cycle; **every read splits** (3-cycle background latency) |
| `0x1000–0x1FFF` | `slave_fast_ram` (12-bit) | Single cycle |
| `0x2000–0x27FF` | `slave_fast_ram` (11-bit) | Single cycle |
| `0x2800–0x2FFF` | *unmapped* | Decode gap — see [Known limitations](#known-limitations) |
| `0x3000–0x3FFF` | *(unmapped)* | UART moved into Master 0; reads `0x00`, never hangs |

Arbitration is fixed priority **M0 > M1**, with one override: an M0 split transaction whose data has arrived jumps ahead of a pending M1 request.

Measured on hardware: **fast read 6 clocks, uncontended split read 12 clocks.**

---

## In-System Sources & Probes — bit patterns

Open the **In-System Sources & Probes Editor** in Quartus and select instance ID **`BUS0`**. Sources are what you drive; probes are what you read back.

### Sources — 50 bits

| Bits | Width | Name | Meaning |
|---|---|---|---|
| `src[0]` | 1 | `m0_go` | Level. A **0→1 edge** launches one Master 0 command. |
| `src[1]` | 1 | `m0_cmd_we` | `1` = write, `0` = read |
| `src[15:2]` | 14 | `m0_cmd_addr` | Bus address |
| `src[23:16]` | 8 | `m0_cmd_wdata` | Write data |
| `src[24]` | 1 | `m1_go` | Level. A **0→1 edge** launches one Master 1 command. |
| `src[25]` | 1 | `m1_cmd_we` | `1` = write, `0` = read |
| `src[39:26]` | 14 | `m1_cmd_addr` | Bus address |
| `src[47:40]` | 8 | `m1_cmd_wdata` | Write data |
| `src[48]` | 1 | `soft_rst` | Clears the sticky `done`/`collision` flags |
| `src[49]` | 1 | `m0_remote` | 1 = run this Master 0 command on the **other board** |

```
 49   48   47........40 39..........26 25   24   23........16 15...........2  1    0
┌────┬────┬────────────┬──────────────┬────┬────┬────────────┬──────────────┬────┬────┐
│rem │srst│ m1_wdata   │  m1_addr     │m1we│m1go│ m0_wdata   │  m0_addr     │m0we│m0go│
└────┴────┴────────────┴──────────────┴────┴────┴────────────┴──────────────┴────┴────┘
                    └──────── Master 1 ────────┘└──────── Master 0 ────────┘
```

Each master occupies a 24-bit slice: `go` at the bottom, then `we`, `addr`, `wdata`.

### Probes — 38 bits

| Bits | Width | Name | Meaning |
|---|---|---|---|
| `prb[7:0]` | 8 | `m0_rl` | Last read data captured for Master 0 |
| `prb[8]` | 1 | `m0_done_s` | Sticky "command finished". Clears when `m0_go` drops. |
| `prb[9]` | 1 | `m0_busy` | Master 0 command in flight |
| `prb[17:10]` | 8 | `m1_rl` | Last read data captured for Master 1 |
| `prb[18]` | 1 | `m1_done_s` | Sticky "command finished". Clears when `m1_go` drops. |
| `prb[19]` | 1 | `m1_busy` | Master 1 command in flight |
| `prb[27:20]` | 8 | `m0_lat` | Master 0 latency in clocks, saturating at `0xFF` |
| `prb[35:28]` | 8 | `m1_lat` | Master 1 latency in clocks, saturating at `0xFF` |
| `prb[36]` | 1 | `collision` | Sticky: both masters were in flight at once |
| `prb[37]` | 1 | `m0_error` | Sticky: last remote command timed out |

```
 37   36   35........28 27........20 19   18   17........10  9    8    7.........0
┌────┬────┬────────────┬────────────┬────┬────┬────────────┬────┬────┬────────────┐
│err │coll│  m1_lat    │  m0_lat    │m1by│m1dn│  m1_rdata  │m0by│m0dn│  m0_rdata  │
└────┴────┴────────────┴────────────┴────┴────┴────────────┴────┴────┴────────────┘
```

### Worked examples

**Write `0xC5` to Fast RAM at `0x1000` (Master 0).** Set `wdata=0xC5`, `addr=0x1000`, `we=1`, then pulse `go`:

```
src[23:16] = 0xC5      src[15:2] = 0x1000      src[1] = 1      src[0] = 0 → 1
```

As a full source word, `0x00C54002` to arm, then `0x00C54003` to fire. Watch `prb[9]` (`m0_busy`) rise then fall and `prb[8]` (`m0_done_s`) latch high.

**Read it back.** Same address, `we=0`. Drop `src[0]` first so `m0_done_s` clears, then raise it:

```
src[15:2] = 0x1000      src[1] = 0      src[0] = 0 → 1
```

`0x00004000` to arm, `0x00004001` to fire. The result appears in `prb[7:0]` — expect `0xC5`, in about 6 clocks.


**Demonstrate a split transaction.** Read `0x0010` (Slave 0) on M0 and `0x1000` (Slave 1) on M1 *in the same source write*, so both `go` edges land on one clock. Slave 0 splits, the arbiter hands the bus to M1, and both complete. `prb[36]` (`collision`) latches high, proving they overlapped.

### Notes on use

- `go` is a **level**, not a pulse. Drop it low before re-issuing, or no new edge is seen.
- `soft_rst` (`src[48]`) clears `done`/`collision` only — **not** `busy`, and not the bus fabric. If a master wedges, only `rst_n` (KEY[0]) recovers it.
- `m0_lat == 0xFF` with `m0_busy` still high means the transaction never completed — see the decode gap below.

---

## Remote access over UART

Master 0 is `master_node_uart`: an ordinary `master_node` core wrapped with a
UART client and server. Set `cmd_remote` and the same address and data are
carried to the **other board** and executed on its bus — reads bring the data
back, writes bring an acknowledgement. The block is symmetric, so each board
also serves the other's requests through its own core.

```
   Board A                                  Board B
   master_node_uart  --- C3 --> D3 ---  master_node_uart
        (client)     <-- D3 --  C3 --      (server)
```

Wire format, 8N1, tagged so the two frame kinds cannot be confused:

```
REQUEST   0xA5, b0, b1, b2      b2:b1:b0 = the 24-bit command
RESPONSE  0x5A, data            read data, or 0x00 acknowledging a write
```

24-bit command — the same field order as one ISSP source slice:

```
 23........16 15...........2  1    0
┌────────────┬──────────────┬────┬────┐
│   wdata    │     addr     │ we │rsvd│
└────────────┴──────────────┴────┴────┘
```

Because the command carries the full 14-bit address, a remote transaction can
reach **any** slave on the far board, not just one window.

If the far board does not answer within `RESP_TIMEOUT` clocks (default 500000,
~10 ms) the transaction completes with `cmd_error` set, so an unplugged cable
reports a failure instead of hanging the bus. Responses take priority over
requests in the shared transmitter, so two boards issuing remote commands at the
same instant still service each other rather than deadlocking.

Framing is 115200 baud from a 50 MHz clock (`CLKS_PER_BIT = 434`). Both that and
`RESP_TIMEOUT` are parameters on `top_debug`, `top_bus_system` and
`master_node_uart`, so testbenches can shrink them.

Wiring between two boards: `ext_tx_serial` (GPIO_0[1], `C3`) of each to
`ext_rx_serial` (GPIO_0[0], `D3`) of the other, plus a common ground.

`tb_uart_remote.v` runs two complete bus systems with a crossed link and checks
remote write, remote read, both directions, a remote read of the far board's
split slave, and that cutting the link yields `cmd_error` rather than a hang.

## Running the hardware test

`System_Bus_Final/issp_bus_test.tcl` runs the integration cases against the board and exits 0 on pass, 1 on failure.

```bash
cd System_Bus_Final
quartus_stp -t issp_bus_test.tcl          # quartus_stp ONLY
quartus_stp -t issp_bus_test.tcl -gap     # + decode-gap deadlock demo
```

**`quartus_stp` is the only interpreter that works.** The `::quartus::jtag` and `::quartus::insystem_source_probe` packages are rejected by `quartus_sh` and absent from the Quartus GUI's Tcl console — no `load_package` can help there.

### Interactive console

For poking the bus by hand, `issp_console.tcl` gives you a prompt:

```bash
quartus_stp -t issp_console.tcl
```

```
bus> w 1 40 5A                 write 0x5A to slave 1, offset 0x40
bus> r 1 40                    read it back
bus> both 1 80 C1 2 20 D2      BOTH masters write on the same clock edge
bus> sweep 1                   write+read 8 words across a slave
bus> rem on                    send M0 commands to the OTHER board
bus> m 1                       switch to Master 1
bus> map                       show the address map
bus> h                         help          q  quit
```

`w`, `r` and `both` prompt for slave / offset / data when given no arguments.
Offsets are hex and relative to the slave base, and every result reports the
latency plus which of the 64 physical words the address actually landed on.

`both` is the one that shows arbitration: a single JTAG source write sets both
`go` bits, so the two requests reach the arbiter on the same clock, and the
sticky `collision` probe bit confirms they really contended. Issuing two
separate commands could never overlap — JTAG round trips are milliseconds
apart while a bus transaction takes nanoseconds.

Shared plumbing for both scripts (connection, the source/probe bit map,
`bus_cmd`) lives in `issp_bus_lib.tcl`.

To launch it **from the Quartus GUI**, use the wrapper instead:

> `Tools → Tcl Scripts…` → select `run_issp_test.tcl` → **Run**
>
> or in the Tcl Console: `source {.../System_Bus_Final/run_issp_test.tcl}`

The GUI interpreter has `exec` even though it lacks the ISSP packages, so the wrapper shells out to `quartus_stp` and echoes the output back into the console. Set `RUN_GAP 1` at its top for the gap demo.

**Close the In-System Sources & Probes Editor tab first** — an open editor holds the JTAG session and the test cannot claim it.

`TEST_DELAY_MS` at the top of `issp_bus_test.tcl` holds 1 s after each test so the LEDs can be read; set it to `0` for fast scripted runs.

## Board pin assignments (DE0-Nano)

| Signal | Pin | Board net |
|---|---|---|
| `clk` | `R8` | CLOCK_50, 50 MHz oscillator |
| `rst_n` | `J15` | KEY[0], active low with pull-up |
| `ext_rx_serial` | `D3` | GPIO_0[0], JP1 pin 1 — UART RX into Master 0 |
| `ext_tx_serial` | `C3` | GPIO_0[1], JP1 pin 2 — UART TX out of Master 0 |
| `led[0..7]` | `A15 A13 B13 A11 D1 F3 B1 L3` | LED[7:0], active high |

All 3.3-V LVTTL. Unused pins are reserved **as input tri-stated** — the board wires SDRAM, EPCS, ADC and the accelerometer to pins this design does not use, and Quartus' default of "as output driving ground" would fight those devices.

`led[7:0]` shows the byte from the last Master 0 **read**; writes leave the display unchanged.

## Simulation

Icarus Verilog, run from `System_Bus_Final/`. Each testbench prints its own banner and error count.

```bash
cd System_Bus_Final

# Full integration test
iverilog -g2005 -o /tmp/top.vvp tb_top_bus_system.v top_bus_system.v bus_interconnect.v \
  address_decoder.v arbiter_2m_split.v master_node.v slave_0_split_4k.v \
  slave_fast_ram.v master_node_uart.v uart_tx.v uart_rx.v
vvp /tmp/top.vvp

# Two-board remote access
iverilog -g2005 -o /tmp/rem.vvp tb_uart_remote.v top_bus_system.v bus_interconnect.v \
  address_decoder.v arbiter_2m_split.v master_node.v master_node_uart.v \
  slave_0_split_4k.v slave_fast_ram.v uart_tx.v uart_rx.v
vvp /tmp/rem.vvp

# A single unit test
iverilog -g2005 -o /tmp/arb.vvp tb_arbiter_2m_split.v arbiter_2m_split.v && vvp /tmp/arb.vvp
```

Testbenches exit `0` regardless of result — grep for `ERROR` / `FAILED` rather than checking the exit code.

Synthesis and bitstream with Quartus Prime Lite:

```bash
quartus_map System_Bus_Final --part=EP4CE22F17C6
quartus_fit System_Bus_Final
quartus_asm System_Bus_Final
```

Run these on a copy: compiling in place rewrites ~100 tracked Quartus build files.

`top_debug` needs the Altera `altsource_probe` megafunction, so `iverilog` can only elaborate it against `quartus/eda/sim_lib/altera_mf.v` or a behavioural stub.

## Known limitations

- **An access to the `0x2800–0x2FFF` gap deadlocks the bus.** `decode_err` is generated but never connected, so no slave select asserts, `bus_ready` never rises, and the master waits forever. There is no timeout, and only `rst_n` recovers it.
- **Slave 0 only works for Master 0.** `m1_split_notify` is never asserted, so an M1 read of `0x0000–0x0FFF` splits and then hangs. The slave also tracks only one outstanding split, with no master ID.
- **No `.sdc` file**, so `clk` is unconstrained and Fmax is never verified.
- **`ram_4k.v` is dead code** — nothing instantiates it.

## Repository notes

`.gitignore` ignores `System_Bus_Final/` wholesale. Existing files there are tracked only because they predate that rule — **new files added to that directory are silently ignored by git**. Use `git add -f`, or narrow the rule to `System_Bus_Final/db/`, `incremental_db/`, `output_files/` and `simulation/`.
