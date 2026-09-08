# Serial_System_Bus — 2-master / 3-slave shared bus with split transactions

Synthesisable **serial** shared-bus interconnect for the **Terasic DE2-115**
(`EP4CE115F29C7`, Cyclone IV E), Quartus Prime Lite 24.1std, Verilog-2001.

Two masters, three memory slaves plus a default slave, fixed-priority
arbitration with bus lock, and AHB-style split transactions.

**The address and the data each travel on a single wire**, MSB first, one bit
per clock, shared by every master and every slave. The whole shared bus is
**8 wires**:

```
bus_astream  1   serial address, 16 bits per frame
bus_dstream  1   serial data, 8 bits, half duplex
bus_valid    1   frame marker, high for 16 clocks
bus_we       1   1 = write; also the data wire's direction control
bus_ready    1   completion strobe
bus_resp     2   OKAY / ERROR / SPLIT
master_id    1   tag of the granted master
```

16-bit word address, 8-bit data, so the slaves are 4 KB / 4 KB / 2 KB.

**Nothing in the datapath ever assembles a 16-bit address.** Slaves shift in
only their own offset — 12 bits or 11 — and the decoder matches the slave
prefix bit by bit as it arrives, settling after 5 of the 16 clocks. The one
16-bit address register left in the design feeds the JTAG probe and nothing
else; `OBSERVE_ADDR = 0` removes it.

```
top_debug                        synthesis top (DE2-115)
 |                               wires, two instances and one assign
 +- bus_issp_driver              JTAG debug front-end, instance "SBUS"
 |                                  IN FRONT of the system: it drives the
 |                                  masters' normal command ports
 |
 +- bus_top                      THE SYSTEM - composition only, no logic
     +- master_uart  (m0)        1. parallel command in, SERIAL onto the bus
     |   +- master core             cmd_addr[15:0] -> m_astream (1 wire)
     |   +- uart_tx / uart_rx       cmd_wdata[7:0] -> m_dstream (1 wire)
     |                              plus REMOTE access to the other board
     +- master       (m1)           local only
     |
     +- system_bus               2. THE BUS - no master, no memory in here
     |   +- arbiter                 priority + bus lock + split mask, param on N
     |   +- addr_decoder            SERIAL prefix matcher, one-hot + default
     |   +- bus_mux                 who drives the two shared wires
     |   +- default_slave           unmapped -> ERROR, so the bus never hangs
     |
     +- slave x3                 3. 4 KB @0x0000 (split capable)
                                    4 KB @0x1000,  2 KB @0x2000
```

**Five things leave the device**: `CLOCK_50`, `rst_n` (`KEY[0]`), `led[7:0]`
(`LEDR[7:0]`, master 0's last read data) and the two UART bridge pins that
reach a second board. Everything else goes over JTAG — there are no switches
to set and no scenario to select. Issue a transaction from the host, read the
answer back.

This is the hierarchy the parallel `System_Bus_Final` design uses, name for
name: a thin synthesis top holding the debug front-end and the system as one
instance — `top_debug` + `top_bus_system` there, `top_debug` + `bus_top`
here.

**`bus_top` contains no logic**, only instantiation and wiring, so it cannot
become a place where bus behaviour hides. The three-kinds-of-module rule is
unchanged: `system_bus` is the bus and holds no master and no memory,
`master` and `slave` hold no bus logic, and `tb_system_bus` still tests the
bus with nothing attached at either end.

What the wrapper buys is that the composition is written **once**.
`top_debug`, `tb_integration` and `tb_bus_issp_driver` all instantiate it, so
the testbenches exercise what the board builds by construction rather than by
three copies being kept in step by hand.

`top_debug` itself contains no logic either — wires, the two instances and
one `assign` for the LEDs.

**Three kinds of module, and only serial wires between them.** `system_bus`
is the bus and contains no master and no memory; `master` and `slave` are
peripherals and contain no bus logic. That split is what lets `tb_system_bus`
test the bus on its own, driving both serial interfaces with nothing attached
at either end.

`bus_top` composes the three into a complete system and is the only place
that wiring is written; `top_debug` and both integration testbenches
instantiate it.

| Interface | Address | Data |
|---|---|---|
| master -> bus | `m_astream`, 1 wire per master | `m_dstream`, 1 wire per master |
| bus -> slave | `bus_astream`, 1 shared wire | `bus_dstream`, 1 shared wire |
| slave -> bus | — | `s_dstream`, 1 wire per slave |
| bus -> master | — | `bus_dstream`, the same shared wire |

## Layout

| Path | Contents |
|---|---|
| `rtl/` | synthesisable modules, one per file, plus `bus_defs.vh`; `shift_ser.v` / `shift_deser.v` are the two serial primitives everything else is built from; `bus_top.v` composes the system; `top_debug.v` is the synthesis top; `master_uart.v` + `uart_tx.v` / `uart_rx.v` are the link to a second board |
| `tb/` | a self-checking testbench per module — including `tb_system_bus`, which exercises the bus with no master and no memory attached — plus `tb_integration`, `tb_uart_remote` (two whole boards on a crossed UART link) and `tb_top_debug`, which drives the real synthesis top through its pins and its JTAG source register |
| `sim/` | `run_icarus.sh`, `run_questa.do` |
| `tcl/` | JTAG debug over In-System Sources & Probes — see below |
| `docs/` | [report.pdf](docs/report.pdf) (the engineering report), [design_notes.md](docs/design_notes.md), [address_map.md](docs/address_map.md), [protocol.md](docs/protocol.md) |
| `Serial_System_Bus.qsf/.qpf/.sdc` | Quartus project, pin assignments and timing constraints |

## Simulate

```bash
cd Serial_System_Bus
./sim/run_icarus.sh              # all 13 testbenches; exit 0 only if all pass
./sim/run_icarus.sh arbiter      # just one
```

Under ModelSim/Questa: `vsim -c -do sim/run_questa.do` from this directory.

The testbenches always exit 0 themselves — **grep the output**, or use the
script, which does it for you.

## Build

Run on a **copy** in a scratch directory; compiling in place rewrites the
`db/` and `output_files/` blobs.

```bash
quartus_map Serial_System_Bus --part=EP4CE115F29C7
quartus_fit Serial_System_Bus
quartus_sta Serial_System_Bus
quartus_asm Serial_System_Bus
```

Result before the board layer was removed: 0 errors, **0 inferred latches,
0 combinational loops**, 81,920 memory bits in M9K, and timing closed against
the 50 MHz requirement — 1,275 LEs and 937 registers at 157.3 MHz with the
ISSP instance in the bitstream, of which 688 LEs and 513 registers were the
bus itself.

Deleting the scenario sequencers, the debounce and display logic removes
several hundred of those logic elements. **The figures above have not been
re-measured since**; the memory bits are unchanged, and `Total memory bits`
in the fit report must still read **81,920**.

Measured transaction cost (simulation): **write 21 clocks, read 30, split
read 79** — the split figure is measured with the other master contending
(`tb_integration` test 4 runs three transfers in the gap the split opens);
uncontended it is 57 at `SPLIT_LATENCY = 6`. Sixteen of the write's and the
read's clocks are the address going out one bit at a time — the price of a
one-wire bus, and the reason the split transaction earns its keep here.

## Board bring-up

Twelve pins used to leave the device; ten do now. There are no switches, no
scenario select and no seven-segment displays — the design is driven entirely
over JTAG, so the only board I/O is the clock, the reset button and eight
LEDs.

| Port | Pin | DE2-115 net | Shows |
|---|---|---|---|
| `CLOCK_50` | `Y2` | `CLOCK_50` (dedicated clock) | 50 MHz, the one clock domain |
| `rst_n` | `M23` | `KEY[0]`, active low with pull-up | reset |
| `led[7:0]` | `G19 F19 E19 F21 F18 E18 J19 H19` | `LEDR[7:0]` | master 0's last **read** data |
| `ext_rx_serial` | `AB22` | `GPIO[0]`, JP5 | UART **in** from the other board |
| `ext_tx_serial` | `AC15` | `GPIO[1]`, JP5 | UART **out** to the other board |

`master` captures `rdata` on reads only, so a write leaves the LEDs
unchanged and the value stays readable between reads. All 8 data bits reach
a pin — that is load-bearing, not decoration: the fitter deletes memory bits
that cannot reach an output, and an earlier revision of this design lost half
of each memory exactly that way.

`STRATIX_DEVICE_IO_STANDARD` is `3.3-V LVTTL` (Quartus otherwise defaults the
banks to 2.5 V) and `RESERVE_ALL_UNUSED_PINS` is `AS INPUT TRI-STATED` — the
DE2-115 hangs SDRAM, SRAM, flash, ethernet, audio, VGA and HSMC off pins this
design does not use, and the default of "as output driving ground" would
drive into them.

Everything else — issuing transactions, choosing the master, enabling slave
0's split, reading back data, latency and the bus-side probes — happens over
JTAG. See **Debugging on the board over JTAG** below.

## The board-to-board link

Master 0 is a `master_uart`: the ordinary master core with a UART **client**
and **server** wrapped around it. **The address selects the board** — there
is no remote-mode bit:

```
addr[15] == 0    local   - goes out on this board's serial bus
addr[15] == 1    remote  - carried over the UART and run on the OTHER board

far address = local address - 0x8000        0x9ABC here == 0x1ABC there
```

| Type here | Reaches the far board's | Slave |
|---|---|---|
| `0x8000`–`0x87FF` | `0x0000`–`0x07FF` | slave 0, 2 KB, device id 0 |
| `0x9000`–`0x9FFF` | `0x1000`–`0x1FFF` | slave 1, 4 KB, device id 1 |
| `0xA000`–`0xAFFF` | `0x2000`–`0x2FFF` | slave 2, 4 KB, device id 2 |

The block is symmetric — it is also the **server** for the other board's
requests, which it runs through the same master core. An incoming request is
executed on the local bus like any other transaction: it goes through the
normal arbiter and decoder, so nothing on the receiving side knows it came
from a UART.

### Wire format

8N1, 115200 baud (`CLKS_PER_BIT = 434` at 50 MHz), little-endian, tagged:

```
REQUEST  (4 bytes)          RESPONSE (2 bytes, READS ONLY)
  0xA5                        0x5A
  cmd[7:0]                    data byte
  cmd[15:8]
  cmd[23:16]

 bit  23 ........ 16 | 15  14 | 13 .......... 2 |  1  |  0
     +---------------+--------+-----------------+-----+-----+
     |    wdata[7:0] |  dev   |   offset[11:0]  | we  |  0  |
     +---------------+--------+-----------------+-----+-----+
                 00=s0 01=s1 10=s2         1=write   reserved
```

`dev` and `offset` together are the far address's low 14 bits, so the command
is just `{wdata, addr[13:0], we, 1'b0}` going out and `{2'b00, cmd[15:2]}`
coming in. The receiving side's top two address bits are always `00`, which
makes 14 bits lossless — and means **a request can never decode back into the
receiver's own remote window**. The link is loop-free by construction; there
is no hop count.

### Rules that are load-bearing

| Rule | Why |
|---|---|
| **Writes are posted** — no response at all | so a `0x5A` frame is never ambiguous: exactly one arrives per read. The sender retires as soon as the request is on the wire |
| **Responses beat requests** in the shared transmitter | if both boards fire at once and requests won, both would wait for an answer neither is sending. `tb_uart_remote` test 7 fires both directions on the same clock |
| **Tag hunting** — hunt a tag, then take exactly 3 more bytes (request) or 1 (response) | a payload byte may itself be `0xA5` or `0x5A`; re-scanning it desynchronises the stream |
| **10 ms timeout → `0xFF` + `cmd_error`** | an unplugged cable reports a failure instead of hanging. Same discipline as the default slave |
| **The local path is a pure pass-through** | a local write still costs 21 clocks and a local read 30, asserted in `tb_uart_remote` test 1 |

Because writes are posted, a remote write **retires before the far board has
executed it** — read it back only after allowing the link time to deliver.

### Cabling

Each board's `rm_tx` to the other's `rm_rx`, plus a **common ground**. On this
board `rm_tx` is `AC15` and `rm_rx` is `AB22` (GPIO header JP5). The spec
quotes `PIN_D3`/`PIN_C3`, but those are the *other* board's pins and do not
exist on the EP4CE115F29C7 — only the baud rate and frame format have to
match between boards.

### Debugging the link

A serial link between two boards fails silently: "the remote read timed out"
is equally true of a missing ground, a baud mismatch, an unprogrammed far
board and a protocol disagreement. So the probe carries counters that tell
those apart:

```bash
quartus_stp -t tcl/issp_link_test.tcl             # diagnose the link
quartus_stp -t tcl/issp_link_test.tcl -loopback   # jumper AC15 to AB22
```

Once the link answers at all, `issp_remote_rw_test.tcl` verifies that what it
carries is *correct*. It is self-verifying — every check writes a value across
the link and reads that same value back, so nothing has to be seeded on the
far side and nothing has to be agreed in advance:

```bash
quartus_stp -t tcl/issp_remote_rw_test.tcl            # write/read-back
quartus_stp -t tcl/issp_remote_rw_test.tcl -loopback  # same, one board
quartus_stp -t tcl/issp_remote_rw_test.tcl -full      # all 256 byte values
```

It is the hardware counterpart of `tb_uart_remote` tests 12–17 and looks for
the same faults: walking bits, every carried address bit, the tag bytes
`0xA5`/`0x5A` as data, adjacent words, and `0x00` as a real answer. When
something fails it diagnoses the *pattern* rather than reporting one byte —
bit-reversed, a stuck bit, one fixed location, or an aliased address each
produce a different verdict. **In two-board mode it writes into the far
board's memory**, in the scratch window `0x1F00–0x1F1F`, `0x07F0`, `0x2F00`.

| What you see | What it means |
|---|---|
| bytes sent = 0 | our transmitter never ran — a fault on **this** board |
| sent > 0, received = 0 | cable not crossed, **no common ground**, far board unprogrammed, or baud wildly off |
| received > 0, no RESPONSE parsed | the wire is fine; the disagreement is protocol or baud |
| RESPONSE parsed | the link works — read the data |

`link_status` prints all of it (`status` in the console shows a summary), and
`link_diagnose` turns it into a verdict.

**Run `-loopback` first.** With `rm_tx` (AC15) jumpered to `rm_rx` (AB22) the
board answers its own remote requests out of its own memory — `0x9ABC` comes
back with whatever is at local `0x1ABC`. That exercises the entire path,
client through server, with no second board. If loopback fails, the problem is
on this board and no amount of cable-wiggling will help.
`tb_uart_remote` test 11 proves the same thing in simulation.

From the JTAG console, remote is just an address:

```
bus[M0]> wa 9ABC 5A     write 0x5A to the far board's 0x1ABC (posted)
bus[M0]> ra 9ABC        read it back from over there
bus[M0]> rr 1ABC        the same read, spelled far-side
```

Both boards must agree on: 115200 8N1, slave sizes 2K/4K/4K with ids 0/1/2,
the `0xA5`/`0x5A` tags and little-endian byte order, the 10 ms timeout, and
crossed cabling with a common ground. Running the same bitstream gives all
five for free.

## Phase 2

Remote access shipped as a **sideband** on master 0's command port rather
than as an address window — see *Remote access over UART* above, and §15 of
[design_notes.md](docs/design_notes.md) for why. So `addr[15] == 1`
(`0x8000`–`0xFFFF`) stays reserved and unmapped; `tb_addr_decoder`'s 64K
sweep still fails if anything is ever mapped there.

Still ready for more: the arbiter is parameterised for `N_MASTERS`
requesters, and the split mechanism is written once in `slave`'s
`SPLIT_CAPABLE` block, so a future bridge that wants to be a bus peripheral
rather than a master sideband reuses both. See §7 of
[design_notes.md](docs/design_notes.md).

## The report

[`docs/report.pdf`](docs/report.pdf) is the engineering report: requirements
and the decisions taken before any RTL, the parallel bus that came first and
why it was replaced, the serial architecture in detail, arbitration, split
transactions, hang-freedom, the module partitioning, verification, measured
results, and limitations. It concerns the **interconnect** — masters and
slaves appear only where they define the bus contract.

Rebuild it from source:

```bash
cd Serial_System_Bus/docs
latexmk -pdf report.tex        # or: pdflatex report.tex, twice
```

Needs TeX Live with `tikz`, `booktabs`, `listings` and `newtx`.

## Debugging on the board over JTAG

Every bitstream carries an In-System Sources & Probes instance, **`SBUS`**,
wired to both masters' command ports. **It is the only way to issue a
transaction** — `top_debug` has no other command source — so the bus sits
idle until a host claims the instance.

```bash
cd Serial_System_Bus
quartus_stp -t tcl/issp_console.tcl     # interactive
quartus_stp -t tcl/issp_bus_test.tcl    # scripted regression, exit 0 = pass
```

`quartus_stp` is the **only** interpreter that works — the JTAG and ISSP Tcl
packages are absent from `quartus_sh` and from the Quartus GUI's Tcl console.
**Close the In-System Sources & Probes Editor tab first**; an open editor
holds the JTAG session.

The driver drives the masters' **normal parallel command ports** — the
serialisation happens inside `master` — so a transaction issued over JTAG
takes exactly the path any transaction takes. This is the bus's front door,
not a back door onto the wires.

```
bus[M0]> w 1 ABC 5A          write 0x5A to slave 1 offset 0xABC
bus[M0]> r 1 ABC             read it back
bus[M0]> ra 2800             read an UNMAPPED address - answers ERROR
bus[M0]> split on            make slave 0 answer SPLIT
bus[M0,split]> r 0 A5C       watch the latency jump
bus[M0]> both 1200 AA 2200 55   both masters on the same clock edge
bus[M0]> status              dump the bus-side probes
```

Three probes exist only because this bus is serial, and they are the ones
worth looking at first:

| Probe | Says |
|---|---|
| `bus_addr` | the address the bus **reassembled off the single wire** — compare it with what you sent |
| `frame_len` | how many clocks the last address frame lasted; **must read 16** |
| `frame_bad` | sticky: some frame was not 16 clocks, so the serial framing is broken on silicon |

For the UART bridge there are three more: `cmd_error` (`prb[29]`, sticky — a
remote transaction timed out), `remote_busy` (`prb[92]`) and `srv_busy`
(`prb[93]`, this board is serving the other one). `status` prints all three.

The bit map lives in three places that must agree: the header of
[`rtl/bus_issp_driver.v`](rtl/bus_issp_driver.v), that file's probe assembly,
and [`tcl/issp_bus_lib.tcl`](tcl/issp_bus_lib.tcl).

`top_debug` must be the top-level entity — the ISSP driver is instantiated
there, and with anything else as top there is no ISSP in the bitstream at
all. Quartus rewrites that line when you set a file as top-level in the GUI,
so check it after opening the project.
