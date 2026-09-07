# Serial_System_Bus — 2-master / 3-slave shared bus with split transactions

Synthesisable shared-bus interconnect for the **Terasic DE2-115**
(`EP4CE115F29C7`, Cyclone IV E), Quartus Prime Lite 24.1std, Verilog-2001.

16-bit word address, 32-bit data, two masters, three memory slaves plus a
default slave, fixed-priority arbitration with bus lock, and AHB-style split
transactions.

```
de2_top                          synthesis top (DE2-115 wrapper)
 +- reset_ctrl                   debounced KEY[0] -> async-assert/sync-release rst_n
 +- debouncer                    KEY[1] single-step
 +- master_prog x2               on-board scenario sequencers (SW[1:0])
 +- bus_top                      the bus itself, board-independent
 |   +- master x2                transaction FSM, replays after SPLIT
 |   +- arbiter                  priority + lock + split mask, parameterised for N
 |   +- addr_decoder             combinational, one-hot + default select
 |   +- bus_mux                  fwd mux by grant, return mux by REGISTERED select
 |   +- slave_mem #0             0x0000-0x0FFF  4K words, split capable
 |   +- slave_mem #1             0x1000-0x1FFF  4K words
 |   +- slave_mem #2             0x2000-0x27FF  2K words
 |   +- default_slave            everything else -> ERROR, so the bus never hangs
 +- seg7_hex x8                  displays
```

## Layout

| Path | Contents |
|---|---|
| `rtl/` | synthesisable modules, one per file, plus `bus_defs.vh` |
| `tb/` | one self-checking testbench per module, plus `tb_bus_top` and `tb_de2_top` |
| `sim/` | `run_icarus.sh`, `run_questa.do` |
| `docs/` | [design_notes.md](docs/design_notes.md), [address_map.md](docs/address_map.md), [protocol.md](docs/protocol.md) |
| `Serial_System_Bus.qsf/.qpf/.sdc` | Quartus project, pin assignments and timing constraints |

## Simulate

```bash
cd Serial_System_Bus
./sim/run_icarus.sh              # all 8 testbenches; exit 0 only if all pass
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

Current result: 0 errors, **0 inferred latches, 0 combinational loops**,
721 LEs, 544 registers, 327,680 memory bits in M9K, **Fmax 117.74 MHz**
against a 50 MHz requirement.

## Board controls

| Control | Function |
|---|---|
| `KEY[0]` | reset (active low, debounced) |
| `KEY[1]` | single step — run exactly one more bus transaction |
| `SW[1:0]` | scenario: `00` one master, `01` two masters, `10` split, `11` unmapped recovery |
| `SW[13]` | display half: `0` read data `[15:0]`, `1` `[31:16]` |
| `SW[14]` | display master: `0` = master 0, `1` = master 1 |
| `SW[15]` | `0` free run at 50 MHz, `1` slow (one transaction per ~0.17 s tick) |
| `SW[16]` | slave 0 split enable — the "slave is busy" model |
| `SW[17]` | run. `0` = stopped, use `KEY[1]` to step |

| Display | Shows |
|---|---|
| `HEX7..HEX4` | address of the selected master's last command |
| `HEX3..HEX0` | 16 bits of its last **read** data, half chosen by `SW[13]` |
| `LEDR[1:0]` | arbiter grant `{m1, m0}` |
| `LEDR[3:2]` | **split mask** `{m1, m0}` — lit while a master is deferred |
| `LEDR[7:4]` | responding slave `{default, s2, s1, s0}` |
| `LEDR[9:8]` | last response: `00` OKAY, `01` ERROR, `10` SPLIT |
| `LEDR[10]` | slave 0 busy with a deferred transfer |
| `LEDR[11]` | a master owns the bus |
| `LEDR[13:12]` | sticky ERROR seen by `{m1, m0}` |
| `LEDR[15:14]` | master busy `{m1, m0}` |
| `LEDR[16]` | slave 0 split enable |
| `LEDR[17]` | run enable |
| `LEDG[0]` / `LEDG[1]` | master 0 / master 1 completed a transaction (stretched) |
| `LEDG[2]` | slave 0 busy |
| `LEDG[3]` | bus granted |
| `LEDG[7:4]` | master 0 split count, low nibble |
| `LEDG[8]` | any master has seen an ERROR since reset |

### Demo to run on the board

1. `SW = 0`, press `KEY[0]`. Everything dark, nothing on the bus.
2. `SW[1:0] = 01`, `SW[15] = 1` (slow), `SW[17] = 1`. Both grant LEDs
   alternate as the two masters take turns.
3. `SW[1:0] = 10`, `SW[16] = 1`. `LEDR[2]` (master 0's split mask) lights for
   ~0.2 s at a time while `LEDG[1]` keeps flashing — master 1 is still
   completing transactions during master 0's stall. `LEDG[7:4]` counts the
   splits.
4. `SW[1:0] = 11`. `LEDG[8]` lights on the first unmapped access and the
   design **keeps running** — that is the recovery, not a hang.

## Phase 2

`addr[15] == 1` (`0x8000`–`0xFFFF`) is reserved for the remote window and is
deliberately unmapped. The arbiter is parameterised for `N_MASTERS`
requesters, and the split mechanism is written once in `slave_mem`'s
`SPLIT_CAPABLE` block, so the bridge reuses both. See §7 of
[design_notes.md](docs/design_notes.md).
