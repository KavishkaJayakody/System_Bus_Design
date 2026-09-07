# Design notes

Running record of every decision and why. Newest sections at the bottom.

---

## 1. Decisions the brief left open

The brief did not pin these down. Each was chosen deliberately; change any of
them and this file is the place to record it.

| # | Question | Decision | Reasoning |
|---|---|---|---|
| 1 | Board | **DE2-115**, device `EP4CE115F29C7` | The brief said DE0, but the Quartus project in this folder targets `EP4CE115F29C7`. The device string was read out of `Serial_System_Bus.qsf`, not guessed, as the brief required. Pin names were cross-checked against the DE2-115 assignments already used elsewhere in this repo. |
| 2 | HDL | **Verilog-2001** throughout | Matches the rest of the repo, and it lets the whole regression run under Icarus Verilog, which does not handle SystemVerilog well. No file mixes the two. |
| 3 | Data width | **8 bits**, parameterised | On a serial bus the data width is a direct latency cost — one clock per bit, on every transfer. 8 bits also makes the slaves 4 KB / 4 KB / 2 KB, which is the natural reading of "slave memory sizes 4K, 4K, 2K". Superseded the original 32-bit choice when the bus went serial; see §8. |
| 3b | Bus signalling | **serial**: one wire for the address, one for the data, control parallel | The project is called `Serial_System_Bus`. The address and data buses are serialised onto one wire each; `valid`/`we`/`ready`/`resp`/`master_id` stay parallel, which keeps the arbiter and decoder unchanged. Shared bus = 8 wires. See §8. |
| 4 | Response encoding | `OKAY=2'b00`, `ERROR=2'b01`, `SPLIT=2'b10` | AHB-flavoured. `OKAY` is all-zero so a reset or an idle bus reads as "nothing wrong". |
| 5 | Handshake polarity | everything active high except `rst_n` and the board's `_n` pins | One rule, no exceptions to remember. |
| 6 | Memory initial contents | **none** | Initialising would need an `initial` block or a MIF, and the brief forbids `initial` in synthesisable code. Every test writes a location before reading it instead. The arrays are also not reset — see §3. |
| 7 | What makes slave 0 "busy" | an explicit `split_en` input | The brief says "slave 0 busy → SPLIT" without saying what busy means. Modelling it as an input makes the split deterministic and testable, and puts it on `SW[16]` for the demo. |
| 8 | Board clocking | **no divided clock** | The brief asked for a PLL or clock divider, but a divided clock is a derived/gated clock and would break the brief's own "one clock domain, no gated clocks" rule. The bus runs at 50 MHz and the *scenario sequencer* is throttled with a slow clock-enable tick. Same visible effect, one clock, fully timing-analysable. |
| 9 | Repo layout | `.qpf`/`.qsf`/`.sdc` stay at the `Serial_System_Bus/` root | The brief's layout puts project files in `quartus/`, but the Quartus project already exists here and moving it would break it. `rtl/`, `tb/`, `sim/`, `docs/` follow the brief. |

---

## 2. Protocol shape

A transfer is an **address frame of `ADDR_W` clocks** followed by a reply.
`bus_valid` marks the frame; the address goes out MSB-first on `bus_astream`
and, on a write, the data goes out right-aligned on `bus_dstream` at the same
time. Selection and the reply come afterwards.

* **Write data is right-aligned in the frame.** This is the choice the whole
  design leans on: every receiver becomes a plain shift register of its own
  width holding "the last W bits I saw", which is exactly the field it wanted.
  No receiver needs a bit counter, and one frame timer in the master serves
  the entire bus.

* **Selection happens after the frame.** The decoder cannot decide anything
  until the last address bit lands, so every slave shifts every frame in
  regardless of whether it is addressed, and acts only on `sel`.

* **The return select is latched, not delayed one cycle.** A write answers in
  1 cycle, a split in 1, a read in 10 — the reply is no longer at a fixed
  offset from the address.

* **Read data is "the last `DATA_W` bits before `bus_ready`".** That contract
  costs no extra wire.

* **The data wire needs no turnaround.** On a write only the master sends,
  during the frame; on a read only the slave sends, after it. The direction
  control is just `bus_we`.

* **The deferred master keeps `bus_req` high.** The arbiter's mask, not
  request withdrawal, removes it from arbitration.

* **`ERROR` is reported, never retried.** An unmapped address will not become
  mapped; a retry would be an infinite loop holding the bus.

Full detail, timing diagrams and the wire table in [protocol.md](protocol.md).

---

## 3. Deliberate exceptions to the coding rules

The brief calls its RTL rules non-negotiable. Three places break one, each for
a concrete reason.

### The memory arrays have no reset

`slave_mem.v` puts `mem[]` and `rdata` in a clock-only `always` block.

Giving a 4096-word array an asynchronous reset stops Quartus inferring M9K
block RAM and makes it build the memory out of LUTs and flip-flops instead —
4096 × 32 registers per slave, which does not fit and would not close timing.
The same mistake is documented in this repo's other design, where a reset loop
over all 4096 words forced exactly that.

Consequence: the arrays hold no defined value at power-up. Every test writes
before it reads.

### `reset_ctrl` has no reset

It *is* the reset generator, so there is nothing to reset it with. It relies
on Cyclone IV registers powering up cleared, which gives a free power-on reset
— `rst_n` is low at configuration and rises once `KEY[0]` has been steady for
the debounce interval. In simulation its registers start as X and resolve
within two clocks.

### `de2_top` uses a tick enable, not a clock divider

See decision 8 above.

---

## 4. Things that were caught and fixed

Worth not reintroducing.

**Static tasks called from a `fork`.** `tb_bus_top` drives both masters
concurrently. Verilog tasks have *static* storage by default, so the two
concurrent calls to `m_run` shared `n`, `a` and `d` and corrupted each other —
three tests failed for a reason that had nothing to do with the RTL. Fixed
with `task automatic`. Any task called from more than one process at a time
must be `automatic`.

**`// synthesis` inside a comment.** A comment reading `// synthesis in each
instance.` was parsed by Quartus as a synthesis pragma and produced three
"unrecognized synthesis attribute" warnings. The word `synthesis` must not
start the text of a comment line.

**A busy counter too narrow for its own parameter.** `slave_mem`'s counter was
16 bits while `de2_top` passes `SPLIT_LATENCY = 10,000,000`. The constant was
silently truncated to 38,528 — the split would have been ~260× shorter than
intended and nobody would have noticed on the board. The counter is now 32
bits so any integer parameter fits, and the truncation warning is gone.

**Read-during-write forcing dual-clock RAM.** The memory originally did
`if (serve) rdata <= mem[addr];`, so a write also read the same location in
the same cycle. Quartus then infers a RAM whose read-during-write result it
explicitly documents as *undefined* — it would not have matched simulation.
The read is now gated with `!we`. Nothing is lost: a write must not disturb
`rdata` anyway, which is the same rule `master.v` follows when it captures
read data.

**The fitter merged two byte lanes and built 24-bit memories.** The demo write
pattern put a constant tag (`0xA0A0` / `0xB1B1`) in the high half, so bits
`[31:24]` and `[23:16]` were always identical. Quartus spotted it and built
the memories 24 bits wide instead of 32 — a third of the datapath quietly did
not exist. The pattern now XORs the tag with the transaction count so all 32
bits vary independently, and the memories are the full 327,680 bits.

**Half the read data was unobservable.** Only `rdata[15:0]` reached the
seven-segment displays, so the fitter trimmed the memories to 16 bits. Both
this and the byte-lane merge above are the same lesson, and it still governs
the 8-bit design: **all 8 data bits reach `HEX1..HEX0`, and the demo write
pattern varies every one of them.** A datapath the fitter cannot see reaching
an output is a datapath it will not build.

---

## 5. Verification status

Ten self-checking testbenches, all passing. Each prints its own PASS/FAIL
banner and an error count.

```
cd Serial_System_Bus
./sim/run_icarus.sh            # all of them; exit 0 only if everything passed
./sim/run_icarus.sh arbiter    # just one
```

The testbenches always `exit 0` themselves, so the script greps their output
and sets the exit status. Do not trust `vvp`'s exit code.

| Testbench | What it proves |
|---|---|
| `tb_addr_decoder` | reset/idle, one-hot per range, first *and* last address of every range, one past each range, the `0x2800` hole, the reserved `addr[15]=1` window, and an exhaustive 64K sweep checking one-hot-ness for every address |
| `tb_arbiter` | no grant out of reset; single requester granted and **held** until completion; both requesting → master 0 wins and master 1 waits; split masks the granted master, it is excluded from arbitration *while still requesting*, the other master runs meanwhile, `split_complete` restores it; the low-priority master can split too; `ERROR` does not set the mask |
| `tb_shift_ser` | reset; MSB-first order; **zero padding past the end of the word**, which is what right-aligns short fields in a long frame; load beating a simultaneous shift; reload for a replay |
| `tb_shift_deser` | reset; MSB-first order; the **"last W bits"** property, checked by feeding one 16-bit frame into 8-, 12- and 16-bit receivers at once and confirming each keeps the field it needs |
| `tb_bus_mux` | reset; the forward mux follows the grant and ignores the other master entirely; the data wire's direction is exactly `bus_we` and a slave cannot disturb a write; the return select is **latched and held for 12 cycles**, so a read reply arriving 10 cycles late still finds the right slave |
| `tb_slave_mem` | both `SPLIT_CAPABLE` builds: reset; write/read-back through the serial path; **response timing** — a write at S+1, a read at S+10; only the **low** address bits reach a slave (`0x2123` and `0xF923` must share offset `0x123` on the 2K slave); **a frame with no select must do nothing** — no ready, no memory change; split read with SPLIT arriving at S+1 while the read it defers costs 10; split of a write not taking effect until the replay; one split outstanding; the data wire left idle |
| `tb_default_slave` | reset, `ERROR` one cycle after `sel`, `rdata = 0`, quiet while idle, back-to-back bad addresses |
| `tb_master` | reset; the testbench **deserialises what the master puts on the wires**, so the checks are on the traffic itself: the frame is exactly `ADDR_W` clocks, the address arrives MSB-first, the write data arrives right-aligned; `0x80` and `0x01` both round-trip (catches a bit-order slip); the master does **not** drive the data wire during a read; `ERROR` reported and not retried; `SPLIT` → `bus_req` held, **no frame at all** while masked, then a complete second frame with the identical address *and data*, and exactly **one** `done` |
| `tb_bus_top` | all five things the brief lists, end to end — see below |
| `tb_de2_top` | the real board top level driven through its pins: KEY[0] reset, all four scenarios, the mask LED lighting while master 1 keeps completing transactions, the sticky error LED, single-stepping with KEY[1], and the displays |

`tb_bus_top` in particular covers:

1. reset — no grant, no request, clean mask, bus idle
2. one master — write/read to all three slaves, first and last address of each,
   and a check that the three memories are genuinely independent
3. two masters requesting on the **same clock** — master 0 wins, master 1 still
   completes, neither corrupts the other's data
4. the full split scenario — master 0 deferred, master 1 doing real work in the
   gap, the re-issued transfer returning the right data, mask up and back down;
   plus a split **write** landing exactly once
5. unmapped access — the hole, the reserved window and the top of memory all
   answer `ERROR`, and the bus **recovers**: the next transfer succeeds, and
   four consecutive bad addresses do not wedge anything
6. the low-priority master splitting too
7. serial framing — every frame on the wire is exactly `ADDR_W` clocks, the
   reassembled address matches what was sent, and a split puts **two** full
   frames on the wire rather than one and a resumption

Every wait in `tb_bus_top` has a cycle budget and reports a timeout as a
FAILURE. That is what makes test 5 meaningful — a hang is detected, not just
endured.

---

## 6. Synthesis status

Quartus Prime Lite 24.1std, `EP4CE115F29C7`, run on a scratch copy so the
tracked tree is not flooded with `db/` and `output_files/` churn.

```
quartus_map Serial_System_Bus --part=EP4CE115F29C7
quartus_fit Serial_System_Bus
quartus_sta Serial_System_Bus
quartus_asm Serial_System_Bus
```

| | |
|---|---|
| Errors | 0 |
| **Inferred latches** | **0** |
| **Combinational loops** | **0** |
| Logic elements | 688 / 114,480 (< 1 %) |
| Registers | 513 |
| Memory bits | 81,920 / 3,981,312 (2 %) — 4 KB + 4 KB + 2 KB, all in M9K at the full 8-bit width |
| Pins | 106 / 529 |
| **Fmax** | **141.8 MHz** (slow 1200 mV 85 °C) against a 50 MHz requirement |

§9 has the same figures next to the parallel design's, for comparison.

Timing is constrained by `Serial_System_Bus.sdc`: `create_clock` on
`CLOCK_50` at 20 ns, `derive_clock_uncertainty`, and false paths on the
switches, buttons, LEDs and displays — all of which are driven or read by a
human and are synchronised inside the design.

### Remaining warnings, and why each is expected

| Warning | Explanation |
|---|---|
| `276027` inferred dual-clock RAM, ×3 | The read and write enables differ (`mem_read` vs `mem_write`), so Quartus infers a simple dual-port RAM and warns generically that read-during-write is undefined. Read-during-write **cannot occur here** — the two enables are mutually exclusive by construction. The fitter report confirms the result is Single Clock. |
| `21074` 14 input pins do not drive logic | `KEY[3:2]` and `SW[13:2]` are brought out to their board pins but unused. Expected. |
| `15714` / I/O assignment warnings | "Missing drive strength" on the LED and HEX pins. Quartus uses the default drive for 3.3-V LVTTL, which is what the board wants. |
| `169177` 3.3 V interface requirements | Standard Cyclone IV advisory (AN 447) for any 3.3-V LVTTL design on this device. |
| `292013` LogicLock licence | Lite edition notice, unrelated to this design. |

---

## 7. Phase 2 readiness

The brief asks that two things be ready for the remote-bridge phase.

* **The arbiter is parameterised for N requesters**, not hard-coded for two.
  `N_MASTERS` and `ID_W` are parameters, the priority encoder is a loop, and
  the mask is a per-bit vector. Adding the bridge as requester 2 means
  `N_MASTERS = 3`, `ID_W = 2` and wiring it up. `tb_arbiter` already exercises
  splits on both a high- and a low-priority master, so the third requester
  inherits tested behaviour.

* **The split mechanism is written once.** All of it lives in `slave_mem`'s
  `SPLIT_CAPABLE` generate block plus the arbiter mask. Any future slave —
  including the remote bridge, which will be slow for the same reason — gets
  split support by setting `SPLIT_CAPABLE = 1`.

* **`addr[15] == 1` is reserved and unmapped.** `tb_addr_decoder`'s 64K sweep
  fails if anything is ever mapped there.

---

## 8. The bus was converted from parallel to serial

The original build was a parallel muxed bus: 16 address wires and 32 data
wires, with the "sharing" done by a mux selected by the grant. That is what
`Serial_System_Bus` did **not** ask for. The address and the data now each
travel on **one wire**.

### What that cost in wires

| | Parallel | Serial |
|---|---|---|
| Shared bus (every master and slave taps these) | **86** | **8** |
| All interconnect nets, including per-master and per-slave stubs | **337** | **60** |

The 8 shared wires are `bus_astream`, `bus_dstream`, `bus_valid`, `bus_we`,
`bus_ready`, `bus_resp[1:0]` and `master_id`. The remaining 52 nets are the
per-master and per-slave stubs into the mux (4 each), arbitration
(`req`/`gnt`/`split_complete`), the decoder's 4 select lines, and the 16-bit
reassembled address inside `bus_top` that feeds the (still combinational)
decoder.

### What it cost in time

| | Parallel | Serial |
|---|---|---|
| Write | 5 clocks | **21** |
| Read | 5 clocks | **30** |
| Split read | 16 clocks | **79** |

Six times slower, and 16 of those clocks are the address going out one bit at
a time. That is the trade the brief's title asked for.

**It also makes the split transaction genuinely worth having.** On the
parallel bus a split freed the bus for a handful of clocks; here the slave
answers SPLIT in 1 clock and gets out of the way of a transfer that would
have taken 30, so the other master gets real work done in the gap.
`tb_bus_top` test 4 and `tb_de2_top` test 4 both check that master 1 completes
transactions *during* master 0's stall.

### The choice that made it cheap: right-aligned data

Write data is right-aligned inside the address frame — `ADDR_W-DATA_W` zeros,
then the data. Every receiver is then just a shift register of its own width,
clocked while `bus_valid` is high, holding **the last W bits it saw**, which
is exactly the field it wanted:

* a 12-bit slave register fed the whole 16-bit frame ends up holding
  `addr[11:0]`, its offset, with the upper bits discarded
* an 8-bit register fed the same frame ends up holding the write data

No receiver needs a bit counter. **One frame timer, in the master, serves the
whole bus.** `shift_ser` shifts zeros in behind the data, which is what makes
the padding automatic.

### What did NOT have to change

`arbiter.v` and `addr_decoder.v` are byte-for-byte the same as the parallel
design. The arbiter only ever looked at `req`, `bus_ready` and `bus_resp`,
none of which were serialised; the decoder is still purely combinational and
is simply enabled once per frame instead of once per access. The master
command interface is still parallel, so `tb_bus_top` and `tb_de2_top` kept
their structure — only the data values changed width.

### Things that had to be got right

**Selection happens after the frame.** The decoder cannot decide anything
until the last address bit lands, so every slave shifts every frame in
whether or not it is addressed. Acting on a frame without `sel` would corrupt
another slave's transfer; `tb_slave_mem` test 4 drives a full frame with no
select and checks that nothing answers and no memory changes.

**The return select had to become a latch.** On the parallel bus the reply
was always exactly one cycle after the address, so a single register worked.
Here a write answers in 1 cycle, a split in 1 and a read in 10 — the reply is
no longer at a fixed offset — so `bus_mux` captures the select on the
decoder's pulse and holds it until the next transfer.

**The read data phase starts at S+2, not S+1.** `mem_q` has to stay a plain
register loaded straight from the array so Quartus can absorb it as the M9K
output register. Merging it with the output shift register would force an
asynchronous array read and drop all three memories into LUTs. The extra
cycle buys block RAM.

**The data wire needs no turnaround.** On a write only the master sends,
during the frame; on a read only the slave sends, after it. The two can never
overlap, so the direction control is just `bus_we` — no bus-turnaround cycle,
no contention window.

### Bugs caught during the conversion

**A testbench address that was wrong for the slave's width.** `tb_slave_mem`
checked that `0x2123` and `0xFB23` hit the same word on the 2K slave. They do
not: an 11-bit offset makes them `0x123` and `0x323`. The RTL was right and
the test was wrong — corrected to `0xF923`, and the arithmetic is now written
out in the comment so the next reader can check it.

---

## 9. Verification and synthesis after the conversion

Ten self-checking testbenches, all passing (`./sim/run_icarus.sh`). Two are
new — `tb_shift_ser` and `tb_shift_deser` for the serial primitives — and the
rest were reworked for the new protocol.

The tests that matter most for a serial bus, and where they live:

| Property | Where |
|---|---|
| Every frame is exactly `ADDR_W` clocks, including a replayed one | `tb_bus_top` test 7, `tb_master` tests 2/6 |
| The address reassembled off the wire matches what was sent | `tb_bus_top` test 7, `tb_master` tests 2/4 |
| Write data arrives right-aligned | `tb_master` test 2 |
| Bit order is MSB-first (0x80 and 0x01 both round-trip) | `tb_master` test 3, `tb_shift_ser` test 2 |
| A slave ignores a frame it was not selected for | `tb_slave_mem` test 4 |
| Only the low address bits reach a slave | `tb_slave_mem` test 3 |
| The master does not drive the data wire during a read | `tb_master` test 3 |
| The data wire is idle when nobody is sending | `tb_slave_mem` test 8, `tb_bus_mux` test 3 |
| A split replays a COMPLETE frame, not a resumption | `tb_master` test 6, `tb_bus_top` test 7 |
| A read reply arriving 10 cycles late still finds the right slave | `tb_bus_mux` test 4 |

Synthesis, `EP4CE115F29C7`, Quartus Prime Lite 24.1std:

| | Parallel | Serial |
|---|---|---|
| Errors | 0 | 0 |
| Inferred latches | 0 | **0** |
| Combinational loops | 0 | **0** |
| Logic elements | 721 | **688** |
| Registers | 544 | **513** |
| Memory bits | 327,680 | **81,920** (4 KB + 4 KB + 2 KB, all M9K, full 8-bit width) |
| Fmax | 117.74 MHz | **141.8 MHz** |

Slightly smaller and faster despite the added shift registers: the 32-bit
muxes and the 32-bit return path were more logic than the serialisers cost.
Fmax rose because the widest combinational path — the 32-bit return mux — is
now one bit wide.

The remaining warnings are the same set as before and are explained in §6.
