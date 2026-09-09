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
| 3b | Bus signalling | **serial**: one wire for the address, one for the data, control parallel | The project is called `Serial_System_Bus`. The address and data buses are serialised onto one wire each; `valid`/`we`/`ready`/`resp`/`master_id` stay parallel, which keeps the arbiter and decoder unchanged. Shared bus = 9 wires (8 while there were only two masters). See §8. |
| 4 | Response encoding | `OKAY=2'b00`, `ERROR=2'b01`, `SPLIT=2'b10` | AHB-flavoured. `OKAY` is all-zero so a reset or an idle bus reads as "nothing wrong". |
| 5 | Handshake polarity | everything active high except `rst_n` and the board's `_n` pins | One rule, no exceptions to remember. |
| 6 | Memory initial contents | **none** | Initialising would need an `initial` block or a MIF, and the brief forbids `initial` in synthesisable code. Every test writes a location before reading it instead. The arrays are also not reset — see §3. |
| 7 | What makes the split slave "busy" | an explicit `split_en` input | The brief says "slave busy → SPLIT" without saying what busy means. Modelling it as an input makes the split deterministic and testable, and puts it on one ISSP source bit (`src[54]`) so a host can turn it on mid-session. |
| 8 | Board clocking | **no divided clock** | The brief asked for a PLL or clock divider, but a divided clock is a derived/gated clock and would break the brief's own "one clock domain, no gated clocks" rule. While the board demo existed, the bus ran at 50 MHz and the *scenario sequencer* was throttled with a slow clock-enable tick instead. The demo is gone (§14) and with it the last reason to slow anything down: there is one clock, undivided, and the JTAG host sets its own pace. |
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

`slave.v` puts `mem[]` and `mem_q` in a clock-only `always` block.

Giving a 4096-word array an asynchronous reset stops Quartus inferring M9K
block RAM and makes it build the memory out of LUTs and flip-flops instead —
at the current 8-bit width that is 32,768 registers for a 4 KB slave and
81,920 across the three, most of the device in flip-flops alone, and it would
not close timing. The same mistake is documented in this repo's other design,
where a reset loop over all 4096 words forced exactly that.

Consequence: the arrays hold no defined value at power-up. Every test writes
before it reads.

### `reset_ctrl` had no reset

*Historical: `reset_ctrl` went with the board layer in §14; `rst_n` is now a
pin.* It *was* the reset generator, so there was nothing to reset it with. It
relied on Cyclone IV registers powering up cleared, which gave a free
power-on reset. The rule it illustrates still stands for any future reset
generator.

### The board layer used a tick enable, not a clock divider

See decision 8 above. Also historical — there is nothing left to throttle.

---

## 4. Things that were caught and fixed

Worth not reintroducing.

**Static tasks called from a `fork`.** `tb_integration` drives both masters
concurrently. Verilog tasks have *static* storage by default, so the two
concurrent calls to `m_run` shared `n`, `a` and `d` and corrupted each other —
three tests failed for a reason that had nothing to do with the RTL. Fixed
with `task automatic`. Any task called from more than one process at a time
must be `automatic`.

**`// synthesis` inside a comment.** A comment reading `// synthesis in each
instance.` was parsed by Quartus as a synthesis pragma and produced three
"unrecognized synthesis attribute" warnings. The word `synthesis` must not
start the text of a comment line.

**A busy counter too narrow for its own parameter.** `slave`'s counter was
16 bits while the top level passes `SPLIT_LATENCY = 10,000,000`. The constant was
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

**The fitter merged two byte lanes and built 24-bit memories.** Found on the
32-bit parallel predecessor, so the widths below are its. The demo write
pattern put a constant tag (`0xA0A0` / `0xB1B1`) in the high half, so bits
`[31:24]` and `[23:16]` were always identical. Quartus spotted it and built
the memories 24 bits wide instead of 32 — a third of the datapath quietly did
not exist. The fix was to XOR the tag with the transaction count so every bit
varies independently, which restored the full 327,680 bits. The lesson still
governs the 8-bit design, and the memories are the full 81,920.

**Half the read data was unobservable.** Only `rdata[15:0]` reached the
seven-segment displays, so the fitter trimmed the memories to 16 bits. Both
this and the byte-lane merge above are the same lesson, and it still governs
the 8-bit design: **all 8 data bits reach `led[7:0]`, and `tb_top_debug`
checks that every one of them is seen both high and low at the pins.** A
datapath the fitter cannot see reaching an output is a datapath it will not
build.

---

## 5. Verification status

Thirteen self-checking testbenches, all passing. Each prints its own PASS/FAIL
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
| `tb_addr_decoder` | reset/idle, one-hot per range, first *and* last address of every range, one past each range, the `0x0800` hole, the reserved `addr[15]=1` window, and an exhaustive 64K sweep checking one-hot-ness for every address |
| `tb_arbiter` | no grant out of reset; single requester granted and **held** until completion; both requesting → master 0 wins and master 1 waits; split masks the granted master, it is excluded from arbitration *while still requesting*, the other master runs meanwhile, `split_complete` restores it; the low-priority master can split too; `ERROR` does not set the mask |
| `tb_shift_ser` | reset; MSB-first order; **zero padding past the end of the word**, which is what right-aligns short fields in a long frame; load beating a simultaneous shift; reload for a replay |
| `tb_shift_deser` | reset; MSB-first order; the **"last W bits"** property, checked by feeding one 16-bit frame into 8-, 12- and 16-bit receivers at once and confirming each keeps the field it needs |
| `tb_bus_mux` | reset; the forward mux follows the grant and ignores the other master entirely; the data wire's direction is exactly `bus_we` and a slave cannot disturb a write; the return select is **latched and held for 12 cycles**, so a read reply arriving 10 cycles late still finds the right slave |
| `tb_slave` | both `SPLIT_CAPABLE` builds: reset; write/read-back through the serial path; **response timing** — a write at S+1, a read at S+10; only the **low** address bits reach a slave (`0x2123` and `0xF923` must share offset `0x123` on the 2K slave); **a frame with no select must do nothing** — no ready, no memory change; split read with SPLIT arriving at S+1 while the read it defers costs 10; split of a write not taking effect until the replay; one split outstanding; the data wire left idle |
| `tb_default_slave` | reset, `ERROR` one cycle after `sel`, `rdata = 0`, quiet while idle, back-to-back bad addresses |
| `tb_system_bus` | **the bus with no master and no memory attached** — the testbench plays both roles on the serial interfaces: an address shifted in on a master's wire comes back reassembled on the right select; the data wire carries the granted master's bit on a write and the selected slave's bit on a read; the latched return select still routes a reply ten cycles late; and an unmapped frame is answered `ERROR` by the bus itself with nothing on the slave ports |
| `tb_master` | reset; the testbench **deserialises what the master puts on the wires**, so the checks are on the traffic itself: the frame is exactly `ADDR_W` clocks, the address arrives MSB-first, the write data arrives right-aligned; `0x80` and `0x01` both round-trip (catches a bit-order slip); the master does **not** drive the data wire during a read; `ERROR` reported and not retried; `SPLIT` → `bus_req` held, **no frame at all** while masked, then a complete second frame with the identical address *and data*, and exactly **one** `done` |
| `tb_integration` | all five things the brief lists, end to end — see below |
| `tb_uart_remote` | **two whole boards** on a crossed UART link: each board's own memory stays independent; a remote write from A lands in B's memory and nowhere else; a remote read brings it back; the reverse direction, proving one core serves both roles; a remote read of the far board's SPLIT slave; **both boards issuing on the same instant**, which is what the transmitter's response-priority rule exists for; an unplugged cable timing out with `cmd_error` instead of hanging; and local traffic still working with the link dead |
| `tb_top_debug` | the real synthesis top driven through its pins and its JTAG source register: reset, write/read back, `led[7:0]` following reads only and every bit moving, all three slaves, an unmapped address answering ERROR and the bus recovering, a host-enabled split, master 1, and `frame_len` reading 16 on the top level |
| `tb_bus_issp_driver` | the JTAG front-end against a real bus built in the testbench, driven through a hierarchical reference into the ISSP source register — the same sequence `issp_bus_lib.tcl` performs over JTAG: the level `cmd_valid` handshake, `go` as bit 0 and not payload, read/write round-trips on all three slaves, an unmapped address answering `ERROR`, a split, and the bus-side probes including `frame_len` and `frame_bad` |

`tb_integration` in particular covers:

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

Every wait in `tb_integration` has a cycle budget and reports a timeout as a
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
| Memory bits | 81,920 / 3,981,312 (2 %) — 2 KB + 4 KB + 4 KB, all in M9K at the full 8-bit width |
| Pins | 106 / 529 |
| **Fmax** | **141.8 MHz** (slow 1200 mV 85 °C) against a 50 MHz requirement |

> **Superseded.** The table above is the measurement taken at that point in
> the design's history and is kept as a record. For the CURRENT design — the
> bridge on the bus, three masters, four decoded targets, ISSP included — the
> measured figures are **1,651 LEs, 1,297 registers, 81,920 memory bits,
> 12 pins, Fmax 124.36 MHz**, with worst setup slack 11.959 ns, worst hold
> slack 0.360 ns, and **zero unconstrained paths**. See §Timing constraints.

These are the figures for the bus and the board layer alone. **Every
bitstream also carries the ISSP debug instance** (§11), which is
instantiated unconditionally in the top level, so what actually gets programmed
is 1,275 logic elements and 937 registers at 157.3 MHz. The memory bits and
the pin count are unchanged by it. §9 has the figures above next to the
parallel design's, for comparison.

Timing is constrained by `Serial_System_Bus.sdc`: `create_clock` on
`CLOCK_50` at 20 ns, `derive_clock_uncertainty`, and false paths on the
switches, buttons, LEDs and displays — all of which are driven or read by a
human and are synchronised inside the design.

### Remaining warnings, and why each is expected

| Warning | Explanation |
|---|---|
| `276027` inferred dual-clock RAM, ×3 | The read and write enables differ (`mem_read` vs `mem_write`), so Quartus infers a simple dual-port RAM and warns generically that read-during-write is undefined. Read-during-write **cannot occur here** — the two enables are mutually exclusive by construction. The fitter report confirms the result is Single Clock. |
| `21074` input pins do not drive logic | Was `KEY[3:2]` and `SW[13:2]` while the board layer existed. Those pins are gone (§14); the warning should be gone with them. |
| `15714` / I/O assignment warnings | "Missing drive strength" on the LED pins. Quartus uses the default drive for 3.3-V LVTTL, which is what the board wants. |
| `169177` 3.3 V interface requirements | Standard Cyclone IV advisory (AN 447) for any 3.3-V LVTTL design on this device. |
| `292013` LogicLock licence | Lite edition notice, unrelated to this design. |

---

## 7. Phase 2 readiness

*The remote link has since shipped, and §17 put it exactly here after all:
`addr[15] == 1` IS the remote window. It is reached through a sideband inside
`bus_bridge` rather than as a bus slave, so the two readiness claims below
still hold and the window is still unmapped as far as the decoder is
concerned. See §15 and §17.*

The brief asks that two things be ready for the remote-bridge phase.

* **The arbiter is parameterised for N requesters**, not hard-coded for two.
  `N_MASTERS` and `ID_W` are parameters, the priority encoder is a loop, and
  the mask is a per-bit vector. Adding the bridge as requester 2 means
  `N_MASTERS = 3`, `ID_W = 2` and wiring it up. `tb_arbiter` already exercises
  splits on both a high- and a low-priority master, so the third requester
  inherits tested behaviour.

* **The split mechanism is written once.** All of it lives in `slave`'s
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
reassembled address inside `system_bus` that feeds the (still combinational)
decoder.

### What it cost in time

| | Parallel | Serial |
|---|---|---|
| Write | 5 clocks | **21** |
| Read | 5 clocks | **30** |
| Split read (other master contending) | 16 clocks | **79** |

Six times slower, and 16 of those clocks are the address going out one bit at
a time. That is the trade the brief's title asked for.

**It also makes the split transaction genuinely worth having.** On the
parallel bus a split freed the bus for a handful of clocks; here the slave
answers SPLIT in 1 clock and gets out of the way of a transfer that would
have taken 30, so the other master gets real work done in the gap.
`tb_integration` test 4 checks that master 1 completes transactions *during*
master 0's stall.

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

`arbiter.v` and `addr_decoder.v` were byte-for-byte the same as the parallel
design at this point. The arbiter only ever looked at `req`, `bus_ready` and
`bus_resp`, none of which were serialised — and it still is unchanged. The
decoder was purely combinational, sitting behind a 16-bit deserialiser and
enabled once per frame instead of once per access; **§16 later made it serial
as well**, which removed that deserialiser from the datapath. The master
command interface is still parallel, so `tb_integration` kept its structure —
only the data values changed width.

### Things that had to be got right

**Selection happens after the frame.** The decoder cannot decide anything
until the last address bit lands, so every slave shifts every frame in
whether or not it is addressed. Acting on a frame without `sel` would corrupt
another slave's transfer; `tb_slave` test 4 drives a full frame with no
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

**A testbench address that was wrong for the slave's width.** `tb_slave`
checked that `0x2123` and `0xFB23` hit the same word on the 2K slave. They do
not: an 11-bit offset makes them `0x123` and `0x323`. The RTL was right and
the test was wrong — corrected to `0xF923`, and the arithmetic is now written
out in the comment so the next reader can check it.

---

## 9. Verification and synthesis after the conversion

Twelve self-checking testbenches, all passing (`./sim/run_icarus.sh`).

The tests that matter most for a serial bus, and where they live:

| Property | Where |
|---|---|
| Every frame is exactly `ADDR_W` clocks, including a replayed one | `tb_integration` test 7, `tb_master` tests 2/6 |
| The address reassembled off the wire matches what was sent | `tb_integration` test 7, `tb_master` tests 2/4 |
| Write data arrives right-aligned | `tb_master` test 2 |
| Bit order is MSB-first (0x80 and 0x01 both round-trip) | `tb_master` test 3, `tb_shift_ser` test 2 |
| A slave ignores a frame it was not selected for | `tb_slave` test 4 |
| Only the low address bits reach a slave | `tb_slave` test 3 |
| The master does not drive the data wire during a read | `tb_master` test 3 |
| The data wire is idle when nobody is sending | `tb_slave` test 8, `tb_bus_mux` test 3 |
| A split replays a COMPLETE frame, not a resumption | `tb_master` test 6, `tb_integration` test 7 |
| A read reply arriving 10 cycles late still finds the right slave | `tb_bus_mux` test 4 |

Synthesis, `EP4CE115F29C7`, Quartus Prime Lite 24.1std:

| | Parallel | Serial |
|---|---|---|
| Errors | 0 | 0 |
| Inferred latches | 0 | **0** |
| Combinational loops | 0 | **0** |
| Logic elements | 721 | **688** |
| Registers | 544 | **513** |
| Memory bits | 327,680 | **81,920** (2 KB + 4 KB + 4 KB, all M9K, full 8-bit width) |
| Fmax | 117.74 MHz | **141.8 MHz** |

Slightly smaller and faster despite the added shift registers: the 32-bit
muxes and the 32-bit return path were more logic than the serialisers cost.
Fmax rose because the widest combinational path — the 32-bit return mux — is
now one bit wide.

The remaining warnings are the same set as before and are explained in §6.

---

## 10. The design was split into three modules

The design used to be one file containing the masters, the slaves and all the
interconnect. It became three kinds of module, which at this point `de2_top`
instantiated side by side with no wrapper between them (§13 later moved the
composition into `bus_top`, leaving the split itself untouched):

| Module | Contains | Does NOT contain |
|---|---|---|
| `system_bus` | arbiter, address decoder, `bus_mux`, the central address deserialiser, the default responder | any master, any memory |
| `master` | command FSM, serialisers, deserialiser, replay | arbitration, decoding |
| `slave` | memory, deserialisers, output shift register, split state | arbitration, decoding |

**At the time, there was no integration wrapper.** `de2_top` instantiated
`master` x2, `system_bus` and `slave` x3 side by side, along with the one OR
gate that combines the split wake-ups per master.

That had a cost: the same composition appeared in three files —
`rtl/de2_top.v`, `tb/tb_integration.v` and `tb/tb_bus_issp_driver.v`. The two
testbenches could not instantiate a wrapper that did not exist, so they built
the system themselves, and changing the wiring in one meant changing all
three or the testbenches quietly stopped testing what the board builds.

**That trade was later reversed — see §13.** The three-kinds-of-module split
described in this section is unchanged and still the rule; only the question
of who holds the composition changed.

### Why the default responder lives inside the bus

It is the only judgement call in the split. It looks like a slave and is
named like one, but it holds none of the state a peripheral has — no memory,
no address of its own — and its entire job is to stop an unmapped address
leaving the bus without a responder. That is a property of the bus, not of
anything hanging off it. Putting it inside `system_bus` also means the
slave-side interface carries only real slaves, so `N_SLAVES` means what it
says.

### The interfaces are serial on both sides

| Interface | Address | Data |
|---|---|---|
| master → bus | `m_astream`, 1 wire per master | `m_dstream`, 1 wire per master |
| bus → slave | `bus_astream`, 1 shared wire | `bus_dstream`, 1 shared wire |
| slave → bus | — | `s_dstream`, 1 wire per slave |
| bus → master | — | `bus_dstream`, the same shared wire |

The per-endpoint `m_astream`, `m_dstream` and `s_dstream` inputs are
*candidates* — one wire from each endpoint into the mux, of which exactly one
reaches the shared wire. `bus_astream` and `bus_dstream` are single nets that
every endpoint taps; `bus_dstream` is one net exposed once and wired to both
masters and slaves, so the "one data wire" is literally one signal in the
netlist rather than a pair that happens to be connected.

There are no `inout` ports and no tristates. An FPGA has no internal tristate
buffers, so a shared wire is a mux — which is why each endpoint has an output
and an input onto the same net rather than one bidirectional port.

### What the split bought

**A testbench for the bus alone.** `tb_system_bus` instantiates no master and
no memory. It plays both roles by driving the two serial interfaces directly:
it shifts an address frame in on a master's wire and answers as a slave on a
slave's wire. That makes it possible to test things that were previously only
observable through a full system:

* the address shifted in on one wire comes back out reassembled and lands on
  the right select
* the data wire carries the granted master's bit on a write and the selected
  slave's bit on a read, and neither side can disturb the other
* the latched return select still routes a reply that arrives ten cycles after
  the frame
* **an unmapped frame is answered `ERROR` by the bus itself, with nothing
  attached to the slave ports at all** — the clearest possible statement that
  hang-freedom is the bus's property and not a slave's

**It cost nothing in hardware.** The build before and after the split is
identical: 688 logic elements, 513 registers, 81,920 memory bits, Fmax
141.8 MHz. It is a pure refactor, and the fitter agrees.

---

## 11. Debugging the bus on silicon

The board demo proves the bus runs, but it proves it with switches and LEDs:
you can see *that* something happened, not *what*. An In-System Sources &
Probes instance (`SBUS`, 56-bit source / 96-bit probe) is built into every
bitstream and wired to both masters' command ports, so any transaction can be
issued from a host and the result read back.

It drives the masters' **normal command ports**, which are parallel — the
serialisation happens inside `master`. So this is the bus's front door, not a
back door onto the wires, and a transaction issued over JTAG takes exactly the
path a transaction from the on-board sequencer takes.

### Three probes that only matter because the bus is serial

| Probe | Says |
|---|---|
| `bus_addr` | the address the bus **reassembled off the single wire** |
| `frame_len` | clocks the last address frame lasted — must read 16 |
| `frame_bad` | sticky: some frame was not `ADDR_W` clocks |

Frame length is the one thing the LEDs cannot show and simulation cannot
prove: whether the framing holds on real silicon at real temperature. If
`frame_bad` is ever set, nothing downstream can be trusted, and that is worth
knowing before chasing a data mismatch.

### Two things the older design's driver got wrong, avoided here

**The command handshake changed.** The parallel design's master took a
one-cycle `cmd_start` pulse. `master` takes a *level* `cmd_valid` and answers
`cmd_accept`, so the driver holds the request until it is taken. Pulsing it
would have worked most of the time and dropped commands when the master was
briefly busy.

**Bit 0 of each source slice is `go`, not payload.** The command fields are
assigned individually rather than as one wide concatenation over the whole
slice. A concat that includes bit 0 aliases `we` onto `go`, which makes reads
impossible — a bug the earlier driver's comments still warn about.

### It was simulated before it was programmed

`tb_bus_issp_driver` wires the driver to a real bus - master, `system_bus`
and slave, built in the testbench - and drives the ISSP
source register through a hierarchical reference, which is exactly what
`issp_bus_lib.tcl` does over JTAG. Its helper tasks mirror the Tcl library one
for one, so a sequence that passes in simulation is a sequence that works on
the board. `tb/altsource_probe_stub.v` stands in for the Altera megafunction,
which `iverilog` cannot elaborate; it is simulation-only and is not in the
`.qsf`.

### Cost

| | Without ISSP | With ISSP |
|---|---|---|
| Logic elements | 688 | 1,275 |
| Registers | 513 | 937 |
| $F_{max}$, `CLOCK_50` | 141.8 MHz | 157.3 MHz |
| Clock domains | 1 | 2 (`altera_reserved_tck` added by the JTAG hub) |

About 590 logic elements and 420 registers for the instance plus the JTAG
endpoint — under 1% of the device, and worth it for being able to interrogate
the bus without rebuilding. Quartus constrains the TCK domain itself; both
domains meet timing with positive slack (`CLOCK_50` +13.6 ns setup,
`altera_reserved_tck` +45.5 ns).

---

## 12. `de2_top` is instantiation only

The board top level had grown to 562 lines, and most of that was not the bus:
switch synchronisers, a tick counter, the run/step decision, the status
capture registers, the LED assembly and eight display instances. In the
Quartus RTL viewer the four blocks that matter — `master`, `system_bus`,
`slave`, `bus_issp_driver` — were lost among loose flops and gates.

That glue now lives in two blocks of its own:

| Module | Contains | Why it is one block |
|---|---|---|
| `board_ctrl` | `reset_ctrl`, the `KEY[1]` debouncer, the two-flop switch synchronisers, the slow tick, the run/step decision | everything between a human-operated input and a control signal, and nothing else — it generates no commands and touches no bus wire |
| `status_display` | the status capture registers (last address, last response, sticky error, the two done-pulse stretchers), the LED assembly, the display mux and `seg7_hex` x8 | **read-only with respect to the bus** — every input is an observation, so deleting it would change what you can see and nothing about what the bus does |

`de2_top` came down to 497 lines at this step (353 after §13) and contains
**no `always` blocks at all**. The
only loose logic left is the four-line mux that decides whether the on-board
sequencers or the JTAG host own the master command ports — which is kept in
plain sight deliberately, because it is the point at which the ISSP takes the
design over.

At this point the bus itself was still not wrapped: `master` x2,
`system_bus` and `slave` x3 stayed side by side in `de2_top`, so the board's
composition still matched what the two testbenches built by hand. **§13 then
replaced that with a single `bus_top` instance**, which removed the
duplication rather than merely keeping it visible.

Nothing else changed: the refactor moves lines between files and renames no
signal. One hierarchical reference in `tb_de2_top` followed the register it
points at, `dut.last_addr0` becoming `dut.u_display.last_addr0`. All twelve
testbenches pass unchanged otherwise.

---

## 13. The hierarchy now follows `System_Bus_Final`

§10 removed the integration wrapper and §12 boxed up the board glue. What was
left was a `de2_top` whose RTL view was clean but which still spelled out the
whole system — master x2, system_bus, slave x3 and every wire between them —
and two testbenches that each spelled out the same thing again.

The parallel `System_Bus_Final` design in this repo does not do that. Its
synthesis top is thin:

```
top_debug                 wires, two instances and one assign
 +- bus_issp_driver       the JTAG front-end
 +- top_bus_system        THE SYSTEM: masters + interconnect + slaves
```

The serial design now matches it, name for name in role:

| `System_Bus_Final` | `Serial_System_Bus` |
|---|---|
| `top_debug` — synthesis top, debug front-end + system | `de2_top` — the same, plus the DE2-115 board layer |
| `top_bus_system` — masters + `bus_interconnect` + slaves | `bus_top` — masters + `system_bus` + slaves |
| `bus_interconnect` — arbiter + decoder + muxes | `system_bus` — arbiter + decoder + `bus_mux` + deserialiser + default responder |

### What it changed

`rtl/bus_top.v` holds the composition — `master` x2, `system_bus`, `slave` x3
and the split-wake-up OR — and **contains no logic of its own**. It is
instantiation and wiring only, deliberately, so that it cannot become a place
where bus behaviour hides. Facing out it offers the two parallel command
ports and a set of observation outputs; everything from `gnt` down is read by
the ISSP driver, the display block and the testbenches, and none of it can
affect a transfer.

`de2_top`, `tb_integration` and `tb_bus_issp_driver` all instantiate it. The
composition is therefore written **once**, and the testbenches exercise what
the board builds by construction rather than by three copies being kept in
step by hand — which is what §10 flagged as the cost of the flat arrangement.

| File | Before | After |
|---|---|---|
| `rtl/de2_top.v` | 497 | **353** |
| `tb/tb_integration.v` | 621 | **460** |
| `tb/tb_bus_issp_driver.v` | 515 | **356** |
| `rtl/bus_top.v` | — | **330** |

`de2_top` is now six instances — `board_ctrl`, `master_prog` x2,
`bus_issp_driver`, `bus_top`, `status_display` — with no `always` blocks and
no loose logic beyond the four-line mux that decides whether the on-board
sequencers or the JTAG host own the command ports.

### What it did NOT change

**The three-kinds-of-module split of §10 stands.** `system_bus` is still the
bus and contains no master and no memory; `master` and `slave` are still
peripherals and contain no bus logic. `bus_top` is a fourth kind of thing — a
composition — not a fourth place for behaviour. `tb_system_bus` still
instantiates `system_bus` alone, with no master and no memory attached at
either end, and it is still the testbench that proves hang-freedom is the
bus's own property.

No signal was renamed and no logic moved. All twelve testbenches pass
unchanged apart from the substitution itself.

---

## 14. The board layer was removed; `top_debug` is the top

§13 made the hierarchy match `System_Bus_Final` from `bus_top` down. This
step finishes the job at the top: **`de2_top` is deleted**, and the synthesis
top is now `top_debug`, holding exactly what its namesake in the parallel
project holds.

```
top_debug                 wires, two instances and one assign
 +- bus_issp_driver       the JTAG front-end, IN FRONT of the system
 +- bus_top               THE SYSTEM: master x2 + system_bus + slave x3
```

### What went

The entire on-board demo: `de2_top`, `board_ctrl`, `status_display`,
`master_prog`, `seg7_hex`, `reset_ctrl`, `debouncer` and `tb_de2_top`, plus
the `SW`/`KEY[3:1]`/`LEDR[17:8]`/`LEDG`/`HEX` pin assignments and the false
paths that went with them. Ten pins leave the device now instead of
thirty-odd (twelve once §15 added the UART bridge):

| Port | Pin | DE2-115 net |
|---|---|---|
| `CLOCK_50` | `Y2` | the 50 MHz oscillator |
| `rst_n` | `M23` | `KEY[0]` |
| `led[7:0]` | `G19`…`H19` | `LEDR[7:0]`, master 0's last read data |

### Why that is not a loss of capability

Everything the switches selected, the JTAG host can already do better, and
could before this change: choose the master, choose read or write, choose any
address including the unmapped ones, turn the split slave on and off, and
read back not just the data but the response, the latency, the split count,
the grant, the split mask, the responder, and the reassembled address and
frame length. The demo could show *that* something happened; the console
shows *what*.

What is genuinely gone is the ability to demonstrate the bus with nothing
attached but a power supply. That was worth having, and it is the price of
this hierarchy.

### The one thing that had to be re-established

`de2_top` was the only module that drove the read data all the way to a pin,
and that is load-bearing: **the fitter deletes memory bits that cannot reach
an output** (§4). `top_debug` drives `led[7:0]` from master 0's `rdata` for
exactly that reason, and `tb_top_debug` checks that every one of the eight
bits is seen both high and low at the pins — so a future change that narrows
the display fails a test instead of silently narrowing the memories.

### Verification

Still twelve self-checking testbenches at this point: `tb_de2_top` is
replaced by `tb_top_debug`, which drives the real synthesis top through its pins and
through `dut.u_dbg.u_issp.source` — the same hierarchical path the Tcl
library writes over JTAG. It covers reset, write/read back, the LED
behaviour, all three slaves, an unmapped address answering ERROR with the bus
recovering, a host-enabled split, master 1, and `frame_len` reading 16 on the
top level itself.

Since `de2_top` no longer exists, `iverilog` elaborating "the real board top
level" now means `top_debug` — which is a smaller claim, but a truer one:
there is nothing between the pins and the bus except the debug driver.

### Not re-measured

The figures in §6 and §11 predate this deletion. Removing the sequencers,
the debounce logic and the display block takes several hundred logic elements
out, so the LE and register counts are now pessimistic; the memory bits are
unchanged and must still total 81,920.

---

## 15. Remote access over UART

> **Superseded by §17.** This section records the first cut, which used a
> `cmd_remote` sideband bit and a 26-bit command. The agreed board-to-board
> interface spec replaced both. The structure described here — core, client,
> server, one shared transmitter — is unchanged and still accurate.

The board can now reach a second board's memory. Master 0 is a `bus_bridge`
— the ordinary `master` core with a UART client and server wrapped around it
— and a transaction is either local or, with `cmd_remote` set, carried over
the link and executed on the far board's bus.

This is `master_node_uart` from `System_Bus_Final`, ported. `uart_tx.v` and
`uart_rx.v` are **copied unchanged**: 8N1 is 8N1, nothing about it depends on
whether the bus behind it is serial or parallel, and that code is already
hardware-verified board to board.

### Why a sideband bit and not the reserved window

§7 anticipated the remote link as a *bus bridge* at `addr[15] == 1`: a
split-capable slave on the near side, a master on the far side. That would
have suited this bus well — a UART round trip is roughly 17,500 clocks at
115200 baud, which is exactly the case the split mechanism exists for, and
`wa 8000 5A` would have needed no new command bit.

The sideband was chosen instead, to match `System_Bus_Final`. The
consequence is recorded plainly: **`0x8000`–`0xFFFF` stays reserved and
unmapped**, `tb_addr_decoder`'s 64K sweep still enforces it, and the phase-2
bridge remains available for a later revision that wants the remote window to
behave like memory. §7's two readiness claims are untouched — the arbiter is
still parameterised, and the split mechanism is still written once.

### What had to change from the parallel design

| | `System_Bus_Final` | here |
|---|---|---|
| Command on the wire | 24 bits, 3 payload bytes | **26 bits, 4 payload bytes** — 16-bit address |
| Core handshake | one-cycle `cmd_start` pulse | **level `cmd_valid` + `cmd_accept`** |
| Timeout counter | 24 bits | **32 bits** |

The 26-bit command is **exactly one ISSP source slice**, bit for bit. That is
worth keeping: the host, the debug driver and the wire all describe a command
the same way, so there is one layout to get right rather than three.

The level handshake is a genuine improvement, not just a difference. In the
parallel design the server had to be guarded against stealing the core from a
local `cmd_start` pulse that would then be lost. Here a local command that
arrives while the server is busy simply waits, held by its own `cmd_valid`,
and is accepted when the core frees up. Nothing can be dropped.

### The three things that are load-bearing

* **Responses take priority over requests** in the shared transmitter.
  Without it, two boards issuing a remote command on the same instant both
  sit in `C_WAIT` waiting for an answer neither is sending, and the link
  deadlocks until both time out. `tb_uart_remote` test 7 fires both
  directions on the same clock and checks that both complete.

* **`RESP_TIMEOUT` completes a remote transaction with `cmd_error`** instead
  of waiting forever, so an unplugged cable reports a failure. This is the
  same discipline as the default slave: *the bus must not be able to hang.*
  Test 8 cuts the link and checks the master finishes; test 9 checks the
  local bus is unharmed.

* **The local path is a pure pass-through.** `cmd_accept`, `done`, `rdata`
  and `resp` come combinationally from the core for a local transaction, so
  wrapping the master costs it nothing. `tb_uart_remote` test 1 asserts the
  local write is still 21 clocks and the local read still 30 — the same
  numbers §8 measured before the UART existed. A wrapper that quietly added
  a cycle to every local transfer would have been a poor trade.

### Cost of a remote transaction

Measured in `tb_uart_remote` with `CLKS_PER_BIT = 4` (simulation): a remote
write is 333 clocks and a remote read 342, against 21 and 30 locally. Almost
all of it is UART time — 5 bytes out and 2 back, at 10 bit-times each. At the
real 115200 baud (`CLKS_PER_BIT = 434`) the same transaction is about 30,000
clocks, or 0.6 ms.

That ratio is the point: **the link is roughly a thousand times slower than
the bus**, which is why the timeout exists and why a future revision might
prefer the split-based bridge after all — a remote transfer that splits would
free the local bus for the whole round trip rather than only for the far
board's part of it.

### Known limits

* The response carries **data only, not a response code**. A remote access to
  an address that is unmapped *on the far board* comes back as `0x00` with
  OKAY, not ERROR. Fixing it means a wider response frame; it is a wire
  format decision, not a bug in the logic.
* **One remote transaction outstanding at a time**, and one incoming request
  held at a time. Both are the same "one outstanding" rule the split slave
  follows.
* **Master 1 is local only.** One UART, one client, and the wire format has
  no field for which master issued a request.

---

## 16. The decoder was made serial too

`system_bus` used to hold a 16-bit `shift_deser` that reassembled the whole
address off `bus_astream` so the combinational `addr_decoder` could compare it
against range constants. It was the one place in the interconnect where the
address became parallel again, and — reasonably — it drew the question: *why
does a serial bus have a 16-bit address register in it?*

It no longer does. The decoder now matches the prefix **as the bits arrive**.

### The problem the old arrangement was solving

Every other receiver on this bus needs no bit counter, because of the
right-aligned frame (§8): a shift register of width W ends up holding "the
last W bits I saw", which is exactly the low-order field it wanted.

The decoder is the one receiver that wants the **other end** of the frame.
Its input is the slave prefix — `addr[15:11]` — and those bits arrive
*first*. No amount of shifting leaves the first bits of a frame in a
register, so "the last W bits" cannot serve it. Buffering all 16 was the
straightforward way out.

### What replaced it

A progressive prefix matcher:

* one **position marker**, `PFX_W` bits wide and one-hot, that starts at the
  first bit of the frame and shifts once per clock. After the prefix it is
  zero and the decoder stops looking — the rest of the address is the slave's
  offset and none of its business.
* one **`alive` bit per slave**, cleared the moment a bit that slave cares
  about disagrees with its prefix.

A slave still alive when the frame ends matched its whole prefix. `addr_done`
strobes the answer out, so the external timing — a one-cycle one-hot pulse —
is unchanged, and nothing downstream noticed.

| | before | after |
|---|---|---|
| Decoder state | 16 flops (the deserialiser) | **8** (5 marker + 3 alive) |
| Decision available | after the 16th address bit | **after the 5th** |
| Map source | hand-written `4'h0` / `5'b00100` | **derived from `bus_defs.vh`** |

The last row is a bonus worth noting. The old compares were literals that did
not track the macros, so moving a slave in `bus_defs.vh` would have left the
decoder behind. The prefix and its care-mask are now computed from `S*_BASE`
and `S*_LADDR_W`, so the map really does live in one place — which
`address_map.md` had been claiming all along.

### The 16-bit register still exists, and is now honestly labelled

The JTAG probe `bus_addr` (`prb[85:70]`) shows the address the bus reassembled
off the single wire, and it is one of the three probes that matter most on
silicon: comparing it with what you sent is how you find a framing fault.
Removing the decoder's need for it does not remove its value as an
observation.

So the deserialiser stays, behind `OBSERVE_ADDR` (default 1), wired to nothing
but the probe. **Set `OBSERVE_ADDR = 0` and it disappears, along with the last
parallel address anywhere in the design.**

That was tested rather than asserted. With `OBSERVE_ADDR = 0` the whole
regression still passes except for exactly four checks — three in
`tb_integration` test 7 and one in `tb_bus_issp_driver` test 8 — and every one
of those four reads `bus_addr` itself. Data integrity, routing, splits, error
recovery, frame length and the measured latencies are all unaffected, which is
the proof that the datapath carries no parallel address.

### Verification

`tb_addr_decoder` drives the DUT the way `system_bus` does — a full
`ADDR_W`-clock frame, then the strobe — and keeps the **exhaustive 65,536
address sweep**, now a frame at a time. Two checks were added for what only a
serial decoder can be asked: that the decision is settled after five bits with
eleven still to come, and that the offset bits cannot change it.

---

## 17. The link was rebuilt to the agreed interface spec

§15 built the UART link the way `System_Bus_Final` does it. The other team
then sent an interface spec for the board-to-board link, and it differs in
four ways that matter. This section is what changed and why.

### Remote is an ADDRESS, not a command bit

`addr[15] == 1` sends the transaction to the other board; the far address is
the local one minus `0x8000`, so `0x9ABC` here is `0x1ABC` there. The
`cmd_remote` sideband and its ISSP source bit are gone — the address carries
the information, so the JTAG console needs no mode and `wa 9ABC 5A` simply
works.

This is where §7 said the remote window would be, so the reserved range is
finally in use. It is still not a *slave*: `bus_bridge` intercepts it in the
command path, so the local decoder never sees `addr[15] == 1` and
`tb_addr_decoder`'s 64K sweep still asserts that nothing up there selects a
slave. If the link were removed, a stray access would answer ERROR rather
than hang.

### The local memory map changed

The spec fixes what both ends must agree on: **2 KB / 4 KB / 4 KB with device
ids 0 / 1 / 2**, and the third slave is the one that splits. This board was
4 KB / 4 KB / 2 KB with the *first* slave splitting.

| | before | after |
|---|---|---|
| slave 0 | 4 KB `0x0000-0x0FFF`, **splits** | 2 KB `0x0000-0x07FF`, plain |
| slave 1 | 4 KB `0x1000-0x1FFF` | 4 KB `0x1000-0x1FFF` |
| slave 2 | 2 KB `0x2000-0x27FF` | 4 KB `0x2000-0x2FFF`, **splits** |
| decode hole | `0x2800-0x2FFF` | `0x0800-0x0FFF` |

This was not optional. With the old geometry a far-side read of
`0xA800-0xAFFF` would have hit this board's decode hole and answered ERROR —
and because the response frame carries data but no status, the far board
would have silently read `0x00` and believed it. A memory map disagreement on
this link is invisible, which is exactly why the spec pins it down.

Total memory bits are unchanged at 81,920: 2K + 4K + 4K is the same 10K words
as 4K + 4K + 2K.

### Writes are posted

The far side sends nothing back for a write, so a `0x5A` frame is never
ambiguous — exactly one arrives per read. The sender retires as soon as the
request is on the wire: a remote write now costs **2 clocks** instead of 333.

The consequence is worth stating plainly: **a remote write completes before
the far board has executed it.** `tb_uart_remote` test 2 asserts the fast
retire and then waits before reading back, which is what any user of the link
must also do.

### The command is 24 bits, not 26

| bits | field |
|---|---|
| `23:16` | wdata |
| `15:14` | dev — 00 / 01 / 10 for slaves 0 / 1 / 2 |
| `13:2` | offset |
| `1` | we |
| `0` | reserved, sent as 0, ignored on receipt |

`dev` and `offset` together are simply the far address's low 14 bits, so the
command is `{wdata, addr[13:0], we, 1'b0}` outbound and `{2'b00, cmd[15:2]}`
inbound. The receiving side's top two address bits are therefore always `00`,
which makes 14 bits lossless **and means a request can never decode back into
the receiver's own remote window**. The link is loop-free by construction and
needs no hop count.

A REQUEST is now 4 bytes rather than 5.

### What did not change

The framing rules from §15 all survive, because the spec agrees with them:
responses beat requests in the shared transmitter, tag hunting takes exactly
3 or exactly 1 more byte without re-scanning the payload, the timeout is
10 ms and returns `0xFF` with `cmd_error`, byte order is little-endian, and
the local path is still a pure pass-through at 21 and 30 clocks.

### Pins

The spec quotes `PIN_D3` and `PIN_C3`, but reads "**my** rm_tx PIN_D3" — they
are the far board's pins, and neither exists on the `EP4CE115F29C7`; the
fitter rejects `D3` outright. The port *names* `rm_tx` / `rm_rx` are shared
vocabulary and worth keeping; the locations are this board's own, `AC15` and
`AB22` on GPIO header JP5. Only the baud rate and frame format have to match.

### Verified on hardware

`quartus_stp -t tcl/issp_bus_test.tcl` passes on the board, including a new
TEST 8 that needs no second board: a read of `0x9ABC` with nothing plugged in
completed in 11 ms, returned `0xFF`, set `cmd_error`, reported `RESP_ERROR`,
and left the local bus working. That exercises the window, the client FSM,
the timeout and the probe bit on real silicon.

Build: 1,426 logic elements, 1,103 registers, 81,920 memory bits, 12 pins,
Fmax 166.8 MHz on `CLOCK_50` and 116.0 MHz on `altera_reserved_tck`.

## Timing constraints, and making an unconstrained path fail the build

`quartus_sta` exits 0 even when it analysed nothing. An unconstrained path is
reported as an *Info*, so a build script that checks only the exit status
cannot tell "everything passed" from "I was never asked". That is not a
hypothetical failure: the earlier parallel project in this repository has no
`.sdc` at all, so its `clk` was never constrained and its Fmax was never
verified — the 50 MHz in its testbenches was simulation-only.

An unconstrained path is not a slow path. It is a path with **no answer**,
and the fitter is free to route it as badly as it likes.

`tcl/sta_check.tcl` runs after the fit, calls `check_timing` and `report_ucp`,
and **exits 1** if anything is unconstrained or if timing is not met. It is
verified in both directions: exit 0 on the current design, exit 1 with the
JTAG cuts removed, and exit 1 again with `create_clock` removed (which
reports 1,067 registers with no clock).

Not every `check_timing` category is a defect:

| Category | Treated as |
|---|---|
| `no_clock`, `latches`, `loops` | **fatal** — always a defect |
| `no_input_delay`, `no_output_delay` | informational — they fire on every port without a `set_input/output_delay`, including ones deliberately cut with `set_false_path`, which is a legitimate answer |
| `report_ucp` non-zero | **fatal** — this is the authority on what is genuinely unanalysed |

`generated_clocks` appears in some documentation as a `check_timing`
category. It is **not** one on Quartus 24.1std; passing it makes the tool
warn and ignore the entire `-include` list.

### What was actually unconstrained

Running this for the first time found the design was *not* fully constrained.
The offenders were not the bus at all — they were the JTAG pins:

```
Unconstrained Input Ports       2      altera_reserved_tdi, altera_reserved_tms
Unconstrained Input Port Paths  40
Unconstrained Output Ports      1      altera_reserved_tdo
```

They arrive with the ISSP megafunction. Quartus constrains the
`altera_reserved_tck` **domain** itself, but not these three **ports**.
Cutting them is correct rather than merely convenient: JTAG is driven by the
USB-Blaster at its own pace, asynchronously to `CLOCK_50`, and no arrival
time relative to the bus clock would mean anything. With those cuts the
design reports **fully constrained for setup and hold**.

The design's own I/O never appeared here — `rst_n`, `rm_rx`, `rm_tx` and
`led[7:0]` are already cut in the `.sdc` with the reasoning written beside
them. That is the distinction the check preserves: it catches what nobody
thought about, not what somebody decided.
