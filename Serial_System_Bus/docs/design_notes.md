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
| 3 | Data width | **32 bits**, parameterised | The brief never states it. Slave sizes are therefore 4K/4K/2K **words**, so `0x0000`–`0x0FFF` is exactly 4096 locations. |
| 4 | Response encoding | `OKAY=2'b00`, `ERROR=2'b01`, `SPLIT=2'b10` | AHB-flavoured. `OKAY` is all-zero so a reset or an idle bus reads as "nothing wrong". |
| 5 | Handshake polarity | everything active high except `rst_n` and the board's `_n` pins | One rule, no exceptions to remember. |
| 6 | Memory initial contents | **none** | Initialising would need an `initial` block or a MIF, and the brief forbids `initial` in synthesisable code. Every test writes a location before reading it instead. The arrays are also not reset — see §3. |
| 7 | What makes slave 0 "busy" | an explicit `split_en` input | The brief says "slave 0 busy → SPLIT" without saying what busy means. Modelling it as an input makes the split deterministic and testable, and puts it on `SW[16]` for the demo. |
| 8 | Board clocking | **no divided clock** | The brief asked for a PLL or clock divider, but a divided clock is a derived/gated clock and would break the brief's own "one clock domain, no gated clocks" rule. The bus runs at 50 MHz and the *scenario sequencer* is throttled with a slow clock-enable tick. Same visible effect, one clock, fully timing-analysable. |
| 9 | Repo layout | `.qpf`/`.qsf`/`.sdc` stay at the `Serial_System_Bus/` root | The brief's layout puts project files in `quartus/`, but the Quartus project already exists here and moving it would break it. `rtl/`, `tb/`, `sim/`, `docs/` follow the brief. |

---

## 2. Protocol shape

A transfer is **one cycle of `bus_valid`** followed by a reply on the next
cycle. Not pipelined, not AHB's two-phase address/data split — the simplest
thing that supports a synchronous memory and a split response.

* **`bus_valid` is exactly one cycle wide.** The slaves treat every cycle of
  `sel` as a fresh access. A master that held `valid` through its wait state
  would open a second transfer. `master.v` drives it in `XFER` only.

* **The return mux uses a registered select.** The memories read
  synchronously, so the reply lands one cycle after the address. Steering the
  return path with the live select would return the wrong slave's data
  whenever the address moved on. `tb_bus_mux` test 4 is exactly this case.

* **The deferred master keeps `bus_req` high.** The arbiter's mask, not
  request withdrawal, is what removes it from arbitration. That keeps the
  replay decision inside the master FSM.

* **`ERROR` is reported, never retried.** An unmapped address will not become
  mapped; a retry would be an infinite loop holding the bus.

Full detail and timing diagrams in [protocol.md](protocol.md).

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
seven-segment displays, so the fitter trimmed the memories to 16 bits.
`SW[13]` now selects which half is displayed, which keeps the whole 32-bit
path alive.

---

## 5. Verification status

Eight self-checking testbenches, all passing. Each prints its own PASS/FAIL
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
| `tb_bus_mux` | reset; the forward mux follows the grant and ignores the other master entirely; the return mux is steered by the **delayed** select, including the case where the live select has already moved on |
| `tb_slave_mem` | both `SPLIT_CAPABLE` builds: reset, write/read-back, `ready` exactly one cycle after `sel` and only one cycle wide, every word its own location (including the `+0x40` aliasing bug), no response while idle, split read, split of a write with the write **not** taking effect until the replay, one-cycle `split_complete` on the right master's bit, and only one split outstanding |
| `tb_default_slave` | reset, `ERROR` one cycle after `sel`, `rdata = 0`, quiet while idle, back-to-back bad addresses |
| `tb_master` | reset; write and read; `m_valid` never wider than one cycle per attempt; `ERROR` reported and **not** retried; `SPLIT` → `bus_req` held high, no transfer driven while masked, the *identical* transfer re-issued on regrant, and exactly **one** `done` for the whole split + replay |
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
| Logic elements | 721 / 114,480 (< 1 %) |
| Registers | 544 |
| Memory bits | 327,680 / 3,981,312 (8 %) — 4K+4K+2K × 32, all in M9K |
| Pins | 106 / 529 |
| **Fmax** | **117.74 MHz** (slow 1200 mV 85 °C) against a 50 MHz requirement |

Timing is constrained by `Serial_System_Bus.sdc`: `create_clock` on
`CLOCK_50` at 20 ns, `derive_clock_uncertainty`, and false paths on the
switches, buttons, LEDs and displays — all of which are driven or read by a
human and are synchronised inside the design.

### Remaining warnings, and why each is expected

| Warning | Explanation |
|---|---|
| `276027` inferred dual-clock RAM, ×3 | The read and write enables differ (`serve && !we` vs `serve && we`), so Quartus infers a simple dual-port RAM and warns generically that read-during-write is undefined. Read-during-write **cannot occur here** — the two enables are mutually exclusive by construction. The fitter report confirms the result is Single Clock. |
| `21074` 13 input pins do not drive logic | `KEY[3:2]` and `SW[12:2]` are brought out to their board pins but unused. Expected. |
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
