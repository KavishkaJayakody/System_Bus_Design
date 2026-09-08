# Bus protocol — serial

One clock domain, one asynchronous active-low reset, one master on the bus at
a time. **The address and the data each travel on a single wire**, MSB first,
one bit per clock.

## Where the protocol lives

Three kinds of module speak this protocol, and the split is strict:

| Module | Role |
|---|---|
| `system_bus` | the bus: arbiter, decoder, `bus_mux`, the central address deserialiser, the default responder. No master, no memory. |
| `master` | drives a frame, absorbs a split, reassembles read data. No arbitration, no decoding. |
| `slave` | shifts every frame in, acts only on `sel`. No arbitration, no decoding. |

There is no integration wrapper: `de2_top` instantiates the three side by
side. Everything below describes the two interfaces between them.

## The shared bus is 8 wires

| Wire | Width | Direction | Meaning |
|---|---|---|---|
| `bus_astream` | 1 | granted master → everyone | serial address |
| `bus_dstream` | 1 | half duplex | serial data |
| `bus_valid` | 1 | granted master → everyone | frame marker, high for `ADDR_W` clocks |
| `bus_we` | 1 | granted master → everyone | 1 = write; also the direction control for `bus_dstream` |
| `bus_ready` | 1 | selected slave → everyone | completion strobe |
| `bus_resp` | 2 | selected slave → everyone | `00` OKAY, `01` ERROR, `10` SPLIT |
| `master_id` | 1 | arbiter → slaves | tag of the granted master |

Plus, point-to-point rather than shared: `bus_req`/`gnt` per master,
`split_complete` per master, and the decoder's 4 select lines.

`bus_dstream` never needs a turnaround, because the two directions cannot
collide by construction: on a **write** only the master sends, during the
frame; on a **read** only the slave sends, after it. So the direction mux is
just `bus_we`.

There are no tristates — an FPGA has no internal tristate buffers — but these
are genuinely single nets that every master and every slave taps.

## The frame

```
                 <---------------- ADDR_W = 16 clocks ---------------->
bus_valid    ____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|______
bus_astream  ----<a15 a14 a13 a12 a11 a10 a9 a8 a7 a6 a5 a4 a3 a2 a1 a0>---
bus_dstream  ----< 0   0   0   0   0   0  0  0 d7 d6 d5 d4 d3 d2 d1 d0>---
                                                ^ write data, RIGHT-ALIGNED
```

**Write data is right-aligned in the frame.** That one choice removes every
bit counter from the receiving side. Each receiver is just a shift register
of its own width, clocked while `bus_valid` is high, and it ends up holding
*the last W bits it saw* — which is exactly the field it wanted:

| Receiver | Width | Ends up holding |
|---|---|---|
| central deserialiser → `addr_decoder` | 16 | the whole address |
| slave 0 / slave 1 address | 12 | `addr[11:0]`, its offset |
| slave 2 address | 11 | `addr[10:0]`, its offset |
| every slave's write data | 8 | the right-aligned data byte |

The upper address bits shift straight through a slave's narrow register and
are discarded — they are the decoder's business, not the slave's. **One frame
timer, in the master, serves every receiver on the bus.**

## Selection happens *after* the frame

The decoder cannot decide anything until the last address bit has arrived. So
every slave shifts every frame in, addressed or not, and selection comes
afterwards:

```
bus_valid    ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|________________     falling edge
addr_done    ____________________|‾‾‾‾‾|__________     = decode strobe (1 clk)
sel[i]       ____________________|‾‾‾‾‾|__________     one-hot, 1 clk
```

`addr_done` is simply the falling edge of `bus_valid`, derived inside
`system_bus`. Deriving it instead of adding an "end of frame" wire keeps the
shared bus at eight wires and keeps the frame length defined in exactly one
place — the master's counter.

`addr_decoder` itself is **unchanged from the parallel design** and still
purely combinational. It is merely enabled one cycle per frame.

Shifting speculatively costs nothing: a slave that was not selected simply
never acts on what it collected. `tb_slave` test 4 drives a full frame
with *no* select and checks that no slave answers and no memory changes.

## Completion

| Case | `ready` arrives | Why |
|---|---|---|
| write | S+1 | the word is committed at S |
| SPLIT | S+1 | the slave is getting out of the way, fast |
| read | S+10 | 1 clock for the M9K output register, 1 to load the output shift register, 8 to shift the data out |
| unmapped | S+1 | the default slave answers immediately |

(S = the cycle `sel` pulses.)

A read data phase:

```
        S    S+1   S+2  S+3 ....  S+9   S+10
sel    ‾|_
mem_q   .   valid
rd_sr   .     .   loaded -shifting->
dstream 0     0     d7   d6  ....  d0     0
ready   _     _      _    _  ....   _   ‾|_
```

**Read data is "the last `DATA_W` bits on `bus_dstream` before `bus_ready`".**
That contract costs no extra wire: the master's deserialiser free-runs
through its wait state and whatever the slave shifted out most recently is
sitting in it when the completion strobe arrives.

The read phase starts at S+2, not S+1, and the extra cycle is deliberate.
`mem_q` has to be a plain register loaded straight from the array so Quartus
can absorb it as the M9K output register; making it the shift register
instead would force an asynchronous array read and drop the whole memory into
LUTs.

## The return select is latched, not delayed

On the parallel bus a reply always came back exactly one cycle after the
address, so a single register was enough. Here a write answers in 1 cycle, a
split in 1, and a read in 10 — **the reply is no longer at a fixed offset**.
So `bus_mux` captures the select on the decoder's one-cycle pulse and holds it
until the next transfer selects something else. Only one transfer is ever
outstanding (the arbiter locks the bus), so holding is safe.

## Split transaction

Unchanged in structure from the parallel design, and much more valuable here.

```
  master 0                arbiter                 slave 0
  --------                -------                 -------
  req, granted  --------> gnt[0], lock
  16-clock frame ------------------------------> shifted in by everyone
                                                 sel, split_en=1, not busy
                <---------------------------- ready, resp=SPLIT   (1 clock)
                mask[0] <= 1                   latch master_id, count down
                gnt <= 0, lock <= 0
  -> SPLIT_WAIT
  bus_req STAYS HIGH
                elig = req & ~mask
                master 1 granted, runs whole
                transfers in the gap
                                              <-- split_complete[0] pulse
                mask[0] <= 0                      resume_pend for master 0
  sees gnt
  -> RE-SENDS THE ENTIRE 16-CLOCK FRAME ------> matches resume_pend
                <---------------------------- ready, resp=OKAY + 8 data bits
  -> DONE, one done pulse for the whole thing
```

Points worth keeping straight:

* **The split answer costs 1 clock; the read it defers costs 10.** On a serial
  bus that asymmetry is the whole point — the slave gets out of the way fast
  and the bus goes to somebody else for tens of clocks, not a handful.

* **The replay re-sends the complete frame.** It is a genuine re-issue, not a
  resumption of a half-finished one. `tb_master` test 6 and `tb_integration` test
  7 both check that the second frame is full length and carries the identical
  address and data.

* **The deferred master keeps `bus_req` asserted.** The arbiter's mask, not
  request withdrawal, is what removes it from arbitration.

* **The deferred transfer is not performed.** No write lands, no read happens.
  The master re-sends everything, so the slave only has to remember *who* it
  deferred — which is why there is no address or data storage in the split
  logic.

* **One split outstanding at a time.** An access arriving while slave 0 is
  busy, or while a resume is owed, is served normally.

## Unmapped addresses

An address matching no slave selects the **default slave**, which replies
`ready` + `ERROR` at S+1. It needs no deserialisers — it does not care what
the address or data were, only that nothing else claimed them — and it never
drives the data wire, so a read of an unmapped address reassembles zero.

Without it, an unmapped access would assert no select, no slave would ever
drive `ready`, and the granted master would hold the bus forever with only a
reset to recover it. `tb_integration` test 5 fires four consecutive bad addresses
and then checks the very next transfer still returns correct data.

`ERROR` is reported, never retried — an unmapped address will not become
mapped.

## Arbitration

**`arbiter.v` is unchanged from the parallel design.** It only ever looked at
`req`, `bus_ready` and `bus_resp`, none of which were serialised. Fixed
priority, master 0 highest, grant locked for the duration of a transfer —
which is now 21 to 30 clocks instead of 5, so the lock matters far more.

Still parameterised on `N_MASTERS` for the phase-2 remote bridge.

## Measured latency (simulation, `tb_integration`)

| Case | Clocks, command accepted → `done` |
|---|---|
| Write | **21** |
| Read | **30** |
| Read while the other master contends | 30 / 59 |
| Split read, `SPLIT_LATENCY = 6` | 79 |

A write costs 21 and a read 30 — the 9-clock difference is exactly the read
data phase. The address frame dominates both: 16 of those clocks are the
address going out one bit at a time, which is the price of a one-wire bus.

Latency ordering between the two masters is an artefact of fixed priority.
Do not write assertions on it.
