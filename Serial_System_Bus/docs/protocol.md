# Bus protocol

One clock domain, one asynchronous active-low reset, one master on the bus at
a time.

## Signals

Forward path (granted master → slaves), driven by `bus_mux`:

| Signal | Width | Meaning |
|---|---|---|
| `bus_valid` | 1 | access strobe, **exactly one cycle per attempt** |
| `bus_we` | 1 | 1 = write |
| `bus_addr` | 16 | word address |
| `bus_wdata` | 32 | write data |
| `master_id` | 1 | tag of the granted master, driven by the arbiter |

Return path (selected slave → all masters), also through `bus_mux`:

| Signal | Width | Meaning |
|---|---|---|
| `bus_ready` | 1 | completion strobe, one cycle |
| `bus_resp` | 2 | `00` OKAY, `01` ERROR, `10` SPLIT |
| `bus_rdata` | 32 | read data, valid with `bus_ready` when `resp = OKAY` |

## Timing of one transfer

```
            T-1      T        T+1      T+2
clk       __|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
bus_req   _____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|______   master holds it until done
gnt       __________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾|________  arbiter locks the bus
bus_valid ______________|‾‾‾‾‾|______________  ONE cycle
sel[i]    ______________|‾‾‾‾‾|______________  decoder, gated by bus_valid
sel_q[i]  ____________________|‾‾‾‾‾|________  registered copy
ready     ____________________|‾‾‾‾‾|________  slave replies here
rdata     ====================< valid >======
```

Two rules follow from this picture, and both are load-bearing:

* **`bus_valid` is one cycle wide.** The slaves treat every cycle of `sel` as
  a new access, so a master that held `valid` through its wait state would
  start a second transfer. `master.v` drives it in the `XFER` state only and
  then waits in `WAIT`.

* **The return mux is steered by `sel_q`, the select delayed one cycle.** The
  memories read synchronously, so the reply arrives in T+1 while the live
  select may already point somewhere else. `tb_bus_mux` test 4 checks exactly
  this case.

## Split transaction

```
  master 0                arbiter                 slave 0
  --------                -------                 -------
  req, granted  --------> gnt[0], lock
  drive valid   -----------------------------> sel, fresh access
                                               split_en=1, not busy
                <---------------------------- ready, resp = SPLIT
                mask[0] <= 1                   latch master_id, count down
                gnt <= 0, lock <= 0
  sees SPLIT
  -> SPLIT_WAIT
  bus_req STAYS HIGH
                elig = req & ~mask             ... counting ...
                master 1 granted
                master 1 runs real transfers
                                              <-- split_complete[0] pulse
                mask[0] <= 0                       resume_pend for master 0
                master 0 eligible again
                gnt[0] <= 1
  sees gnt
  -> XFER, RE-ISSUES the identical transfer -> sel, matches resume_pend
                <---------------------------- ready, resp = OKAY, rdata
  -> DONE, one done pulse for the whole thing
```

Points worth keeping straight:

* **The deferred master keeps `bus_req` asserted.** It is the arbiter's mask,
  not request withdrawal, that removes it from arbitration. That is what lets
  the replay decision live entirely inside the master FSM.

* **The deferred transfer is not performed.** No write lands, no `rdata` is
  captured. The master replays the whole transfer, so the slave only has to
  remember *who* it deferred, not what — which is why there is no address or
  data storage in the split logic.

* **The caller sees one command and one `done`.** However many times the
  transfer was actually driven onto the bus, `master.v` reports it once. The
  split count is exported separately for the LEDs and the testbenches.

* **One split outstanding at a time.** An access arriving while slave 0 is
  busy, or while a resume is still owed, is served normally with `OKAY`. This
  is deliberate: with single-entry deferred state, allowing a second split
  would mask two masters with only one `split_complete` to wake them.

## Unmapped addresses

An address matching no slave selects the **default slave**, which replies
`ready` + `ERROR` on the usual T+1 cycle. The master reports the error and
returns to `IDLE`; the arbiter sees `ready` and releases the grant.

Without a default slave, an unmapped access would assert no select at all, no
slave would ever drive `ready`, and the granted master would hold `bus_req`
forever with only a reset to recover it. `tb_bus_top` test 5 fires four
consecutive bad addresses and then checks that the very next transfer still
returns correct data.

`ERROR` is reported, never retried — an unmapped address will not become
mapped, so a retry would be an infinite loop.

## Arbitration

Fixed priority, master 0 highest. The grant is **locked** for the duration of
a transfer and re-arbitrated on the cycle after it completes, so master 0
cannot hold the bus across a whole burst but a single transfer is never torn
in half.

The arbiter is parameterised on `N_MASTERS` (with `ID_W = ceil(log2 N)`), not
hard-coded for two, so the phase-2 remote bridge can be added as a third
requester without touching the logic.

## Measured latency (simulation, `tb_bus_top`)

| Case | Clocks, command accepted → `done` |
|---|---|
| Read of a mapped slave, uncontended | 5 |
| Read while the other master is contending | 5 / 9 |
| Split read, `SPLIT_LATENCY = 6` | 16 |

Latency ordering between the two masters is an artefact of fixed priority.
Do not write assertions on it.
