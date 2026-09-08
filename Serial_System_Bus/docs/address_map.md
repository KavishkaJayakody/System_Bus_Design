# Address map

16-bit **word** address, 8-bit data, both carried **one bit at a time** on a
single wire each. 64K word space, of which 10K words are mapped.

The address is 16 bits because the map needs it, and that means every
transfer spends 16 clocks shifting it. See [protocol.md](protocol.md).

| Slave | Range | Size | Device id | Decode | Notes |
|---|---|---|---|---|---|
| Slave 0 | `0x0000`–`0x07FF` | 2048 x 8 = **2 KB** | 0 | `addr[15:11] == 5'b00000` | plain memory |
| Slave 1 | `0x1000`–`0x1FFF` | 4096 x 8 = **4 KB** | 1 | `addr[15:12] == 4'h1` | plain memory |
| Slave 2 | `0x2000`–`0x2FFF` | 4096 x 8 = **4 KB** | 2 | `addr[15:12] == 4'h2` | **split capable** |
| Default slave | everything else below `0x8000` | — | — | no other match | answers `ERROR` |

**The sizes and the device ids are fixed by the board-to-board link spec.**
Both ends must agree on 2 KB / 4 KB / 4 KB and ids 0/1/2, or a remote access
lands somewhere other than where the far board believes it sent it. Changing
them here means changing them on the other board too.

## The remote window

`addr[15] == 1` is not a slave at all. `master_uart` takes those
transactions off to the other board over the UART link instead of putting
them on the local bus, and the far address is the local one minus `0x8000`:

| Type here | Reaches | Far slave |
|---|---|---|
| `0x8000`–`0x87FF` | the far board's `0x0000`–`0x07FF` | slave 0, 2 KB, id 0 |
| `0x9000`–`0x9FFF` | the far board's `0x1000`–`0x1FFF` | slave 1, 4 KB, id 1 |
| `0xA000`–`0xAFFF` | the far board's `0x2000`–`0x2FFF` | slave 2, 4 KB, id 2 |

`0x9ABC` here is `0x1ABC` there. Because slave 0 is only 2 KB,
`0x8800`–`0x8FFF` mirrors `0x8000`–`0x87FF` on the far side.

The window never reaches the local decoder, so the decoder still treats
`addr[15] == 1` as unmapped — which is what `tb_addr_decoder`'s 64K sweep
still asserts. That matters: if the link were ever removed, a stray access up
there would answer `ERROR` rather than hang.

## Unmapped regions

| Range | Why it is unmapped |
|---|---|
| `0x0800`–`0x0FFF` | the hole above slave 0, because slave 0 is 2K inside a 4K-aligned decode |
| `0x3000`–`0x7FFF` | nothing mapped there |
| `0xB000`–`0xFFFF` | above the remote window; leaves over the link and the far board answers nothing useful |

Every one of the first two decodes to the default slave, which completes the
transfer in the normal one cycle with `RESP_ERROR`. The bus therefore cannot
hang on a bad address — see [protocol.md](protocol.md).

`tb_addr_decoder` proves this exhaustively: it sweeps all 65,536 addresses and
checks that `{def_sel, slv_sel}` is one-hot for every single one, and that
nothing inside `addr[15] == 1` ever selects a real slave.

## How much of the address each slave actually sees

A slave never receives the whole address. Its deserialiser is only as wide as
its own offset, so after the 16-clock frame it holds exactly the low bits it
needs and the upper bits have shifted straight through and been discarded.

| Slave | Deserialiser width | Holds |
|---|---|---|
| Slave 0 | 11 | `addr[10:0]` |
| Slave 1 | 12 | `addr[11:0]` |
| Slave 2 | 12 | `addr[11:0]` |

Deciding *which* slave is the decoder's job — and it does not assemble the
address either. It matches the prefix serially as the bits arrive, using a
5-bit position marker and one `alive` bit per slave. **Nowhere in the
datapath does a 16-bit address exist.** `tb_slave` test 3 checks this directly on an 11-bit slave: two addresses
sharing `addr[10:0]` must hit the same word.

## Constants

All of the above lives in one place, `rtl/bus_defs.vh`. Change a range there
and the decoder, the slave sizes and the testbenches all follow.

## Addresses used by the testbenches

There is no on-board scenario sequencer any more — every transaction is
issued over JTAG — so the addresses below are the ones the testbenches and
the Tcl console exercise. They pick varied nibbles deliberately: a constant
byte lane is one the fitter is entitled to delete, which is how an earlier
revision of this design ended up with narrower memories than it specified.

| Slave | Addresses used |
|---|---|
| Slave 0 (2K) | `0x0000`, `0x0040`, `0x05C3`, `0x07FF` |
| Slave 1 (4K) | `0x1000`, `0x1234`, `0x1ABC`, `0x1DEF`, `0x1FFF` |
| Slave 2 (4K, splits) | `0x2000`, `0x2010`, `0x2100`, `0x2345`, `0x2567`, `0x2A5C`, `0x2FFF` |
| Unmapped | `0x0800` / `0x0FFF` (the hole), `0x3000`, `0x4000`, `0x7FFF` |
| Remote | `0x85C3`, `0x9ABC`, `0xA100`, `0xA567`, `0xAFFF` |
