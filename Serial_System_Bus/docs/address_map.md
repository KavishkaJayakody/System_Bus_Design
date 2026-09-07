# Address map

16-bit **word** address, 8-bit data, both carried **one bit at a time** on a
single wire each. 64K word space, of which 10K words are mapped.

The address is 16 bits because the map needs it, and that means every
transfer spends 16 clocks shifting it. See [protocol.md](protocol.md).

| Slave | Range | Size | Decode | Notes |
|---|---|---|---|---|
| Slave 0 | `0x0000`–`0x0FFF` | 4096 x 8 = **4 KB** | `addr[15:12] == 4'h0` | split capable |
| Slave 1 | `0x1000`–`0x1FFF` | 4096 x 8 = **4 KB** | `addr[15:12] == 4'h1` | plain memory |
| Slave 2 | `0x2000`–`0x27FF` | 2048 x 8 = **2 KB** | `addr[15:11] == 5'b00100` | plain memory |
| Default slave | everything else | — | no other match | answers `ERROR` |

## Unmapped regions

| Range | Why it is unmapped |
|---|---|
| `0x2800`–`0x2FFF` | the hole above slave 2, because slave 2 is 2K inside a 4K-aligned decode |
| `0x3000`–`0x7FFF` | nothing mapped there |
| `0x8000`–`0xFFFF` | **reserved for the phase-2 remote window** (`addr[15] == 1`) |

Every one of these decodes to the default slave, which completes the transfer
in the normal one cycle with `RESP_ERROR`. The bus therefore cannot hang on a
bad address — see [protocol.md](protocol.md).

`tb_addr_decoder` proves this exhaustively: it sweeps all 65,536 addresses and
checks that `{def_sel, slv_sel}` is one-hot for every single one, and that
nothing inside `addr[15] == 1` ever selects a real slave.

## How much of the address each slave actually sees

A slave never receives the whole address. Its deserialiser is only as wide as
its own offset, so after the 16-clock frame it holds exactly the low bits it
needs and the upper bits have shifted straight through and been discarded.

| Slave | Deserialiser width | Holds |
|---|---|---|
| Slave 0 | 12 | `addr[11:0]` |
| Slave 1 | 12 | `addr[11:0]` |
| Slave 2 | 11 | `addr[10:0]` |
| Central, for the decoder | 16 | the whole address |

Deciding *which* slave is the decoder's job, and it is the only receiver that
needs all 16 bits. `tb_slave_mem` test 3 checks this directly: `0x2123` and
`0xF923` share `addr[10:0]`, so on the 2K slave they must hit the same word.

## Constants

All of the above lives in one place, `rtl/bus_defs.vh`. Change a range there
and the decoder, the slave sizes and the testbenches all follow.

## Addresses used by the on-board demo

`master_prog` picks addresses with varied nibbles so every seven-segment digit
actually changes (round numbers leave most segments stuck, and the fitter
reports the pins as tied).

| Scenario | Master 0 | Master 1 |
|---|---|---|
| `00` one master | W/R `0x1234`, W/R `0x2567` | idle |
| `01` two masters | W/R `0x1ABC`, W/R `0x1DEF` | W/R `0x2345`, W/R `0x2678` |
| `10` split | W/R `0x0A5C`, W/R `0x0369` | W/R `0x21A7`, W/R `0x24E3` |
| `11` unmapped | R `0x2ABC` (hole), R `0x1234`, R `0x8DEF` (reserved), R `0x2567` | R `0x2345` ×4 |
