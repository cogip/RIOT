# riotboot CAN-FD Streaming Protocol

## Overview

The CAN bootloader uses a streaming protocol backed by `riotboot_flashwrite` (RAW mode).
The **host** streams firmware bytes; the **device** manages flash erasure and page boundaries internally.

CAN-FD frames carry up to 64 bytes. With protocol overhead (type + len + crc8 = 3 bytes),
each DATA frame delivers up to **61 bytes** of firmware payload.

## Frame Format

All CRC-protected commands use:

```
[type(1B) | len(1B) | data(len B) | crc8(1B)]
```

- CRC-8 is computed over `type + len + data` (first `2 + len` bytes)
- Polynomial: `0x31`, initial value: `0xFF`
- Exception: `CMD_BOOT` has no CRC (`[type | slot_char]`)

## CAN ID Addressing

Each node has a unique `NODE_ID` (0-127). CAN IDs are derived as:

| Direction    | CAN ID                          |
|-------------|---------------------------------|
| Host -> Node | `BASE_ID + 2 * NODE_ID`        |
| Node -> Host | `BASE_ID + 2 * NODE_ID + 1`    |

Default `BASE_ID` = `0x100`. Set `NODE_ID` via `CFLAGS += -DRIOTBOOT_CAN_NODE_ID=N`.

## Synchronisation (Handshake)

```
Host                          Device
  |--- PROBE ('?') ----------->|
  |<-- STAT_WAITING ('b') -----|
  |                             |
  |--- ENTER_LOADER ('B') ---->|
  |<-- STAT_READY ('>') -------|
```

The device sends `STAT_WAITING` periodically during the boot delay (`RIOTBOOT_CAN_DELAY_MS`, default 250ms).
The host must send `ENTER_LOADER` before the delay expires to prevent auto-boot.

Forced entry: writing `RIOTBOOT_MAGIC` to `RIOTBOOT_MAGIC_ADDR` before reset skips the delay.

**Note:** PROBE and ENTER_LOADER are also handled in the main command loop after entering
the bootloader, allowing the host to re-synchronise at any time.

## Commands

### CMD_START (`'s'`) - Start Firmware Update

The bootloader automatically selects the best slot for the update (in priority order):
1. First slot with an invalid firmware (corrupt or absent)
2. First slot whose header start address does not match the expected image start address (inconsistent header)
3. Among valid slots, the one with the lowest firmware version (oldest firmware gets overwritten)

Initialises `riotboot_flashwrite` for the selected slot. Erases the first flash page
and sets the write offset to 4 (skipping the "RIOT" magic bytes).

```
Host -> [s, 0x00, crc8]
Device -> ['.', slot(1B), size_b0, size_b1, size_b2, size_b3]   (OK + slot + slot_size LE)
```

- `slot`: selected slot number (chosen by the bootloader)
- Response includes the slot size (4 bytes, little-endian) so the host can validate firmware length

Errors:
- `'!'` (ILLEGAL) - no slot available
- `'E'` (ERROR) - flash init failed

### CMD_DATA (`'d'`) - Firmware Data Chunk

Streams firmware bytes sequentially via `riotboot_flashwrite_putbytes()`.
The flashwrite module handles page buffering, boundary crossing, and automatic page erasure.

```
Host -> [d, len, fw_bytes(len B), crc8]
Device -> ['.']                                        (OK)
```

- `fw_bytes`: firmware data **starting from byte 4** (the host MUST skip the first 4 bytes = "RIOT" magic)
- Maximum `len` per frame: 61 bytes (CAN-FD) or 5 bytes (classic CAN)
- Chunks must be sent **sequentially** in order

Errors:
- `'!'` (ILLEGAL) - no update in progress, or `len == 0`
- `'E'` (ERROR) - flash write failed (update is aborted, slot remains invalid)

### CMD_FINISH (`'f'`) - Finish Firmware Update

Flushes any remaining buffered data, then writes the "RIOT" magic to the first 4 bytes
of the slot via `riotboot_flashwrite_finish()`. This is the **atomic commit**: the image
becomes valid and bootable only after this command succeeds.

```
Host -> [f, 0x00, crc8]
Device -> ['.']                                        (OK)
```

Errors:
- `'!'` (ILLEGAL) - no update in progress
- `'E'` (ERROR) - flash flush or finish failed

### CMD_BOOT (`'b'`) - Boot Application

```
Host -> [b, slot_char]          (no CRC)
Device -> ['.']                 (OK, then reboots)
```

- `slot_char`: `'0'` = slot 0, `'1'` = slot 1, `'\n'` = default (highest version)

## Complete Flashing Sequence

```
Host                              Device
  |--- PROBE --------------------->|
  |<-- STAT_WAITING ---------------|
  |--- ENTER_LOADER -------------->|
  |<-- STAT_READY -----------------|
  |                                 |
  |--- CMD_START ------------------>|  auto-selects slot + riotboot_flashwrite_init()
  |<-- OK + slot + slot_size ------|
  |                                 |
  |--- CMD_DATA(chunk_1) --------->|  riotboot_flashwrite_putbytes()
  |<-- OK -------------------------|
  |--- CMD_DATA(chunk_2) --------->|  riotboot_flashwrite_putbytes()
  |<-- OK -------------------------|
  |    ...                          |
  |--- CMD_DATA(chunk_N) --------->|  riotboot_flashwrite_putbytes()
  |<-- OK -------------------------|
  |                                 |
  |--- CMD_FINISH ---------------->|  flush() + finish() -> writes "RIOT" magic
  |<-- OK -------------------------|
  |                                 |
  |--- CMD_BOOT('\n') ------------>|  reboots into updated firmware
  |<-- OK -------------------------|
```

## Response Codes

| Code | Char | Meaning                        |
|------|------|--------------------------------|
| OK   | `'.'`| Operation successful            |
| BAD_CRC | `'?'` | CRC-8 checksum mismatch    |
| ILLEGAL | `'!'` | Invalid parameter / state   |
| ERROR   | `'E'` | Flash operation failed      |

## Safety Properties

1. **Atomic update**: The "RIOT" magic is written last (`CMD_FINISH`).
   A partially-written image is never bootable.
2. **Power-loss safe**: If power is lost mid-update, the slot has no valid magic
   and will be skipped at boot. The other slot (if valid) boots normally.
3. **Bootloader protection**: `riotboot_flashwrite` only writes to the target slot,
   never to the bootloader region.
4. **CRC integrity**: Every command (except BOOT) is CRC-8 protected.
   A CRC failure is rejected without side effects; the host can retransmit.
5. **State tracking**: DATA and FINISH are rejected if no update is in progress.
   A flash error during DATA aborts the update (slot stays invalid).
