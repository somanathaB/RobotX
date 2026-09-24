# Pi 5 ⇄ ESP32 Link Protocol — v2

Specification of the UART protocol between the Raspberry Pi 5 (high-level controller) and
the ESP32 (low-level hardware controller), as implemented in `protocol.cpp` and
`comm.cpp`. A Python reference implementation is in `tests/rover_link.py`.

Protocol v2 **replaces** v1 (bare JSON lines, `strstr` parsing, no CRC). It is not
backward compatible: see [Changes from v1](#14-changes-from-v1).

---

## 1. Physical layer

| Item | Value |
|---|---|
| ESP32 port | UART0 (`Serial`), GPIO1 = TX → Pi RX, GPIO3 = RX ← Pi TX, common GND |
| Settings | 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control |
| Byte time | 10 bits per byte → 86.8 µs per byte, 11 520 bytes/s maximum |
| ESP32 RX ring | 1024 bytes · TX ring 3072 bytes |

UART0 is also the ESP32 boot ROM's console. **The ROM prints plain text at every reset,
before the firmware runs; firmware cannot suppress it.** That text never carries a valid
CRC trailer, so the Pi discards it (see §12).

## 2. Frame format

Every message in both directions is one line:

```
FRAME   = PAYLOAD "*" CRC4 LF
PAYLOAD = one flat JSON object, printable ASCII (0x20–0x7E), starting "{" ending "}"
CRC4    = 4 hex digits (ESP32 sends upper case; either case accepted on receive)
LF      = 0x0A          (one CR, 0x0D, immediately before LF is tolerated and ignored)
```

Example (Pi → ESP32):

```
{"type":"COMMAND","seq":42,"cmd":"DRIVE","left":150,"right":150}*E0DC
```

Receive rules (ESP32; the Pi should apply the same):

| Condition | Result |
|---|---|
| Empty line (or CR only) | Ignored. Sending a lone LF is a safe way to resynchronise. |
| More than **160 bytes** before LF | The entire line is discarded, never truncated. One `ERROR FRAME_TOO_LONG` is sent when its LF arrives. |
| Any byte outside 0x20–0x7E (CR before LF excepted) | `ERROR INVALID_FRAME` |
| Does not end in `*` + 4 hex digits | `ERROR INVALID_FRAME` |
| First byte is not `{` (garbage before a frame) | `ERROR INVALID_FRAME`. The line is never searched for an embedded command. |
| CRC mismatch | `ERROR INVALID_CRC` |

Partial frames are accumulated across reads for as long as they take to arrive. Two
frames written back to back are processed in order.

## 3. CRC

| Parameter | Value |
|---|---|
| Algorithm | CRC-16/CCITT-FALSE |
| Polynomial | 0x1021, processed MSB first |
| Initial value | 0xFFFF |
| Input / output reflection | none |
| Final XOR | 0x0000 |
| Check value | CRC(`"123456789"`) = `29B1` |
| Bytes covered | The PAYLOAD exactly as transmitted, from `{` through `}` inclusive. Not the `*`, the CRC digits, CR or LF. |
| Text form | 4 hex digits, most significant nibble first (`0x0A3F` → `0A3F`) |
| On mismatch | The frame is discarded, **nothing** in it is acted on, and `ERROR INVALID_CRC` with `"seq":null` is returned. The seq inside a corrupt frame is never trusted. |

```python
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

## 4. Payload grammar (strict)

Payloads are a deliberately small subset of JSON:

| Element | Accepted | Rejected |
|---|---|---|
| Object | one flat object, at most **10** keys, no duplicate keys | nesting, arrays, trailing comma, anything after `}` |
| Key | `"` + 1–16 chars of `[A-Za-z0-9_]` + `"` | other characters, empty key |
| String | 0–32 printable chars, no `"` and no `\` | escapes of any kind |
| Integer | `-?(0\|[1-9][0-9]*)` | `150abc`, `1.5`, `1e3`, `01`, `-`, `0x10`, `+5` |
| Literals | `true`, `false`, `null` | anything else |
| Whitespace | space / tab between tokens | — |

Integers outside the int32 range are well-formed JSON; they are rejected later as
`OUT_OF_RANGE`, **never wrapped or clamped**.

## 5. Message types

Every frame's first key is `"type"`. The Pi dispatches on `type` alone.

| type | Direction | Purpose |
|---|---|---|
| `COMMAND` | Pi → ESP32 | the only type the ESP32 accepts |
| `ACK` | ESP32 → Pi | exactly one per command frame that passed the envelope checks (§7) |
| `ERROR` | ESP32 → Pi | a line that could not be accepted as a command at all |
| `EVENT` | ESP32 → Pi | something happened: `READY`, `COMMAND_TIMEOUT`, `MOTORTEST_DONE`, boot `I2CSCAN` |
| `TELEMETRY` | ESP32 → Pi | compact operational state, every 200 ms |
| `DIAG` | ESP32 → Pi | detailed diagnostics, one rotating section every 400 ms |

## 6. Commands (Pi → ESP32)

Envelope, required on every command:

| Key | Type | Rule |
|---|---|---|
| `type` | string | must be `"COMMAND"` |
| `seq` | integer | 1 … 65535 (§8) |
| `cmd` | string | command name, case-sensitive |

Command fields. **All listed fields are required unless marked optional. Any other key is
rejected with `UNKNOWN_FIELD`.** Ranges are inclusive, and values outside them are rejected.

| cmd | Fields | Effect | Refreshes watchdog |
|---|---|---|---|
| `DRIVE` | `left` int −255…255, `right` int −255…255 | differential drive request | **yes** (result ACCEPTED or GATED) |
| `STOP` | — | zero both sides, hold stopped | **yes** |
| `MOVE` | `dir` `"F"`/`"B"`/`"L"`/`"R"`/`"S"`; `speed` int 0…255 (optional only for `"S"`) | F=(s,s) B=(−s,−s) L=(55%·s,s) R=(s,55%·s) S=STOP | **yes**, except an invalid `dir` |
| `RESET` | — | clear the safety-stop and timeout latches; no motion | no |
| `PING` | — | liveness; ACK carries `state`, `uptime_ms`, `proto` | no |
| `MOTORTEST` | `motor` 0…3, `power` −255…255, `ms` 1…3000, `onblocks` bool (must be `true`) | spin ONE raw channel, safety gate bypassed. **Rover on blocks.** | no |
| `I2CSCAN` | — | scan 0x08–0x77; refused while the wheels are driven | no |
| `I2CSTATUS` | — | SDA/SCL pad levels, no bus traffic | no |
| `TCATEST` | `addr` int 8…119 (optional; the configured address once it is confirmed) | TCA9548A read-back test | no |
| `PCATEST` | `addr` int 8…119 (optional, as above) | read-only PCA9685 probe | no |
| `TOFTEST` | `sensor` int 0…2 (optional; all three if absent) | one blocking VL53L0X read each | no |
| `HWREPORT` | — | configuration and the list of what is unverified | no |

Diagnostic results are carried **inside the ACK**, so every command still gets exactly one
response.

## 7. Validation order and responses

Checks run in this order. The first failure decides the response, so a frame with several
faults always yields the same reason.

| # | Check | Failure response |
|---|---|---|
| 1 | Line length ≤ 160 bytes | `ERROR FRAME_TOO_LONG`, seq null |
| 2 | Printable ASCII, `*XXXX` trailer, leading `{` | `ERROR INVALID_FRAME`, seq null |
| 3 | CRC | `ERROR INVALID_CRC`, seq null |
| 4 | Grammar (§4) | `ERROR INVALID_MESSAGE` / `MALFORMED_NUMBER` / `DUPLICATE_FIELD` / `TOO_MANY_FIELDS` / `FIELD_TOO_LONG`, seq null |
| 5 | `seq` present, integer, 1…65535 | `ERROR MISSING_FIELD` / `WRONG_TYPE` / `INVALID_SEQUENCE` (field `seq`), seq null |
| 6 | `type` present, string, `"COMMAND"` | `ERROR MISSING_FIELD` / `WRONG_TYPE` / `INVALID_MESSAGE_TYPE` (field `type`), **seq echoed** |
| 7 | `cmd` present, string | `ERROR MISSING_FIELD` / `WRONG_TYPE` (field `cmd`), **seq echoed** |
| 8 | Sequence order (§8) | `ACK DUPLICATE` or `ACK REJECTED STALE_SEQ`, not executed |
| 9 | Command known | `ACK REJECTED UNKNOWN_COMMAND` |
| 10 | Unknown keys, then required fields **in the order listed in §6** | `ACK REJECTED UNKNOWN_FIELD` / `MISSING_FIELD` / `WRONG_TYPE` / `OUT_OF_RANGE` / `INVALID_ARGUMENT`, with `field` |
| 11 | Command-specific preconditions | `ACK REJECTED <reason>` (e.g. `MOTOR_PWM_UNAVAILABLE`, `ONBLOCKS_REQUIRED`, `I2C_NOT_READY`, `TCA_ADDRESS_UNCONFIRMED`) |
| 12 | Executed | `ACK ACCEPTED`, or `ACK GATED` when the safety or availability gate limited the output |

**The response rule for the Pi:** after sending seq N, expect exactly one `ACK` or `ERROR`
carrying `"seq":N`. If it never arrives, the frame was corrupted or garbled beyond the
point where seq could be trusted (the ESP32 then sends an `ERROR` with `"seq":null`).

ERROR frames with `"seq":null` are rate-limited to **10 per second**, so line noise cannot
flood the TX link. Suppressed errors are counted in `DIAG SYSTEM link.errors_suppressed`.

## 8. Sequence numbers

| Rule | Value |
|---|---|
| Range | 1 … 65535. **0 is invalid** (`INVALID_SEQUENCE`). |
| Pi behaviour | increment by 1 per command; after 65535 comes 1 |
| Order | `steps = (seq − last_seq) mod 65535`, where `last_seq` is the seq of the most recent frame that received an `ACK` |
| `steps == 0` | **duplicate**: `ACK result DUPLICATE`, `reason DUPLICATE_SEQ`, plus `original_cmd` / `original_result`. Not executed. |
| `1 ≤ steps ≤ 32767` | new: processed normally |
| `steps > 32767` | **stale** (an older frame replayed): `ACK result REJECTED`, `reason STALE_SEQ`, plus `last_seq`. Not executed. |
| What updates `last_seq` | only frames answered by an `ACK` other than DUPLICATE or STALE, whether ACCEPTED, GATED or REJECTED. ERROR frames never update it. |
| After ESP32 reboot | `last_seq` is forgotten (`TELEMETRY "last_seq":null`), so the next seq is always accepted. The Pi detects the reboot from `EVENT READY` or `uptime_ms` going backwards. |
| After Pi restart | read `last_seq` from any `TELEMETRY` frame and continue from `last_seq + 1`. On `STALE_SEQ`, set seq to the ACK's `last_seq + 1`. |

A seq identifies **one frame's content**. A corrected command needs a new seq; reusing the
seq of a REJECTED command returns DUPLICATE.

Duplicates and stale frames are never executed. They do not refresh the watchdog, and
they neither restart nor cancel a running `MOTORTEST`. For DRIVE/STOP a retransmission is
therefore idempotent: the original already took effect.

## 9. ACK

```
{"type":"ACK","seq":42,"cmd":"DRIVE","result":"GATED","reason":"MOTOR_PWM_UNAVAILABLE",
 "req_left":150,"req_right":150,"gated_left":0,"gated_right":0,
 "applied_left":0,"applied_right":0}*XXXX
```

| Key | Always | Meaning |
|---|---|---|
| `seq`, `cmd` | yes | echoed from the command |
| `result` | yes | `ACCEPTED` · `GATED` (valid and executed, but output limited by safety/availability) · `REJECTED` · `DUPLICATE` |
| `reason` | yes | `NONE` when ACCEPTED, otherwise the precise cause |
| `field` | on field errors | the offending key |

For DRIVE, MOVE and STOP the ACK also carries three pairs of values:

| Pair | Meaning |
|---|---|
| `req_left/right` | what the command asked for |
| `gated_left/right` | what the safety and availability gates permitted |
| `applied_left/right` | what the motor layer **actually outputs**: the gated value after clamp and the ±40 deadband, and 0 when there is no verified PCA9685 |

Since out-of-range values are rejected, `applied` can only differ from `gated` through the
deadband (|v| < 40 → 0) or through drive being unavailable.

GATED reasons: `MOTOR_PWM_UNAVAILABLE`, `FRONT_OBSTACLE`, `FRONT_SENSOR_FAULT`,
`REAR_OBSTACLE`, `REAR_SENSOR_FAULT`, `REAR_UNCONFIGURED`.

## 10. ERROR and EVENT

```
{"type":"ERROR","seq":null,"reason":"INVALID_CRC"}*XXXX
{"type":"ERROR","seq":17,"reason":"INVALID_MESSAGE_TYPE","field":"type"}*XXXX
{"type":"ERROR","seq":null,"reason":"TX_FRAME_OVERFLOW","uptime_ms":1234}*XXXX
```

`TX_FRAME_OVERFLOW` means an outgoing frame did not fit the 2560-byte buffer. It carries the
seq when the lost frame was an ACK, and replaces that frame; a truncated frame is never sent.

| EVENT | When | Extra keys |
|---|---|---|
| `READY` | once, at the end of `setup()` | `proto`, `fw`, `core`, `seq_min`, `seq_max`, `line_max`, hardware status |
| `I2CSCAN` | once at boot (`trigger":"BOOT"`) | scan result |
| `COMMAND_TIMEOUT` | watchdog trips (false → true transition; again after a RESET) | `command_age_ms`, `timeout_ms` |
| `MOTORTEST_DONE` | a test ends | `seq` of the MOTORTEST, `reason` `EXPIRED`/`CANCELLED`/`REPLACED` |

Every EVENT carries `seq` (null except `MOTORTEST_DONE`) and `uptime_ms`.

## 11. Telemetry

### TELEMETRY (fast): every 200 ms

```
{"type":"TELEMETRY","uptime_ms":1400,"state":"DRIVE_UNAVAILABLE","block_reason":"MOTOR_PWM_UNAVAILABLE",
 "last_seq":1,"last_reject":"NONE","left_cmd":0,"right_cmd":0,"left_applied":0,"right_applied":0,
 "front_left_cm":150.0,"front_right_cm":150.0,"front_valid":true,"front_obstacle":false,
 "front_warning":false,"rear_mm":[null,null,null],"rear_available":false,"rear_obstacle":false,
 "rear_sensor_fault":false,"forward_blocked":false,"reverse_blocked":false,"safety_stop":false,
 "command_age_ms":1356,"command_timeout":false,"motor_drive_available":false}*XXXX
```

| Key | Meaning |
|---|---|
| `state`, `block_reason` | recomputed state; why the gate is clamping (or `NONE`) |
| `last_seq`, `last_reject` | last ACKed seq (null after boot); last rejection reason |
| `left_cmd`/`right_cmd` | what the Pi requested |
| `left_applied`/`right_applied` | what the motor layer outputs (§9) |
| `front_*_cm` | filtered distance, one decimal, **uncalibrated**; `null` when not a valid measurement |
| `front_valid`, `front_obstacle`, `front_warning` | front sensing trustworthy / latched obstacle / advisory |
| `rear_mm` | three VL53L0X by **TCA channel** (not left/centre/right), `null` when invalid |
| `rear_available`, `rear_obstacle`, `rear_sensor_fault` | rear backend reachable / latched obstacle / policy fault |
| `forward_blocked`, `reverse_blocked`, `safety_stop` | gates; latched safety stop |
| `command_age_ms`, `command_timeout` | watchdog age; latch |
| `motor_drive_available` | false means no wheel can turn, whatever is sent |

### DIAG: one section every 400 ms, 100 ms after a fast frame, rotating FRONT → REAR → SYSTEM

| Section | Keys |
|---|---|
| `FRONT` | `front_left_valid`, `front_right_valid`, `front_left_health`, `front_right_health`, `front_health`, `front_closest_cm`, `sensor_map_verified`; debug: `front_*_raw_cm`, `front_*_samples`, `front_*_timeout_streak` |
| `REAR` | `rear_backend`, `rear_orientation_verified`, arrays `rear_sensor_valid` / `_status` / `_health` / `_obstacle`, `rear_health`, `rear_closest_mm`, `rear_warning`; debug: `rear_sensor_raw_mm`, `rear_fail_streak`, `rear_tca_channels` |
| `SYSTEM` | `proto`, `i2c_ready`, `tca_status`, `pca_status`, `tca_address_confirmed`, `pca_address_confirmed`, `motor_drive_status`, `motor_map_verified`, `command_ever_received`, `left_gated`, `right_gated`, `link{rx_ok, rx_empty, rx_bad_frame, rx_bad_crc, rx_bad_message, rx_too_long, rx_rejected, rx_duplicates, rx_stale, errors_suppressed, tx_overflows}` |

Debug keys are controlled by `TELEMETRY_INCLUDE_DEBUG` in `config.h`.

## 12. Boot, READY and resynchronisation

1. On reset the ROM prints plain text (e.g. `rst:0x1 (POWERON_RESET),boot:0x13 …`).
   The Pi discards every line that fails the CRC check.
2. The firmware sends `EVENT READY`, then `EVENT I2CSCAN` (boot scan), then begins
   telemetry.
3. **Pi startup procedure:**
   - send a lone `\n`, which clears any partial line in the ESP32's buffer;
   - wait for any valid frame;
   - take `last_seq` from TELEMETRY and continue from `last_seq + 1`, or from 1 if it is null;
   - send `PING`.

## 13. Watchdog interaction

The 2-second movement failsafe (`COMMAND_TIMEOUT_MS`) is unchanged. It is refreshed
**only** by a DRIVE, MOVE or STOP that passed every check in §7 and was executed with result
ACCEPTED or GATED. GATED refreshes it as in v1: the Pi is alive and talking.

It is **not** refreshed by any of these:

- invalid frames or bad CRCs;
- malformed content, or a bad seq, type or cmd;
- unknown commands or rejected commands;
- duplicate or stale seqs;
- PING, RESET, MOTORTEST or any diagnostic command.

When it trips, both sides are zeroed and `EVENT COMMAND_TIMEOUT` is sent once.

A running MOTORTEST is cancelled by any received non-empty line **except** a valid new
MOTORTEST (which replaces it) and a duplicate or stale frame.

## 14. Changes from v1

| v1 | v2 |
|---|---|
| bare JSON line | `PAYLOAD*CRC4` + LF |
| no message type; telemetry identified by `"state"` | `"type"` on every frame |
| no seq | `seq` required; echoed in every ACK |
| `{"ack":"DRIVE","accepted":…}` | `{"type":"ACK","seq":…,"cmd":"DRIVE","result":…,"reason":…}` |
| DRIVE with one side missing → other side 0 | `MISSING_FIELD` |
| out-of-range → clamped to ±255 | `OUT_OF_RANGE` |
| `150abc` → 150 | `MALFORMED_NUMBER` |
| MOVE without speed → 150 | `MISSING_FIELD` (except `dir:"S"`) |
| MOTORTEST aliases `ch`/`speed`, defaults, clamping | `motor`/`power`/`ms`/`onblocks` all required, range-checked |
| TCATEST/PCATEST `addr` | integer only (e.g. `112`) |
| single ~2.1 KB telemetry line every 200 ms | TELEMETRY + rotating DIAG |
| `left_applied` = gated value | `left_applied` = actual motor output; gated value in `DIAG SYSTEM left_gated` |
| `obstacle`, `sensor_fault`, `sensor_health`, `warning`, `reverse_guarded` | `front_obstacle`, `!front_valid`, `front_health`, `front_warning`, `rear_available` |
| `rear_sensor_N_*` flat keys | `rear_mm` (fast) and `rear_sensor_*` arrays (DIAG REAR) |
| legacy `front_sensor_1_cm`, `rear_sensing`, `rear_ir*` | removed (duplicates of the fields above) |
| `{"event":"TELEMETRY_OVERFLOW"}` | `ERROR TX_FRAME_OVERFLOW` |

## 15. Bandwidth (measured)

Measured on the host simulator, which runs the real formatting code; 10 bits per byte at
115200 baud.

| Frame | Bytes | Wire time |
|---|---|---|
| TELEMETRY, typical | 565 | 49.0 ms (24.5 % of its 200 ms slot) |
| TELEMETRY, worst case (every field at maximum width) | 621 | 53.9 ms (27.0 %) |
| DIAG FRONT / REAR / SYSTEM | 391 / 480 / ~500–535 | 33.9 / 41.7 / ~46 ms |
| ACK DRIVE (−255/−255) | 195 | 16.9 ms |
| ACK PING / STOP | 136 / 174 | 11.8 / 15.1 ms |
| ERROR (seq null) | 58 | 5.0 ms |
| ACK HWREPORT (largest diagnostic) | 1417 | 123 ms |
| ACK I2CSCAN, 3 devices / 16 devices (estimate) | 679 / ~1940 | 59 / ~168 ms |
| Command DRIVE / longest command (MOTORTEST) | 70–75 / 103 | 6.1–6.5 / 8.9 ms |

Steady-state ESP32 → Pi load: fast ≈ 2825 B/s + DIAG ≈ 1170 B/s ≈ **35 %** of the link
(measured 33–36 %), against ≈ **92 %** for the v1 telemetry line.

**ACK latency, estimated for a physical 115200 link in normal operation** (DRIVE
command, 70 B up and 195 B back):

| Component | Typical | Worst (normal operation) |
|---|---|---|
| command on the wire | 6 ms | 6.5 ms |
| wait for `commPoll()` (a front ping busy-waits up to 20 ms) | 1–2 ms | 20 ms |
| queued behind a frame already in the TX ring (fast and DIAG are never back to back) | 0 ms | 54 ms |
| ACK on the wire | 17 ms | 17 ms |
| **total** | **≈ 25 ms** | **≈ 97 ms** |

Diagnostic commands (HWREPORT, I2CSCAN) add their own frame time. These are estimates;
the physical number comes from `tests/test_link.py --port …` (it reports round-trip times).

## 16. Testing

```
python tests/run_host_tests.py              # unit tests + simulator suites (no hardware)
python tests/test_link.py --port /dev/ttyAMA0   # from the Pi 5, against the real ESP32
```

The simulator compiles the real sketch with only the Arduino core, Wire, the VL53L0X
library and the PCA9685 driver stubbed. The link suite refuses to run against real hardware
that reports `motor_drive_available:true` unless `--allow-motion` is given.
