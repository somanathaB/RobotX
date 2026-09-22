# Socket.IO architecture on the Pi

How the backend boundary is built, and why it is built this way. The wire
contract itself is in [ROBOT_BACKEND_PROTOCOL.md](ROBOT_BACKEND_PROTOCOL.md).

---

## 1. Where the boundary sits

```
   camera ─► perception ─┐
                         ├─► RobotState ──► RobotSnapshot ──┐
   GPS ─► localization ──┤        ▲                         │
                         │        │                         ▼
   navigation ───────────┤        │                  BackendLink
                         │        │                    ▲     │
   decision ─► MotionIntent       │                    │     │  Socket.IO
                                  │                    │     ▼
                        mission methods ◄──── CommandExecutor ── backend
                     (stop/pause/resume/return)
```

`BackendLink` reads an immutable `RobotSnapshot` and calls four mission
methods. That is the **entire** coupling between communication and the robot.

It is one direction of data and one narrow direction of control, which is what
keeps the future Pi→ESP32 link independent: neither transport knows the other
exists, and replacing either touches no code in the other.

---

## 2. Modules

| File | Responsibility |
|---|---|
| `communication/protocol.py` | Event names (as data), payload builders, inbound validation, redaction. The only module that knows the wire format |
| `communication/commands.py` | Validated command → agent mission intent. Idempotency, refusals |
| `communication/backend_link.py` | The one Socket.IO client: connect loop, backoff, rates, dispatch, state reporting |

There is exactly **one** Socket.IO client in the repository. The previous
`communication/socket_client.py` was removed; keeping two clients would mean
two connection lifecycles and two versions of "is the robot online".

### What communication may not import

Enforced by a test that walks the AST of every module in the package: no
`RPi`, `gpiozero`, `serial`, `picamera2`, `cv2`, no `robotx.hardware.*`, no
`robotx.perception.*`.

That test has already paid for itself — it caught `protocol.py` importing
`GPSStatus` from `robotx.hardware.gps` purely to name an enum value. The
import was replaced with `GpsReading.has_fix`.

---

## 3. Why the Pi drives its own reconnection

`socketio.AsyncClient(reconnection=False)`.

The library can reconnect by itself, but its retries are invisible to the rest
of the agent: state would sit at `CONNECTED` while the library quietly failed
in a loop. Since a dashboard's "online" indicator is downstream of that state,
the link owns its connect loop so that every attempt, failure, classification
and backoff is observable in `RobotState` and on `/backend`.

### Backoff

Exponential from `backoff_initial_s`, capped at `backoff_max_s`, multiplied by
a random factor in `[0.5, 1.0]`. The jitter prevents a fleet from reconnecting
in lockstep and re-crashing the backend it was waiting for.

A refusal by the server uses `backoff_rejected_s` (60 s) instead: a rejected
credential will be rejected again one second later, so retrying at network
speed only loads a server that has already said no.

Classification is textual, because python-socketio gives no structured code.
Measured against 5.11.4:

| Client exception text | Meaning | Backoff |
|---|---|---|
| `One or more namespaces failed to connect` | Transport reached the server; the server refused | Rejected (long) |
| `Cannot connect to host …` | Network/DNS failure | Normal |

Anything unrecognized is treated as a **network** failure. That is the safe
direction to be wrong in: the cost is a few extra connect attempts, whereas
the reverse error would leave a robot slow to find a healthy backend.

---

## 4. Rate control lives in the link

Telemetry and status rates are enforced inside `BackendLink`'s publish loop,
not by callers. No part of the agent can flood the backend, because no part of
the agent can emit. The loop wakes every 250 ms and each channel decides from
its own elapsed time whether it is due.

The event channel adds two further limits: identical messages are deduplicated
for 30 s, and no more than one event per second is emitted at all.

Measured on this Pi (Camera Module 3 active, 2×120 s windows):

| | Agent alone | Agent + link | Delta |
|---|---|---|---|
| CPU (% of one core) | 14.0 | 14.0 | **0.0** |
| Loop period, median | 100.49 ms | 100.53 ms | +0.04 ms |
| Loop period, p95 | 101.07 ms | 101.26 ms | +0.19 ms |
| Perception processing | 4.55 ms | 4.89 ms | +0.34 ms |
| Max RSS | 122.6 MB | 127.6 MB | +5.0 MB |

Build + encode one telemetry payload: ~5.2 µs + ~7.6 µs, 154 bytes. At 1 Hz
that is immeasurable against a 10 Hz agent loop, which is the point.

---

## 5. Threading and the event loop

The link is pure asyncio and runs on the agent's existing loop. It starts no
threads.

Everything it does per event is bounded and non-blocking: payload building is
dictionary construction over an immutable snapshot, and the mission methods it
calls take a lock briefly and return. There is no file I/O, no network call and
no sleep on the command path, so a command cannot stall the loop that
perception and navigation share.

The camera and GPS keep their own threads, as before. The link never touches
them.

---

## 6. Startup and shutdown order

**Startup:** camera → perception → GPS → state → agent loop → **backend link
last**. The agent is fully operational before the backend is contacted, and
`start()` never blocks or fails on it. A robot that will not boot without a
dashboard cannot be recovered when the dashboard is what broke.

The `socketio` import is *inside* `RobotAgent._start_backend_link`, so with
`ROBOTX_SOCKET_ENABLED=0` the library is never loaded at all — the standalone
agent keeps the exact runtime footprint it had before this work. A test
asserts `'socketio' not in sys.modules` in that configuration.

**Shutdown:** backend link first, then the agent loop, then perception, GPS
and camera in reverse order of acquisition. The link goes first because it
reads state: a link still publishing while subsystems are torn down would
report a robot that is halfway gone as if it were running.

---

## 7. Observability

`GET /backend` on the Pi's local HTTP API:

```json
{
  "enabled": true, "status": "CONNECTED", "server": "http://backend:3000",
  "tls": false, "authenticated_client_side": true,
  "protocol": { "source": "PROVISIONAL", "provisional": true, "namespace": "/robot", ... },
  "integrated": false,
  "loss_policy": "pause",
  "stats": { "connects": 1, "telemetry_sent": 412, "telemetry_skipped": 8,
             "commands_received": 3, "acks_sent": 3, "unexpected_events": 0, ... }
}
```

`integrated` is the honest bottom line and is **false** whenever the binding is
provisional, however healthy the socket looks. A connected socket speaking
guessed event names is a connection, not an integration.

`unexpected_events` counts inbound events the Pi has no binding for. A non-zero
value is direct evidence the binding is wrong.

No credential appears in this output, in `/config`, or in any log line.

---

## 8. Testing

| Suite | What it covers | Count |
|---|---|---|
| `tests/unit/test_protocol.py` | Payloads, refusal to fabricate, inbound validation, redaction | 56 |
| `tests/unit/test_commands.py` | Execution, refusals, idempotency, the safety boundary | 25 |
| `tests/unit/test_backend_link.py` | Lifecycle, backoff, rates, dispatch, honest reporting | 44 |
| `tests/unit/test_agent_backend_commands.py` | The real agent under each command | 19 |
| `tests/integration/test_socketio_transport.py` | **Real Socket.IO transport** | 25 |

The integration suite runs a real `socketio.AsyncServer` on a loopback port
and a real `socketio.AsyncClient` — genuine handshake, genuine JSON, nothing
faked on the client side. It covers handshake, credential delivery, server
refusal, telemetry, status, command round-trip, duplicates, disconnect,
reconnect, re-registration, and a backend that is absent at boot and appears
later.

What it does **not** prove: anything about FalconAut. The test server speaks
the Pi's own provisional binding, so a green run means "the transport and the
state machine are correct", not "the integration is done".

```bash
venv/bin/python -m unittest discover -s tests/unit -t .
venv/bin/python -m unittest discover -s tests/integration -t .
PYTHONPATH=. venv/bin/python tests/integration/soak_backend_link.py --seconds 120
```
