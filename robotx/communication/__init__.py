"""The backend boundary: everything that knows a FalconAut server exists.

    protocol.py      the wire contract -- event names, payloads, validation
    commands.py      inbound commands -> agent mission intents, idempotently
    backend_link.py  the one Socket.IO client: lifecycle, rates, dispatch

Nothing outside this package emits or parses a backend message, and nothing
inside it imports hardware, a motor driver or GPIO. The future Pi -> ESP32 link
is a separate boundary and must stay one: the backend must never become able to
reach the motors through this package.

Importing `backend_link` pulls in `socketio`. Import it lazily, as
`RobotAgent._start_backend_link` does, so a standalone agent with no backend
configured never loads the library at all.
"""
