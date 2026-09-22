"""The authoritative robot state and the telemetry built from it.

`robot_state.RobotState` is the single place holding what the Pi knows; every
other module writes into it or reads a snapshot from it.
"""
