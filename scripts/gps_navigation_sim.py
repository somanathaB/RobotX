"""GPS navigation scenarios against a SIMULATED differential-drive rover.

    venv/bin/python -m scripts.gps_navigation_sim                    # scenario table
    venv/bin/python -m scripts.gps_navigation_sim --trace aligned    # one scenario, tick by tick
    venv/bin/python -m scripts.gps_navigation_sim --turn-rate 50 --track-m 0.45

Runs the Pi's real localization, navigation, decision, safety and progress code
in virtual time through `tests.fixtures.diff_drive_sim`. Nothing here opens a
port, binds a socket or reads /etc/robotx. The drivetrain and GPS numbers are
assumptions: a PASS is evidence about the Pi's navigation logic, never about
the physical rover.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from typing import Callable, Dict, List, Optional

from tests.fixtures.diff_drive_sim import (
    ClosedLoopNav, GpsModel, Physics, Window, offset, settings, straight_route,
)

START = (12.9700000, 77.5900000)


def scenarios(env: Dict[str, str], physics: Physics, gps: GpsModel) -> Dict[str, Callable[[], ClosedLoopNav]]:
    def sim(yaw: float, bearing: float, length: float = 30.0, **kw) -> ClosedLoopNav:
        return ClosedLoopNav(START, yaw, route=straight_route(START, bearing=bearing, length_m=length),
                             cfg=settings(**env), physics=kw.pop("physics", physics), gps=kw.pop("gps", gps), **kw)

    def with_(s: ClosedLoopNav, **attrs) -> ClosedLoopNav:
        for name, value in attrs.items():
            getattr(s, name).append(value) if isinstance(getattr(s, name), list) else setattr(s, name, value)
        return s

    corner = offset(START, north_m=25.0)
    return {
        "aligned": lambda: sim(0.0, 0.0),
        "target-right-60": lambda: sim(0.0, 60.0),
        "target-left-60": lambda: sim(0.0, 300.0),
        "target-right-90": lambda: sim(0.0, 90.0),
        "target-behind": lambda: sim(0.0, 180.0),
        "waypoints-L": lambda: ClosedLoopNav(START, 0.0, route=[offset(START, north_m=12.0), corner,
                                                               offset(corner, east_m=12.0), offset(corner, east_m=25.0)],
                                             cfg=settings(**env), physics=physics, gps=gps),
        "gps-outage-20s": lambda: with_(sim(0.0, 0.0, 40.0), gps_outages=Window(30.0, 50.0)),
        "gps-outage-long": lambda: with_(sim(0.0, 0.0, 40.0), gps_outages=Window(30.0, 300.0)),
        "gps-inaccurate-15s": lambda: with_(sim(0.0, 0.0, 40.0), gps_bad_acc=Window(20.0, 35.0, 6.0)),
        "gps-hacc-4.5m": lambda: sim(30.0, 0.0, 40.0, gps=GpsModel(h_acc_m=4.5, sigma_m=1.5)),
        "range-veto-15s": lambda: with_(sim(0.0, 0.0, 40.0), obstacles=Window(25.0, 40.0, 50.0)),
        "operator-stop": lambda: with_(sim(0.0, 0.0, 40.0), stop_at=30.0),
    }


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="SIMULATED closed-loop GPS navigation scenarios (no hardware).")
    p.add_argument("--trace", metavar="SCENARIO", help="print one scenario tick by tick (every second)")
    p.add_argument("--turn-rate", type=float, help="ROBOTX_NAV_TURN_RATE_DPS the Pi assumes")
    p.add_argument("--full-scale-mps", type=float, default=Physics.full_scale_mps, help="simulated side speed at DRIVE 255")
    p.add_argument("--track-m", type=float, default=Physics.track_m, help="simulated effective track width")
    p.add_argument("--h-acc", type=float, default=GpsModel.h_acc_m, help="simulated GPS stated accuracy (m)")
    p.add_argument("--json", action="store_true")
    args = p.parse_args(argv)
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)

    env = {"ROBOTX_NAV_TURN_RATE_DPS": str(args.turn_rate)} if args.turn_rate else {}
    physics = Physics(full_scale_mps=args.full_scale_mps, track_m=args.track_m)
    gps = GpsModel(h_acc_m=args.h_acc, sigma_m=max(0.35, args.h_acc * 0.35))
    table = scenarios(env, physics, gps)

    if args.trace:
        result = table[args.trace]().run(until_paused=args.trace != "operator-stop")
        print(f"{'t':>6} {'x_m':>6} {'y_m':>6} {'yaw':>5} {'L':>4} {'R':>4} {'heading':>13} "
              f"{'desired':>7} {'err':>6} {'dist':>5} wp  reason")
        for r in result.rows[::10]:
            heading = "-" if r.believed_heading is None else f"{r.believed_heading:.0f}"
            print(f"{r.t:6.1f} {r.x:6.1f} {r.y:6.1f} {r.yaw:5.0f} {r.left:4d} {r.right:4d} "
                  f"{r.heading_source[:9]:>9}{heading:>4} {r.desired or 0:7.0f} {r.est_error or 0:6.0f} "
                  f"{r.distance or 0:5.1f} {r.waypoint_index:2d}  {r.reason}")
        print(json.dumps(result.summary()))
        return 0

    rows = []
    for name, build in table.items():
        result = build().run(until_paused=name != "operator-stop", max_s=600.0 if name != "operator-stop" else 60.0)
        rows.append({"scenario": name, **result.summary()})
    if args.json:
        print(json.dumps({"assumptions": {"physics": physics.__dict__, "gps": gps.__dict__, "env": env},
                          "results": rows, "physical_movement": "NOT TESTED"}, indent=2))
        return 0
    print(f"SIMULATED drivetrain: full scale {physics.full_scale_mps} m/s, track {physics.track_m} m; "
          f"GPS h_acc {gps.h_acc_m} m; assumed turn rate {args.turn_rate or 90.0} deg/s")
    print(f"{'scenario':<20} {'outcome':<8} {'t_s':>6} {'path_m':>6} {'rot':>4} {'final_m':>7} {'rev':>3}  reason")
    for s in rows:
        print(f"{s['scenario']:<20} {s['outcome']:<8} {s['t_s']:6.1f} {s['path_m']:6.1f} {s['rotations']:4.1f} "
              f"{s['final_distance_m'] if s['final_distance_m'] is not None else '-':>7} "
              f"{s['reverse_or_pivot_drives']:3d}  {s['reason'][:70]}")
    print("NOT TESTED  physical wheel movement, real GPS heading, real turn rate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
