"""Bounded navigation stops and per-fix off-route counting.

`Navigator` with repeated fixes, `ProgressMonitor` on a fake clock, then the
whole agent over a fake ESP32 (`Rig`) losing its GPS or getting nowhere.

Nothing here opens a serial port, starts a thread or touches hardware.
"""

import logging
import unittest

# First: importing robotx.state.robot_state (here via robotx.hardware.gps)
# before the agent hits a pre-existing robot_state <-> mission import cycle.
import robotx.application.agent  # noqa: F401
from robotx.hardware.gps import GpsReading, GPSStatus
from robotx.localization.position import Position
from robotx.navigation.navigator import NavigationConfig, NavigationState, NavigationStatus, Navigator
from robotx.navigation.progress import ProgressConfig, ProgressMonitor
from robotx.state.robot_state import OperatingMode
from tests.fixtures import esp32 as fx
from tests.unit.test_agent import HERE
from tests.unit.test_obstacle_recovery import TICK_S, AgentCase


def setUpModule():
    logging.getLogger("robotx").setLevel(logging.CRITICAL + 1)


CFG = ProgressConfig()
START = (51.5, -0.1)
END = (51.501, -0.1)               # ~111 m north
OFF = (51.5005, -0.0990)           # ~70 m east of the leg: outside a 25 m corridor


def fix(point, t):
    return Position(latitude=point[0], longitude=point[1], timestamp=t)


# --- off-route counting ---------------------------------------------------------


class TestOffRouteCountsFixesNotTicks(unittest.TestCase):
    def nav(self):
        n = Navigator(NavigationConfig(reroute_after_n=8))
        n.set_route([END])
        n.update(fix(START, 1.0))                  # the leg's start
        return n

    def test_one_bad_fix_seen_on_every_tick_is_one_count(self):
        n = self.nav()
        bad = fix(OFF, 2.0)
        for _ in range(30):                        # 3 s of 10 Hz ticks on one 1 Hz fix
            state = n.update(bad)
        self.assertIs(state.status, NavigationStatus.NAVIGATING)

    def test_consistently_off_route_fixes_still_reach_reroute_needed(self):
        n = self.nav()
        for i in range(8):
            for _ in range(10):
                state = n.update(fix(OFF, 2.0 + i))
        self.assertIs(state.status, NavigationStatus.REROUTE_NEEDED)

    def test_a_new_route_forgets_the_last_fix(self):
        n = self.nav()
        p = fix(START, 5.0)
        n.update(p)
        n.set_route([END])
        n.update(p)                                # same fix, new route: it is the leg start again
        self.assertEqual(n.planner.active_segment(), (START, END))


# --- the monitor on its own -----------------------------------------------------


def nav_state(status=NavigationStatus.NAVIGATING, *, distance=50.0, index=0, issue=None):
    return NavigationState(status=status, distance_to_target_m=distance, waypoint_index=index,
                           waypoints_total=3, position_issue=issue)


class TestProgressMonitor(unittest.TestCase):
    def setUp(self):
        self.clock = fx.FakeClock()
        self.mon = ProgressMonitor(CFG, clock=self.clock)

    def run_for(self, seconds, state, *, moving=False, active=True):
        verdicts = []
        for _ in range(int(round(seconds / TICK_S))):
            self.clock.advance(TICK_S)
            verdicts.append(self.mon.update(active=active, navigation=state, moving=moving))
        return verdicts

    def test_a_long_gps_hold_escalates_once_with_its_reason(self):
        v = self.run_for(CFG.hold_assist_after_s + 5, nav_state(NavigationStatus.NO_POSITION, distance=None))
        assists = [x for x in v if x.assist]
        self.assertEqual(len(assists), 1)
        self.assertIn("navigation: no usable GPS position", assists[0].reason)
        first = next(i for i, x in enumerate(v) if x.assist)
        self.assertGreaterEqual((first + 1) * TICK_S, CFG.hold_assist_after_s)

    def test_inaccurate_fix_and_off_route_have_their_own_reasons(self):
        inaccurate = nav_state(NavigationStatus.NO_POSITION, distance=None, issue="hAcc 9 m")
        v = self.run_for(CFG.hold_assist_after_s + 1, inaccurate)
        self.assertIn("not accurate enough", [x for x in v if x.assist][0].reason)
        self.mon.reset()
        v = self.run_for(CFG.hold_assist_after_s + 1, nav_state(NavigationStatus.REROUTE_NEEDED))
        self.assertIn("off route", [x for x in v if x.assist][0].reason)

    def test_a_hold_that_ends_sooner_is_forgotten(self):
        self.run_for(CFG.hold_assist_after_s - 5, nav_state(NavigationStatus.NO_POSITION, distance=None))
        self.run_for(1.0, nav_state())             # the fix came back
        v = self.run_for(CFG.hold_assist_after_s - 5, nav_state(NavigationStatus.NO_POSITION, distance=None))
        self.assertFalse(any(x.assist for x in v))

    def test_paused_time_does_not_count(self):
        hold = nav_state(NavigationStatus.NO_POSITION, distance=None)
        self.run_for(CFG.hold_assist_after_s - 5, hold)
        self.run_for(600, hold, active=False)
        self.assertFalse(any(x.assist for x in self.run_for(4.0, hold)))

    def test_driving_without_getting_closer_escalates(self):
        v = self.run_for(CFG.no_progress_after_s + 5, nav_state(distance=50.0), moving=True)
        assists = [x for x in v if x.assist]
        self.assertEqual(len(assists), 1)
        self.assertIn("no progress toward waypoint 1/3", assists[0].reason)

    def test_time_stopped_is_not_a_stall(self):
        v = self.run_for(CFG.no_progress_after_s * 3, nav_state(distance=50.0), moving=False)
        self.assertFalse(any(x.assist for x in v))

    def test_gps_wander_is_not_progress_but_real_closing_is(self):
        s = self.mon
        wander = [50.0, 48.5, 51.0, 48.0, 50.5]    # never 3 m better than the best
        elapsed, i = 0.0, 0
        assisted = False
        while elapsed < CFG.no_progress_after_s + 2:
            self.clock.advance(TICK_S)
            elapsed += TICK_S
            assisted |= s.update(active=True, navigation=nav_state(distance=wander[i % 5]), moving=True).assist
            i += 1
        self.assertTrue(assisted)

        s.reset()
        distance, elapsed = 50.0, 0.0
        while elapsed < CFG.no_progress_after_s * 2:
            self.clock.advance(TICK_S)
            elapsed += TICK_S
            distance -= 0.1                          # 0.5 m/s closing
            self.assertFalse(s.update(active=True, navigation=nav_state(distance=distance), moving=True).assist)

    def test_reaching_the_next_waypoint_is_progress(self):
        self.run_for(CFG.no_progress_after_s - 5, nav_state(distance=50.0), moving=True)
        v = self.run_for(CFG.no_progress_after_s - 5, nav_state(distance=60.0, index=1), moving=True)
        self.assertFalse(any(x.assist for x in v))


# --- the agent over a fake ESP32 ------------------------------------------------


class TestAgentBoundedStops(AgentCase):
    def setUp(self):
        super().setUp()
        self.rig.agent.progress.clock = self.rig.clock

    def lose_gps(self, status=GPSStatus.NO_FIX):
        self.rig.agent.gps.reading = GpsReading(status=status)

    def test_lost_gps_stops_at_once_and_pauses_with_its_reason(self):
        rig = self.rig
        self.lose_gps()
        mark = rig.mark()
        snaps = rig.run(CFG.hold_assist_after_s + 1.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE without a position")
        self.assertTrue(all(s.motion_intent.is_stop for s in snaps))
        self.assertIs(snaps[0].mode, OperatingMode.AUTO)          # an ordinary stop at first
        self.assertIs(snaps[-1].mode, OperatingMode.PAUSED)
        self.assertIn("navigation: no usable GPS position", snaps[-1].last_error)

        # The fix comes back: nothing resumes on its own.
        rig.agent.gps.set_fix(*HERE)
        snaps = rig.run(3.0)
        self.assertTrue(all(s.mode is OperatingMode.PAUSED for s in snaps))
        self.assertEqual(rig.drives(mark), [])

        rig.agent.resume_mission("test")
        rig.run(1.0)
        self.assertTrue(rig.drives(mark), "RESUME with a valid fix never drove")

    def test_a_short_gps_dropout_resumes_on_the_valid_fix(self):
        rig = self.rig
        self.lose_gps(GPSStatus.STALE)
        mark = rig.mark()
        rig.run(5.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE on a stale position")
        rig.agent.gps.set_fix(*HERE)
        snaps = rig.run(1.0)
        self.assertTrue(rig.drives(mark))
        self.assertIs(snaps[-1].mode, OperatingMode.AUTO)

    def test_driving_that_gets_nowhere_pauses_and_stops_the_wheels(self):
        rig = self.rig
        # The fake receiver reports the same place however long the rover drives.
        snaps = rig.run(CFG.no_progress_after_s + 2.0)
        paused = next(i for i, s in enumerate(snaps) if s.mode is OperatingMode.PAUSED)
        # Driving began in Rig.start(), 1.0 s before this run, on the same clock.
        driven_s = 1.0 + (paused + 1) * TICK_S
        self.assertGreaterEqual(driven_s, CFG.no_progress_after_s - 2 * TICK_S)
        self.assertLessEqual(driven_s, CFG.no_progress_after_s + 2 * TICK_S)
        self.assertIn("no progress", snaps[paused].last_error)
        self.assertTrue(snaps[paused].motion_intent.is_stop)
        mark = rig.mark()
        rig.run(3.0)
        self.assertEqual(rig.drives(mark), [], "DRIVE after the stall pause")
        self.assertEqual(rig.port.command_names()[-1], "STOP")

    def test_a_controller_pause_says_so(self):
        rig = self.rig
        rig.fields.update(safety_stop=True)
        snap = rig.run(1.0)[-1]
        self.assertIs(snap.mode, OperatingMode.PAUSED)
        self.assertTrue(snap.last_error.startswith("controller: "), snap.last_error)


if __name__ == "__main__":
    unittest.main()
