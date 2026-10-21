"""Local checks for the ACC controller checkpoint; CARLA is not required."""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import types
import unittest
from enum import Enum
from typing import NamedTuple


class Mode(Enum):
    CRUISING = 0
    FOLLOWING = 1


class Observation(NamedTuple):
    ego_velocity: float
    desired_speed: float
    distance_to_lead: float


simulator_stub = types.ModuleType("mp1_simulator.simulator")
simulator_stub.Mode = Mode
simulator_stub.Observation = Observation
package_stub = types.ModuleType("mp1_simulator")
sys.modules["mp1_simulator"] = package_stub
sys.modules["mp1_simulator.simulator"] = simulator_stub

controller_path = (
    pathlib.Path(__file__).resolve().parents[1] / "mp1_controller" / "controller.py"
)
spec = importlib.util.spec_from_file_location("checkpoint_controller", controller_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)
Controller = module.Controller


class ControllerCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller(distance_threshold=4.0)

    def test_far_lead_cruises_and_accelerates_hard(self):
        action, mode = self.controller.run_step(Observation(0.0, 20.0, 100.0))
        self.assertEqual(mode, Mode.CRUISING)
        self.assertEqual(action, 10.0)

    def test_close_lead_enters_following(self):
        _, mode = self.controller.run_step(Observation(10.0, 20.0, 30.0))
        self.assertEqual(mode, Mode.FOLLOWING)

    def test_distance_over_40_immediately_cruises(self):
        self.controller.run_step(Observation(10.0, 20.0, 30.0))
        _, mode = self.controller.run_step(Observation(10.0, 20.0, 40.1))
        self.assertEqual(mode, Mode.CRUISING)

    def test_exactly_40_is_following(self):
        _, mode = self.controller.run_step(Observation(10.0, 20.0, 40.0))
        self.assertEqual(mode, Mode.FOLLOWING)

    def test_infinite_distance_cruises_and_clears_history(self):
        self.controller.run_step(Observation(10.0, 20.0, 30.0))
        _, mode = self.controller.run_step(Observation(10.0, 20.0, math.inf))
        self.assertEqual(mode, Mode.CRUISING)
        self.assertIsNone(self.controller._prev_distance)

    def test_output_is_always_bounded(self):
        for obs in (
            Observation(0.0, 200.0, math.inf),
            Observation(100.0, 0.0, math.inf),
            Observation(30.0, 20.0, 5.0),
            Observation(0.0, 20.0, 0.0),
        ):
            action, _ = self.controller.run_step(obs)
            self.assertGreaterEqual(action, -10.0)
            self.assertLessEqual(action, 10.0)

    def test_emergency_gap_commands_hard_brake(self):
        # Two steps so lead-speed estimate can initialize.
        self.controller.run_step(Observation(15.0, 20.0, 20.0))
        action, mode = self.controller.run_step(Observation(15.0, 20.0, 8.0))
        self.assertEqual(mode, Mode.FOLLOWING)
        self.assertEqual(action, -10.0)

    def test_creep_when_stopped_with_moving_lead_and_healthy_gap(self):
        # Seed a positive lead-speed estimate via increasing gap while ego is slow,
        # then hold ego near stop with a comfortable gap.
        self.controller.run_step(Observation(0.2, 20.0, 20.0))
        self.controller.run_step(Observation(0.2, 20.0, 22.0))  # gap opening
        action, mode = self.controller.run_step(Observation(0.05, 20.0, 25.0))
        self.assertEqual(mode, Mode.FOLLOWING)
        self.assertGreater(action, 0.0)

    def test_cruise_holds_near_target_when_no_lead(self):
        # No lead in lane → pure speed hold (avoid finite-d safety overlay).
        action, mode = self.controller.run_step(Observation(20.0, 20.0, math.inf))
        self.assertEqual(mode, Mode.CRUISING)
        self.assertGreaterEqual(action, -1.0)
        self.assertLessEqual(action, 1.0)


class OfflineKinematicSmokeTests(unittest.TestCase):
    """Light closed-loop smoke tests without CARLA (point-mass plant)."""

    DT = 0.1

    def _plant(self, u: float) -> float:
        if u >= 0:
            return 5.5 * (u / 10.0)
        return -8.0 * (-u / 20.0)

    def _simulate(self, lead_accel_fn, d0: float, T: float = 20.0):
        ctrl = Controller(distance_threshold=4.0)
        ego_v, lead_v, d = 0.0, 0.0, float(d0)
        t = 0.0
        rows = []
        for _ in range(int(T / self.DT) + 1):
            u, mode = ctrl.run_step(Observation(ego_v, 20.0, d))
            rows.append(
                {
                    "t": t,
                    "v": ego_v,
                    "d": d,
                    "v_lead": lead_v,
                    "mode": mode.value,
                }
            )
            ego_v = max(0.0, ego_v + self._plant(u) * self.DT)
            lead_v = max(0.0, lead_v + lead_accel_fn(t) * self.DT)
            d = max(0.0, d + (lead_v - ego_v) * self.DT)
            t += self.DT
        return rows

    def test_coast_scenario_stays_safe_and_reaches_band(self):
        rows = self._simulate(lambda t: 2.0 if t < 8 else 0.0, d0=100.0)
        self.assertGreaterEqual(min(r["d"] for r in rows), 4.0)
        self.assertTrue(any(abs(r["v"] - 20.0) < 2.0 for r in rows[:61]))

    def test_hard_brake_scenario_stays_safe(self):
        def lead(t):
            if t < 3:
                return 3.0
            if t < 10:
                return -4.0
            return 0.0

        rows = self._simulate(lead, d0=68.0)
        self.assertGreaterEqual(min(r["d"] for r in rows), 4.0)


if __name__ == "__main__":
    unittest.main()
