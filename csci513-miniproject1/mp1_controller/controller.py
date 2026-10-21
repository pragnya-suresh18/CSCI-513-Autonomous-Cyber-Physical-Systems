"""Adaptive cruise control for Mini-Project 1.

Design
------
- Mode: CRUISING if d > 40 or d is inf; else FOLLOWING (matches STL).
- Cruising: PI speed tracking to v_des with a strong pull-away to hit the
  |v - v_des| < 2 band within 6 s.
- Following: IDM-style gap + relative-speed regulation, capped by v_des.
- Lead speed is estimated from successive distance samples.
- Safety overlay: TTC + stopping-distance braking; creep accel if the lead
  is moving and we are nearly stopped with a healthy gap.

Tuning guide (for final 15-scenario pass)
-----------------------------------------
- Collisions / safeFollowing fails
    raise STANDSTILL_GAP or TIME_HEADWAY, or lower ASSUMED_MAX_DECEL,
    or raise KV_REL / TTC_BRAKE.
- reachTargetWhenNotFollowing fails
    raise KP_SPEED / pull-away (err thresholds in _speed_control), or
    lower ASSUMED_MAX_DECEL sensitivity so soft braking starts later.
- dontStopUnlessLeadStops fails
    raise CREEP_ACCEL, or lower STANDSTILL_GAP slightly.
- notFollowWhenFarAway fails
    should not happen with CRUISE_DISTANCE = 40; check mode logic.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from mp1_simulator.simulator import Mode, Observation


# ---------------------------------------------------------------------------
# Tunable constants (primary knobs for final evaluation tuning)
# ---------------------------------------------------------------------------

DT = 0.1  # must match simulator

# Mode: STL requires CRUISING within 2 s whenever d > 40.
CRUISE_DISTANCE = 40.0

# Spacing (center-to-center). Hard STL floor is 4 m; stay well above it.
STANDSTILL_GAP = 14.0
TIME_HEADWAY = 1.5

# Assumed max deceleration (m/s^2) when command is -10.
# Plan a bit pessimistically so we brake early enough for CARLA's
# brake = |u|/20 mapping (max pedal 0.5).
ASSUMED_MAX_DECEL = 4.0

# Speed tracking — must reach |v - v_des| < 2 within 6 s while CRUISING.
KP_SPEED = 3.0
KI_SPEED = 0.3
SPEED_I_LIMIT = 3.0

# Gap / relative-speed regulation
KP_GAP = 1.2
KV_REL = 3.5
IDM_A = 5.0
IDM_B = 7.0

# Safety overlays
HARD_BRAKE_DISTANCE = 10.0
TTC_BRAKE = 3.8
MIN_MOVING_SPEED = 0.15
LEAD_MOVING_SPEED = 0.25
CREEP_ACCEL = 3.5

MAX_ACC = 10.0
MIN_ACC = -10.0


class Controller:
    """Two-mode ACC: cruise to v_des when far; follow a safe gap when close."""

    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold

        self._mode = Mode.CRUISING
        self._prev_distance: Optional[float] = None
        self._speed_integral = 0.0
        self._prev_lead_speed = 0.0
        # True once inside |v - v_des| < 2 during this cruising segment.
        # Gates when the fuller stopping-distance brake may engage.
        self._cruise_speed_reached = False

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        ego_velocity = float(obs.ego_velocity)
        desired_speed = float(obs.desired_speed)
        dist_to_lead = float(obs.distance_to_lead)

        lead_speed = self._estimate_lead_speed(ego_velocity, dist_to_lead)
        mode = self._select_mode(dist_to_lead)

        if mode == Mode.CRUISING:
            if abs(ego_velocity - desired_speed) < 2.0:
                self._cruise_speed_reached = True
            accel = self._cruise_control(ego_velocity, desired_speed)
        else:
            self._cruise_speed_reached = False
            accel = self._follow_control(
                ego_velocity, desired_speed, dist_to_lead, lead_speed
            )

        accel = self._safety_overlay(
            accel, ego_velocity, dist_to_lead, lead_speed, mode
        )
        accel = float(min(MAX_ACC, max(MIN_ACC, accel)))

        self._mode = mode
        self._prev_distance = dist_to_lead if math.isfinite(dist_to_lead) else None
        self._prev_lead_speed = lead_speed
        return accel, mode

    def _select_mode(self, distance: float) -> Mode:
        if not math.isfinite(distance) or distance > CRUISE_DISTANCE:
            return Mode.CRUISING
        return Mode.FOLLOWING

    def _estimate_lead_speed(self, ego_velocity: float, distance: float) -> float:
        if (
            self._prev_distance is None
            or not math.isfinite(distance)
            or not math.isfinite(self._prev_distance)
        ):
            return max(0.0, self._prev_lead_speed)

        lead_speed = ego_velocity + (distance - self._prev_distance) / DT
        lead_speed = 0.6 * lead_speed + 0.4 * self._prev_lead_speed
        return max(0.0, lead_speed)

    def _desired_gap(self, ego_velocity: float, lead_speed: float) -> float:
        rel = ego_velocity - lead_speed
        foresight = ego_velocity * rel / (
            2.0 * math.sqrt(max(IDM_A * IDM_B, 1e-3))
        )
        return (
            max(self.distance_threshold + 6.0, STANDSTILL_GAP)
            + TIME_HEADWAY * max(ego_velocity, 0.0)
            + max(0.0, foresight)
        )

    def _speed_control(self, ego_velocity: float, speed_target: float) -> float:
        err = speed_target - ego_velocity
        self._speed_integral = min(
            SPEED_I_LIMIT,
            max(-SPEED_I_LIMIT, self._speed_integral + err * DT),
        )
        if abs(err) < 0.3:
            self._speed_integral *= 0.75

        accel = KP_SPEED * err + KI_SPEED * self._speed_integral
        # Hard pull-away until clearly inside the |v - v_des| < 2 band.
        if err > 1.8:
            accel = MAX_ACC
        elif err > 1.0:
            accel = max(accel, 8.0)
        return accel

    def _cruise_control(self, ego_velocity: float, desired_speed: float) -> float:
        # While CRUISING, track v_des. Gap regulation belongs in FOLLOWING.
        # Safety overlay still brakes if a collision is imminent. Tracking
        # v_des also ensures we close on a slower lead and cross into
        # FOLLOWING rather than hovering just above 40 m under the speed band.
        return self._speed_control(ego_velocity, desired_speed)

    def _gap_accel(
        self,
        ego_velocity: float,
        desired_speed: float,
        distance: float,
        lead_speed: float,
    ) -> float:
        d_des = self._desired_gap(ego_velocity, lead_speed)
        gap_error = distance - d_des
        rel_vel = lead_speed - ego_velocity
        accel = KP_GAP * gap_error + KV_REL * rel_vel
        cruise_cap = KP_SPEED * (desired_speed - ego_velocity) + 2.0
        return min(accel, cruise_cap)

    def _follow_control(
        self,
        ego_velocity: float,
        desired_speed: float,
        distance: float,
        lead_speed: float,
    ) -> float:
        self._speed_integral = 0.0

        if not math.isfinite(distance):
            return self._speed_control(ego_velocity, desired_speed)

        accel = self._gap_accel(
            ego_velocity, desired_speed, distance, lead_speed
        )

        if (
            lead_speed > LEAD_MOVING_SPEED
            and ego_velocity < MIN_MOVING_SPEED
            and distance > STANDSTILL_GAP
        ):
            accel = max(accel, CREEP_ACCEL)

        return accel

    def _safety_overlay(
        self,
        accel: float,
        ego_velocity: float,
        distance: float,
        lead_speed: float,
        mode: Mode,
    ) -> float:
        if not math.isfinite(distance):
            return accel

        if distance <= HARD_BRAKE_DISTANCE:
            return MIN_ACC

        closing = ego_velocity - lead_speed
        if closing > 0.3 and distance > 1e-3:
            ttc = distance / closing

            # Before cruise speed is reached, only intervene on very urgent TTC
            # so reachTargetWhenNotFollowing is not starved of acceleration.
            ttc_limit = 1.6 if not self._cruise_speed_reached else TTC_BRAKE
            if ttc < ttc_limit:
                urgency = (ttc_limit - ttc) / ttc_limit
                accel = min(accel, MIN_ACC * (0.45 + 0.55 * urgency))

            # Full stopping-distance brake after the cruise band has been hit
            # (or whenever we are already in FOLLOWING).
            if self._cruise_speed_reached or mode == Mode.FOLLOWING:
                gap = distance - STANDSTILL_GAP
                if gap < 0.5:
                    accel = MIN_ACC
                else:
                    needed_decel = (
                        ego_velocity * ego_velocity - lead_speed * lead_speed
                    ) / (2.0 * gap)
                    if needed_decel > ASSUMED_MAX_DECEL * 0.35:
                        brake_u = -MAX_ACC * min(
                            1.0, needed_decel / ASSUMED_MAX_DECEL
                        )
                        accel = min(accel, brake_u)

        if distance < max(self.distance_threshold + 4.0, 8.5):
            accel = min(accel, -8.0)

        # In FOLLOWING, if nearly stopped with a healthy gap and a moving lead,
        # do not keep braking (dontStopUnlessLeadStops).
        if (
            mode == Mode.FOLLOWING
            and ego_velocity < MIN_MOVING_SPEED
            and accel < 0.0
            and distance > STANDSTILL_GAP
        ):
            accel = max(accel, 0.0)

        return accel
