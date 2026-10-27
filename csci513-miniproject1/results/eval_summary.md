# MP1 official evaluator results (CARLA)

## Current controller (2026-10-25, An)

**Host:** local workstation, RTX 4070, CARLA 0.9.15 + AdditionalMaps (Town06)  
**Python:** 3.8 conda env, `pip install -e .`  
**Controller:** `mp1_controller/controller.py` (working tree, see "What changed")

### Aggregate (all 15 scenarios, stock `python3 -m mp1_simulator test_data/*`)

| Requirement | Fraction satisfied |
|---|---|
| `safeFollowing` | **1.0** |
| `forwardProgress` | **1.0** |
| `dontStopUnlessLeadStops` | **1.0** |
| `notFollowWhenFarAway` | **1.0** |
| `reachTargetWhenNotFollowing` | **1.0** |

Controller score under the course rubric: **100%**. 75/75 checks `sat`.

The simulator picks the ego blueprint at random from `vehicle.lincoln.*`
(`mkz_2017` or `mkz_2020`), so the grader may get either. Both were run
(pinned via a test-only wrapper) and both score 1.0 on all five.

### Margins per scenario

"Reach wait" is the longest time, from any CRUISING sample in [0, 14] s, until
`|v - 20| < 2` or FOLLOWING; the STL limit is 6 s. 3.9 s is the floor: that is
how long full throttle takes from rest to 18 m/s.

| Scenario | min d (m) 2020 / 2017 | reach wait (s) 2020 / 2017 | min ego v while lead moving (m/s) | before: min d / reach wait |
|---|---|---|---|---|
| scenario1_0 | 29.6 / 29.5 | 3.9 / 3.9 | 7.8 | 30.4 / 4.9 |
| scenario1_1 | 30.2 / 30.1 | 3.9 / 3.9 | 6.9 | 36.1 / 4.7 |
| scenario1_2 | 25.2 / 25.3 | 3.9 / 3.9 | 4.6 | 26.1 / 4.1 |
| scenario1_3 | 32.0 / 32.0 | 3.9 / 3.9 | 8.0 | 35.1 / 5.1 |
| scenario1_4 | 30.0 / 30.0 | 3.9 / 3.9 | 7.7 | 32.0 / 4.9 |
| scenario3_0 | 18.0 / 18.0 | 3.9 / 3.9 | 14.7 | 18.7 / 3.9 |
| scenario3_1 | 20.3 / 20.3 | 4.3 / 4.3 | 5.5 | 19.4 / 3.9 |
| scenario3_2 | 19.1 / 18.9 | 3.9 / 3.9 | 14.7 | 22.0 / 3.9 |
| scenario3_3 | 19.6 / 18.8 | 4.6 / 4.6 | 5.6 | 19.5 / 3.9 |
| scenario3_4 | 18.2 / 17.5 | 2.8 / 2.8 | - (lead stopped at 3 s) | 18.5 / 2.8 |
| scenario3_5 | 17.5 / 17.2 | 3.0 / 3.0 | 5.0 | 17.6 / 3.0 |
| scenario3_6 | 20.5 / 20.3 | 3.9 / 3.9 | 7.1 | 21.5 / 3.9 |
| scenario3_7 | 18.5 / 18.4 | 3.9 / 3.9 | 11.9 | 18.8 / 3.9 |
| scenario3_8 | 19.7 / 19.7 | 3.9 / 3.9 | 7.6 | 21.4 / 3.9 |
| scenario3_9 | 17.9 / 17.8 | 4.2 / 4.1 | - (lead stopped at 3 s) | 9.5 / 3.9 |

Worst case: min d 17.2 m (was 9.1 m), reach wait 4.6 s (was 5.1 s),
`notFollowWhenFarAway` wait 0.1 s of 2 s.

### Why the 2026-10-21 controller was changed

It scored 1.0 on most runs, but:

- The very first run here (mkz_2020, fresh Town06 load) failed
  `reachTargetWhenNotFollowing` on `scenario1_0`. Later runs passed, so the
  margin, not the logic, was the problem.
- Cause: once v hit 18 m/s, the stopping-distance brake engaged while still
  CRUISING at d ≈ 70 m. The ego then crawled at ~12 m/s through d = 40-56 m
  for ~5 s, under the speed band but not yet FOLLOWING (wait 4.9-5.1 s of 6).
- Before reaching the band it ignored the lead entirely (TTC 1.6 s only), so
  in `scenario3_9` it charged a stopped lead and got to 9.1 m.
- With 0.2-0.3 s of extra actuation delay it collided (`safeFollowing` 14/15)
  and stopped behind a moving lead (`dontStopUnlessLeadStops` 13/15).

### What changed (`controller.py` only)

1. **CRUISING approach speed limit** (`_approach_speed_limit`): the ego tracks
   v_des but rides just under the smaller of (a) the speed that still settles
   `APPROACH_GAP` behind a lead holding its speed and (b) the speed that still
   stops `STOP_GAP` short of a lead braking to a stop (RSS-style). Planned
   decels are below what CARLA delivers. The ego holds the band as long as is
   safe, then brakes late and firmly into FOLLOWING range. This replaces the
   CRUISING-mode TTC 3.8 s / stopping-distance brake and `_cruise_speed_reached`.
2. **Low-speed coasting** (`COAST_*`): in CARLA, any brake held ~1 s below
   ~8 m/s locks the car to a full stop (even u = -0.5), while u = 0 still
   sheds ~2-3 m/s² of drag. Behind a moving lead, FOLLOWING now coasts instead
   of braking whenever drag alone keeps the gap above 8 m.
3. FOLLOWING-mode gap control, creep and hard-brake logic are unchanged.

Measured CARLA dynamics (both Lincolns): full throttle 0 → 18 m/s in 3.9 s;
u = -10 stops from 20 m/s in ~40 m (~5 m/s² after ~0.3 s lag); coasting
~2.5 m/s².

### Robustness sweep (mkz_2020, perturbation injected in a test-only wrapper)

| Perturbation | Before (fractions) | After (fractions) | After: min d / reach wait |
|---|---|---|---|
| none | 1, 1, 1, 1, 1 | 1, 1, 1, 1, 1 | 17.5 m / 4.6 s |
| +0.1 s actuation delay | 1, 1, 1, 1, 1 (min d 7.1 m) | 1, 1, 1, 1, 1 | 17.3 m / 4.7 s |
| +0.2 s actuation delay | 0.93, 1, 1, 1, 1 (collision) | 1, 1, 1, 1, 1 | 16.8 m / 4.3 s |
| +0.3 s actuation delay | 0.93, 1, 0.87, 1, 1 (collision) | 1, 1, 1, 1, 1 | 13.6 m / 4.4 s |
| command × 0.9 | 1, 1, 1, 1, 1 | 1, 1, 1, 1, 1 | 18.0 m / 5.0 s |
| command × 1.1 | 1, 1, 1, 1, 1 (min d 9.5 m) | 1, 1, 1, 1, 1 | 17.2 m / 4.6 s |
| command × 0.8 | 1, 1, 1, 1, 1 | 1, 1, 1, 1, 0.93 | 18.2 m / 6.6 s |

Order: safeFollowing, forwardProgress, dontStopUnlessLeadStops,
notFollowWhenFarAway, reachTargetWhenNotFollowing. The × 0.8 case (20% weaker
throttle than the real simulator) is the only miss.

### How this was produced

```bash
# CARLA server (local, headless)
~/CARLA_0.9.15/CarlaUE4.sh -RenderOffScreen -nosound &
# client
rm -rf logs vids
python3 -m mp1_simulator test_data/*
python3 -m mp1_evaluation logs/*
python3 -m unittest discover -s tests -v   # 11/11 pass
```

---

## Earlier record (2026-10-21, Prags)

USC Discovery (`d13-04`, Tesla V100), CARLA 0.9.15 via Singularity
`/project2/jdeshmuk_786/carla-0.9.15_4.11.sif`, previous controller: 1.0 on all
five requirements for all 15 scenarios. See above for why that controller was
replaced.

Raw CSV traces (`logs/`) and videos (`vids/`) are not committed; only
`mp1_controller/controller.py` is submitted.
