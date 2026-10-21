# Mini-Project 1 — Handoff (Prags → An)

## What is complete

`mp1_controller/controller.py` is a full two-mode ACC ready for CARLA validation:

- required `Controller` API (`__init__`, `run_step`) unchanged;
- **CRUISING** when `d > 40` or `d` is infinite; **FOLLOWING** otherwise;
- PI cruise-speed tracking with strong pull-away so `|v - v_des| < 2` can be met within 6 s;
- IDM-style following (standstill gap + time headway + closing foresight);
- lead-speed estimate from successive distance samples;
- safety overlay (TTC + stopping-distance braking);
- creep acceleration so the ego does not stay stopped while the lead is still moving;
- all primary gains/thresholds as named module-level constants.

Do **not** rename `Controller`, change the signatures of `__init__` or `run_step`,
or depend on edits outside `controller.py` (only that file is submitted).

Local unit tests (no CARLA) live in `tests/test_controller_checkpoint.py`.

## An's task

1. Run the full **15** scenarios in CARLA.
2. Evaluate with the official STL evaluator.
3. If anything fails, tune **only the named constants** at the top of `controller.py`
   (prefer that over rewriting control structure).
4. Send back the final `controller.py` and the evaluator “Fraction satisfied” summary.

### Constants to tune first

| Constant | Typical effect |
|---|---|
| `STANDSTILL_GAP`, `TIME_HEADWAY` | larger → safer following, less collision risk |
| `ASSUMED_MAX_DECEL` | lower → earlier braking; higher → later braking |
| `KP_GAP`, `KV_REL`, `TTC_BRAKE` | following aggressiveness / emergency timing |
| `KP_SPEED`, `KI_SPEED` | cruise speed acquisition |
| `CREEP_ACCEL` | recovery if ego stops while lead is moving |
| `HARD_BRAKE_DISTANCE` | last-resort hard brake range |

See the docstring at the top of `controller.py` for a failure→knob map.

## Quick local checks (no CARLA)

From the project root `csci513-miniproject1/`:

```bash
python3 -m py_compile mp1_controller/controller.py
python3 -m unittest discover -s tests -v
```

These validate structure and basic decisions only. They do **not** predict the grade.

## Docker note (Additional Maps URL)

If `make build` fails downloading Additional Maps, ensure `Carla.Dockerfile` line 16 uses:

```text
https://downloads.carlasim.com/Linux/AdditionalMaps_0.9.15.tar.gz
```

(the old `carla-releases.s3.eu-west-3.amazonaws.com` link is dead).

## Install / refresh the controller on HPC

```bash
# if editing locally, copy onto Discovery:
cp mp1_controller/controller.py ~/csci513-miniproject1/mp1_controller/controller.py
cd ~/csci513-miniproject1
```

Editable install means no reinstall after controller edits (`pip install -e .` once).

## Start the USC Discovery experiment

Follow `HPC-Instructions.md`. Short form after `salloc` on a GPU node:

```bash
module load apptainer
source ~/.bashrc
singularity exec --nv /project2/jdeshmuk_786/carla-0.9.15_4.11.sif bash
source ~/.bashrc
conda activate ~/.conda/envs/carla
bash /home/carla/CarlaUE4.sh -nosound -vulkan -RenderOffScreen &
cd ~/csci513-miniproject1
```

Wait until CARLA is up before running simulations.

## Run checks in increasing scope

Clear or move stale logs so the evaluator does not mix old runs:

```bash
rm -rf logs vids
mkdir -p logs vids
```

Smoke test (one easy + one braking scenario):

```bash
python3 -m mp1_simulator test_data/scenario1_0.json test_data/scenario3_4.json
python3 -m mp1_evaluation logs/episode-scenario1_0.csv logs/episode-scenario3_4.csv
```

Full suite:

```bash
python3 -m mp1_simulator test_data/*
python3 -m mp1_evaluation logs/*
```

Target: `Fraction satisfied` of `1.0` for all five properties.
Record per-scenario failures before changing parameters. After every accepted
change, rerun all 15 scenarios.

## Failure-to-adjustment guide

| Failure | First evidence to inspect | Likely adjustment |
|---|---|---|
| `safeFollowing` | min `distance_to_lead` in CSV | raise `STANDSTILL_GAP` / `TIME_HEADWAY`; lower `ASSUMED_MAX_DECEL`; raise `KV_REL` |
| `forwardProgress` | any negative `ego_velocity` | avoid prolonged hard brake near zero (should already be gated) |
| `dontStopUnlessLeadStops` | ego≈0 while `lead_speed`>0 after t=3 s | raise `CREEP_ACCEL`; slightly lower `STANDSTILL_GAP` |
| `notFollowWhenFarAway` | `mode` while `d > 40` | keep `CRUISE_DISTANCE = 40`; do not report FOLLOWING when far |
| `reachTargetWhenNotFollowing` | time to reach ~18 m/s in cruise | raise `KP_SPEED` / pull-away; delay soft braking (`ASSUMED_MAX_DECEL`) |

## Final acceptance checklist

- [ ] All 15 scenario files ran without an exception
- [ ] Evaluator run only on fresh logs from the final controller
- [ ] All five aggregate fractions recorded
- [ ] Final controller rerun after the last parameter change
- [ ] Only `mp1_controller/controller.py` will be submitted
