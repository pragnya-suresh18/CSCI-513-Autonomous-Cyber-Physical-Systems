# MP1 official evaluator results (Discovery + CARLA)

**Date:** 2026-10-21  
**Host:** USC Discovery (`d13-04`, Tesla V100)  
**CARLA:** 0.9.15 via Singularity  
`/project2/jdeshmuk_786/carla-0.9.15_4.11.sif`  
**Project path on HPC:**  
`~/CSCI-513-Autonomous-Cyber-Physical-Systems/csci513-miniproject1`  
**Controller:** `mp1_controller/controller.py` (repo `main`)

## Aggregate (all 15 scenarios)

| Requirement | Fraction satisfied |
|---|---|
| `safeFollowing` | **1.0** |
| `forwardProgress` | **1.0** |
| `dontStopUnlessLeadStops` | **1.0** |
| `notFollowWhenFarAway` | **1.0** |
| `reachTargetWhenNotFollowing` | **1.0** |

Controller score under the course rubric: **100%**  
(each requirement is 20%; all five are fully satisfied on all 15 scenarios).

## Per-scenario

Every scenario below reported `sat` on all five requirements:

- `scenario1_0` … `scenario1_4`
- `scenario3_0` … `scenario3_9`

## How this was produced

```bash
rm -rf logs vids && mkdir -p logs vids
python3 -m mp1_simulator test_data/*
python3 -m mp1_evaluation logs/*
```

Raw CSV traces and videos were left on HPC (`logs/`, `vids/`) and are gitignored.
They are not required for submission; only `mp1_controller/controller.py` is submitted.
