# Mini-Project 1 — Handoff (Prags → An)

## Status (read this first)

CARLA validation on USC Discovery is **done**.

- All **15** scenarios ran without errors.
- Official evaluator: **1.0** on all five requirements.
- Summary saved in [`results/eval_summary.md`](results/eval_summary.md).
- No constant tuning was required after the CARLA run.

**Submit only** `mp1_controller/controller.py`.

If you want to reproduce the run yourself, follow the Discovery recipe below
(this is exactly how Prags ran it). Local unit tests are optional sanity checks.

## What’s in the controller

`mp1_controller/controller.py` is a full two-mode ACC:

- required `Controller` API (`__init__`, `run_step`) unchanged
- **CRUISING** when `d > 40` or `d` is infinite; **FOLLOWING** otherwise
- PI cruise-speed tracking with strong pull-away for `|v - v_des| < 2` within 6 s
- IDM-style following (standstill gap + time headway + closing foresight)
- lead-speed estimate from successive distance samples
- safety overlay (TTC + stopping-distance braking)
- creep acceleration so ego does not stay stopped while the lead is moving
- tunable gains/thresholds as named module-level constants

Do **not** rename `Controller`, change `__init__` / `run_step` signatures, or
depend on edits outside `controller.py`.

## Repo layout you’ll use

Clone (or pull) this GitHub repo on Discovery:

```bash
cd ~
git clone https://github.com/pragnya-suresh18/CSCI-513-Autonomous-Cyber-Physical-Systems.git
# if already cloned:
cd ~/CSCI-513-Autonomous-Cyber-Physical-Systems && git pull
```

Project root on HPC:

```text
~/CSCI-513-Autonomous-Cyber-Physical-Systems/csci513-miniproject1
```

## An’s optional task

Only needed if you want to double-check before submit, or if the autograder
setup differs:

1. Reproduce the full 15-scenario CARLA run on Discovery (steps below).
2. Confirm `python3 -m mp1_evaluation logs/*` still prints 1.0 for all five.
3. If something fails on a fresh run, tune **only** the named constants at the
   top of `controller.py`, then rerun all 15.
4. Keep `results/eval_summary.md` in sync if scores change.

### Constants to touch only if a rerun fails

| Constant | Typical effect |
|---|---|
| `STANDSTILL_GAP`, `TIME_HEADWAY` | larger → safer following |
| `ASSUMED_MAX_DECEL` | lower → earlier braking |
| `KP_GAP`, `KV_REL`, `TTC_BRAKE` | following / emergency timing |
| `KP_SPEED`, `KI_SPEED` | cruise speed acquisition |
| `CREEP_ACCEL` | recovery if ego stops while lead moves |
| `HARD_BRAKE_DISTANCE` | last-resort hard brake range |

## Quick local checks (no CARLA, optional)

From `csci513-miniproject1/`:

```bash
python3 -m py_compile mp1_controller/controller.py
python3 -m unittest discover -s tests -v
```

## Docker note (only if building locally)

`Carla.Dockerfile` already uses the working Additional Maps URL:

```text
https://downloads.carlasim.com/Linux/AdditionalMaps_0.9.15.tar.gz
```

On Discovery we used the course Singularity image instead of Docker.

---

## Reproduce on USC Discovery (Prags’ working recipe)

Needs: Cisco USC VPN, Discovery access, course GPU account.

### 1) VPN + login

Connect Cisco AnyConnect to USC VPN, then:

```bash
ssh <netid>@discovery.usc.edu
```

Or OnDemand: https://ondemand.carc.usc.edu/

### 2) One-time conda setup (skip if `~/.conda/envs/carla` already exists)

```bash
module purge
module load conda
source ~/.bashrc

# if you see MultipleKeysError about auto_activate_base / auto_activate:
sed -i '/auto_activate_base/d' ~/.condarc

conda activate base
conda create -n carla python=3.8 conda -c conda-forge
~/.conda/envs/carla/condabin/conda init bash
source ~/.bashrc
conda activate ~/.conda/envs/carla

python --version    # expect Python 3.8.x
which python        # expect ~/.conda/envs/carla/bin/python
```

Important: always activate with the **full path**
`conda activate ~/.conda/envs/carla`.
The prompt may still say `(base)`. Trust `which python`.

### 3) Get / update the code

```bash
cd ~
git clone https://github.com/pragnya-suresh18/CSCI-513-Autonomous-Cyber-Physical-Systems.git
cd ~/CSCI-513-Autonomous-Cyber-Physical-Systems/csci513-miniproject1
```

### 4) GPU allocation

```bash
salloc --time=2:00:00 --cpus-per-task=8 --mem=32GB --account=jdeshmuk_786 --partition=gpu --gres=gpu:1
nvidia-smi
```

### 5) Singularity + CARLA

On the allocated node:

```bash
module load apptainer
source ~/.bashrc
conda activate ~/.conda/envs/carla
which python

singularity exec --nv /project2/jdeshmuk_786/carla-0.9.15_4.11.sif bash
```

Inside `Apptainer>`:

```bash
source ~/.bashrc
conda activate ~/.conda/envs/carla
which python

# first time in this container/env only:
pip install -U carla==0.9.15
cd ~/CSCI-513-Autonomous-Cyber-Physical-Systems/csci513-miniproject1
python3 -m pip install -U -e .

# start CARLA (chmod warning about read-only filesystem is normal)
bash /home/carla/CarlaUE4.sh -nosound -vulkan -RenderOffScreen &
```

Hit ENTER when the prompt returns. Check connection:

```bash
cd /home/carla/PythonAPI/util
python3 test_connection.py
# expect: CARLA 0.9.15 connected at 127.0.0.1:2000.
```

### 6) Smoke test, then full suite

```bash
cd ~/CSCI-513-Autonomous-Cyber-Physical-Systems/csci513-miniproject1
rm -rf logs vids
mkdir -p logs vids

# smoke
python3 -m mp1_simulator test_data/scenario1_0.json test_data/scenario3_4.json
python3 -m mp1_evaluation logs/episode-scenario1_0.csv logs/episode-scenario3_4.csv

# full 15
rm -rf logs vids && mkdir -p logs vids
python3 -m mp1_simulator test_data/*
python3 -m mp1_evaluation logs/*
```

Expected aggregate (already achieved on 2026-10-21):

```text
Fraction satisfied
Robustness for `safeFollowing`               = 1.0
Robustness for `forwardProgress`             = 1.0
Robustness for `dontStopUnlessLeadStops`     = 1.0
Robustness for `notFollowWhenFarAway`        = 1.0
Robustness for `reachTargetWhenNotFollowing` = 1.0
```

### 7) Leave the job

```bash
exit   # leave Apptainer
exit   # leave salloc / GPU node
```

`salloc` expires after the requested time (2 hours above). Files on `$HOME` stay.

---

## Failure-to-adjustment guide (only if a rerun regresses)

| Failure | Likely adjustment |
|---|---|
| `safeFollowing` | raise `STANDSTILL_GAP` / `TIME_HEADWAY`; lower `ASSUMED_MAX_DECEL`; raise `KV_REL` |
| `forwardProgress` | avoid prolonged hard brake near zero (already gated) |
| `dontStopUnlessLeadStops` | raise `CREEP_ACCEL`; slightly lower `STANDSTILL_GAP` |
| `notFollowWhenFarAway` | keep `CRUISE_DISTANCE = 40` |
| `reachTargetWhenNotFollowing` | raise `KP_SPEED` / pull-away; delay soft braking |

## Acceptance checklist

- [x] All 15 scenarios ran without an exception (Discovery, 2026-10-21)
- [x] Evaluator run on fresh logs from this controller
- [x] All five aggregate fractions = 1.0 (see `results/eval_summary.md`)
- [x] No further parameter change after that successful full run
- [ ] Course submission of `mp1_controller/controller.py` (when due)
