# Fixed-link co-design of the 2-link rail monoped (MAD_M6C12)

A self-contained snapshot of the fixed-link co-design work for the 2-link
leg on a vertical rail: the simulation and CMA-ES optimizer, all study
results, and the final hip/knee gearbox designs for 3D printing.

It does not change anything else in this repository: the code and data
here are copies, and `model.py` reads the gearbox table and motor config
from `./data` instead of the repository root.

## Contents

| Path | What it is |
|---|---|
| `model.py`, `kinematics.py`, `simulate.py` | MuJoCo model generation, kinematics and jump simulation |
| `optimize_fixed_link_motor.py` | CMA-ES co-design with 0.20 m links and MAD_M6C12 motors (main script) |
| `optimize_fixed_link.py`, `optimize.py` | Variants that also optimize the motors / link lengths |
| `preview.py`, `render_video.py`, `view_static.py`, `generate_xml.py` | Viewer and rendering tools |
| `run_cmaes*.sh` | Non-interactive launchers |
| `tests/` | Unit tests (`python -m pytest tests`) |
| `data/optimal_gearbox_selection.csv` | Gear ratio -> best gearbox (mass, efficiency) lookup |
| `data/config.json` | Motor specs (with the U10 mount PCD and `Motor8020_framed` alias added) |
| `results/` | All study outputs (see below) |
| `README_2bar_model.md` | Original README of the 2-bar model |

## Optimizer options

All options are off by default; without them the optimizer behaves as
before.

| Option | Meaning |
|---|---|
| `--energy-metric per-joint` | Count hip and knee positive work separately (co-design paper Eq. 16-17) |
| `--gearbox-efficiency 0.7` | Override the table efficiency (COMPAct measured 60-80% for printed gears) |
| `--payload 1.0` | Fix the carriage payload instead of optimizing it |
| `--ratio-max 8.7` | Upper bound of both gear ratios (single-stage QDD range) |
| `--k-max 3000` | Upper bound of the VMC stiffness K |
| `--ori-l-max 0.395` | Cap the VMC rest length at the leg's reach (0.20 m links) |
| `--min-apex-gain 0.05` | Reject candidates that barely leave the ground |
| `--control-mode contact` | Apply the VMC whenever the foot touches the ground (paper approach) instead of a fixed thrust window |

`preview.py --from-json <best_design.json>` replays a saved design in
the MuJoCo viewer with its recorded efficiency.

Example (the settings of the final study):

```bash
python optimize_fixed_link_motor.py --seed 6 --maxiter 300 --popsize 16 \
  --payload 1.0 --ratio-max 8.7 --gearbox-efficiency 0.7 --energy-metric per-joint \
  --k-max 3000 --min-apex-gain 0.05 --ori-l-max 0.395
```

## Results

Each study ran seeds 5, 6 and 7. Every run folder has `metadata.json`
(settings), `all_candidates.csv`, `best_design.json` and `summary.txt`.

| Folder | Settings | Best reported jump |
|---|---|---|
| `fixed_link_MAD_M6C12_run_2026-10-16_*` | Defaults, 100 x 8 | 1.12 m (table efficiency, CPG 14.5/16.8:1) |
| `seed_study_2026-10-18_15-43-00` | Defaults vs efficiency 0.7 / per-joint energy / K <= 3000, 100 x 8 | Inconsistent across seeds |
| `seed_study_long_*` | + payload 1.0 kg, ratio <= 8.7, 300 x 16 | 0.84 m; one seed stuck at zero jump |
| `seed_study_minapex_*` | + min apex gain 0.05 m | 0.86 m; one seed found no valid design |
| `seed_study_orilmax_*` | + ori_l <= 0.395 m | **0.87 m in two seeds, 0.72 m in one; knee converges to 8.65:1** |
| `seed_study_contact_*` | + `--control-mode contact` | 0.20 m, almost all of it leg straightening (see caveats) |
| `final_gearboxes/` | Chosen hip/knee gearboxes | See below |

### Final gearboxes (`results/final_gearboxes/`)

From upstream COMPAct (paper cost function), for the MAD_M6C12 motor:

| Joint | Gearbox | Ratio | Teeth | Module | Mass |
|---|---|---|---|---|---|
| Hip | SSPG | 4.4:1 | 30 / 36 / 102 | 0.6 | 0.486 kg |
| Knee | CPG | 8.67:1 | 18 / 46 / 32 / 96 | 0.8 | 0.674 kg |

The knee ratio is where every seed of the ori-l-max study converged. The
hip ratio barely matters (6% height, 1% energy across 4-8.7:1, see
`hip_sweep.png`), so the lightest option was chosen. The folder also has
the SolidWorks and Onshape equation files and a README with the
reasoning.

## Caveats

- **Jump height is measured from the crouched start.** `apex_gain` also
  counts the leg straightening and rewards a deeper crouch. For the
  ori-l-max best design, 0.87 m reported = 0.31 m leg extension + 0.56 m
  real flight.
- **Landing is not controlled** in the default thrust-window mode: the
  motors are off after the push, so the leg folds on landing.
- **Contact mode** lands cleanly but found no real jump (its 0.20 m is
  leg straightening).
- **Assumed hardware values:** 0.20 m links, 1.0 kg payload, tube link
  masses and 0.7 gearbox efficiency stand in for the real robot's values.
- The link-to-rail clearance check can underestimate collisions (the
  segment-distance helper overestimates distances in some poses).

Source: branch `monoped-codesign` of the Windows/WSL working copy, commit
676519f9.
