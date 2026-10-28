# Two-link vertical monoped: CMA-ES co-design prototype

Adapted from the uploaded five-bar repository's **CMA-ES co-design idea**; the five-bar dynamics have been replaced by a new MuJoCo two-link vertical-slider model.

## Critical validation warning
`motor_config.json` contains **illustrative placeholders, NOT verified MAD MC12 specifications**. Replace the parameters with confirmed MAD MC12 data before interpreting results. The model uses a simplified DC-equivalent motor approximation, constant gearbox efficiency, and no gearbox inertia. Gearbox topology is deliberately not selected. The knee motor is visualized at the hip, with ideal remote torque transmission; physical belt/cable dynamics are not yet modelled. Masses are preliminary. This is a first-jump research prototype, **not a validated final co-design model**.

## Ubuntu setup
```bash
cd two_link_codesign
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python optimize.py --generations 30 --population 12
python optimize.py --generations 30 --population 12 --view
```

Outputs in `results/`: `best.json`, `trajectory.csv`, `convergence.csv`. The optimizer searches hip/knee ratios in [4,25] and hip/knee PD gains. It does not optimize gearbox topology. The score rewards airborne hip rise and penalizes energy and joint-limit contact. A zero-rise run is not a successful jump.

## Next improvements
Confirm motor parameters, refine motor model, add physical remote knee transmission and gearbox reflected inertia, validate masses, improve contact detection and reference trajectory co-design, run multiple CMA-ES seeds, compare best designs across energy-height tradeoff. The present PD references are fixed, so this is a **six-variable** co-design baseline, not a full trajectory co-design.
