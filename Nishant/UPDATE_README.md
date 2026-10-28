# Updated jump evaluation and CMA-ES

This release uses one common MuJoCo simulation (`jump_engine.py`) for both
`first_jump.py` and `optimize.py`.

Changes:
- Starts from verified crouch with 1 mm initial foot penetration.
- Recomputes MuJoCo contact forces at the post-integration state (`mj_forward`).
- Debounces takeoff/landing by 5 ms and requires >=80 ms flight to qualify.
- Records hip and mass-weighted centre-of-mass heights.
- Switches to a crouch target after the first qualifying landing.
- Uses first qualified completed flight's COM rise as the objective.
- Keeps independent 4..25 gear ratios and four PD gains as CMA-ES variables.
- Writes diagnostics and optimization convergence files.

Limitations:
- Motor electrical data is UNVERIFIED placeholder, not validated MAD MC12 data.
- Motor model is simplified and does not account for regeneration, inductance,
  reflected drivetrain inertia, cable transmissions or gearbox mass changes.
- Ground force is summed contact-normal magnitude, not global vertical GRF.
- Flight detection relies on contact and time thresholds and needs validation.
- 1 mm initial penetration and floor solver contact may create transients.
- No MuJoCo dynamics execution was possible in the authoring environment;
  run and verify locally before relying on optimization outcomes.

Run:
  source .venv/bin/activate
  python first_jump.py
  python optimize.py --generations 5 --population 6

The original project files should be backed up before replacing.
