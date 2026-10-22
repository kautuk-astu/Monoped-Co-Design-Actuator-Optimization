# Final gearboxes for the 2-link rail monoped

Chosen from the fixed-link MAD_M6C12 seed studies (0.20 m links, 1.0 kg
payload, gearbox efficiency 0.7, gear ratio 4-8.7, per-joint energy,
min apex gain 0.05 m, ori_l <= 0.395 m).

Gear parameters come from **upstream COMPAct**
(github.com/singhaman1750/COMPAct, `main` at `c34465a`), run with
`actOpt.py MAD_M6C12 sspg 4.4` and `actOpt.py MAD_M6C12 cpg 8.65`. Upstream
uses the COMPAct paper's gearbox cost (mass - 2*efficiency + 0.2*width)
and matches the SolidWorks/Onshape templates; this repo's older
`actuator_optimization/` copy uses 0.33*mass - 0.67*efficiency and picks
module-0.5 designs instead (see this folder's git history).

| Joint | Motor | Gearbox | Ratio | Ns | Np (big / small) | Nr | Module | Planets | Mass (kg) | Model eff. | Peak torque (Nm) | Width (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Hip  | MAD_M6C12 | SSPG | 4.40:1 | 30 | 36 | 102 | 0.6 | 3 | 0.486 | 0.956 | 10.1 | 63.0 |
| Knee | MAD_M6C12 | CPG  | 8.67:1 | 18 | 46 / 32 | 96 | 0.8 / 0.8 | 3 | 0.674 | 0.939 | 20.0 | 72.5 |

Face widths: hip 5.75 mm (sun), 8.0 mm (planet, ring); knee 6.40 mm
(sun, big planet) and 6.63 mm (small planet, ring). Ring OD: hip 71.2 mm,
knee 86.8 mm. The knee's exact ratio is 8.67:1 (target 8.65:1).

## Why these ratios

- **Knee about 8.65:1.** Every seed of the `seed_study_orilmax_*` study
  converged to the top of the allowed range. For MAD_M6C12, COMPAct only
  has SSPG designs up to about 6.1:1, so this ratio is a CPG.
- **Hip 4.4:1.** The hip ratio barely matters. Re-simulating the best
  design (ori-l-max seed 6) at hip ratios 4.0-8.7 with everything else
  fixed changes jump height by 6% and energy by 1% (`hip_sweep.png`,
  `hip_sweep.csv`). Height tracks the hip actuator mass (correlation
  -0.998), so the lightest option wins: an SSPG at 4.4:1.
- The sweep and the co-design runs used the older lookup table
  (`results/optimal_gearbox_selection.csv`). The upstream designs are
  slightly lighter (hip 0.486 vs 0.500 kg, knee 0.674 vs 0.683 kg), so the
  conclusions above still hold.

## Files

- `hip_sspg_equations.txt`, `knee_cpg_equations.txt`: variable files for
  the SolidWorks templates (`sspg_actuator.SLDASM`, `cpg_actuator.SLDASM`
  from upstream `CADs/SSPG/sspg_actuator.zip` and `CADs/CPG/cpg_actuator.zip`).
  Place them as `CADs/SSPG/sspg_equations.txt` and
  `CADs/CPG/cpg_equations.txt` next to the extracted templates, open the
  assembly and rebuild.
- `hip_sspg_equations_onshape.txt`, `knee_cpg_equations_onshape.txt`: the
  same values for the Onshape templates.
- `hip_sweep.png`, `hip_sweep.csv`: the hip-ratio sensitivity sweep.

## Caveats

- Model efficiency is about 0.94-0.96, but COMPAct measured 60-80% for
  3D-printed gears; the co-design runs assumed 0.7.
- COMPAct measured more backlash and less stiffness for a printed CPG
  (2.6 deg, about 200 Nm/rad) than for an SSPG (0.6 deg, about
  240 Nm/rad), which matters for the knee.
- The U10 `motor_mount_hole_PCD` in this repo's `config.json` is 22 mm;
  upstream has 32 mm. It does not affect these MAD_M6C12 designs.
