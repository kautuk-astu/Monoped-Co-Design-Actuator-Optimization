# Running saved monoped cases

Activate the existing Python environment that contains the repository's
simulation dependencies, then run from any directory:

```bash
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh nominal
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh caseA
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh caseB
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh caseC
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh controller-fixed
/path/to/Monoped-Co-Design-Actuator-Optimization/run_case.sh all
```

`all` runs the four paper cases followed by the repository's saved
controller-fixed ablation. By default each run writes an MP4 to
`results/videos/` and machine-readable metrics to `results/run_cases/`.
Use `--no-video` when running on a system without an offscreen MuJoCo backend.

The launcher never creates an environment and uses `python3` from the active
environment. Set `PYTHON_BIN=/path/to/python` if that interpreter has a
different name. It verifies `numpy`, `pandas`, `scipy`, `mujoco`, and `imageio`
before simulation.

## Saved case mapping

| Launcher case | Saved configuration | XML path | Original study |
| --- | --- | --- | --- |
| `nominal` | `summary_nominal.json` → `all_min_row`, ID `60f37fab` | `xmls/Nominal_xmls/60f37fab.xml` | `cmaes_baseline.py` |
| `caseA` | `summary_CaseA.json` → `all_min_row`, ID `52961d30` | `xmls/Case_A_xmls/52961d30.xml` | `cmaes_ll.py` |
| `caseB` | `summary_CaseB.json` → `all_min_row`, ID `0724bf7e` | `xmls/Case_B_xmls/0724bf7e.xml` | `cmaes_gear.py` |
| `caseC` | `summary_CaseC.json` → `all_min_row`, ID `92df14d3` | `xmls/design_xmls/92df14d3.xml` | `cmaes.py` |
| `controller-fixed` | minimum-cost row in `results/CMAES_output/only_design/all_*.csv`, ID `29f16492` | `xmls/only_design_xmls/29f16492.xml` | `cmaes_ctrlfixed.py` |

The launcher checks the listed XML before every run. Missing XMLs are rebuilt
deterministically from the saved row using the same actuator lookup, link-mass
interpolation, geometry edits, and actuator-limit equations used by the
corresponding CMA-ES scripts. It never reruns optimization merely to regenerate
an XML.

Reported comparison distance is `abs(signed_jump_distance)`. The detailed
per-case report retains the raw signed distance, which is important for Case C
because it can jump toward negative x.

## Explicitly rerunning an optimization

This is intentionally opt-in and starts the original script:

```bash
./run_case.sh caseC --optimize
```

The same form works for the other named cases. `all --optimize` is rejected to
avoid accidentally launching every long CMA-ES study.
