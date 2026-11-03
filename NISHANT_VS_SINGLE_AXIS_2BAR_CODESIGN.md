# Review: `Nishant` vs. `single_axis_2bar_codesign`

Review date: 2026-10-28  
Reviewed revision: `8e8a9c901` (`added Nishan's implementation`)

## Executive summary

The two directories model the same broad system—a two-link monoped whose body is constrained to vertical translation—but they are not equivalent implementations and their saved numerical results should not be compared directly.

`Nishant` is an exploratory, staged research branch. Its main line uses a fixed MuJoCo model, a prescribed joint-space PD jump, explicit voltage/current/speed approximations, debounced flight qualification, and later COMPAct-derived WPG/DSPG mass and efficiency data. It contains several generations of the experiment (`jump_engine`, V2, V3, fixed-candidate comparison, joint co-design, and Stage 4), together with backups and saved outputs.

`single_axis_2bar_codesign` is the cleaner software baseline. It separates kinematics, model construction, simulation, optimization, plotting, and rendering; uses virtual-model control (VMC); applies explicit geometry-clearance constraints; logs all candidates and run metadata; and includes a small regression-test suite. Its fixed-motor study uses an SSPG lookup, not COMPAct WPG/DSPG designs.

The best path forward is to retain the modular structure and run logging of `single_axis_2bar_codesign`, then port in three ideas from `Nishant`: qualified flight segmentation, a signed electrical drive model, and ingestion of actual COMPAct outputs. A common model, controller, objective, and measurement definition must be established before comparing optimization results.

## Side-by-side comparison

| Area | `Nishant` | `single_axis_2bar_codesign` |
|---|---|---|
| Primary intent | Exploratory actuator/gearbox and PD-control studies, including COMPAct topology comparisons | Focused fixed-link, fixed-`MAD_M6C12` SSPG co-design with paper-inspired VMC and objective |
| Mechanical layout | Fixed 0.20 m thigh and shank; vertical slider; both actuator masses at the carriage/hip; ideal remote knee drive | Same broad 2R/vertical-slider concept, with generated geometry, visible guide fixture, both actuator masses on the carriage, and ideal remote knee drive |
| Simulation setup | Fixed geometry; RK4; explicit hip/knee joint limits; initial foot penetration of 1 mm | Candidate-dependent mass/pose; `implicitfast`; no joint limits; initial foot tangent to ground; explicit geometry-clearance rejection |
| Initial pose | Fixed crouch `[0.6, -1.2]` rad | Hip and knee initial angles are optimization variables |
| Controller | Joint-space PD following a fixed hold/push/recovery schedule | Contact VMC (radial spring-damper plus orientation moment) and flight joint PD; optional single-jump latch |
| Core search | Six variables: two ratios and four PD gains | Eight variables: two initial angles, payload, two ratios, and three VMC gains |
| Other searches | Fixed COMPAct candidate + four PD gains; joint six-variable screening; Stage 4 fixed gearbox + four PD gains | A more general 15-variable optimizer remains in `optimize_fixed_link.py`, while the documented fixed study uses eight variables |
| Gearbox data | V1 placeholders; V2/V3 assumed ratio-dependent mass/efficiency/inertia; later WPG/DSPG COMPAct outputs | SSPG CSV lookup for mass and efficiency; requested ratio remains continuous |
| Motor/electrical limits | V1 uses back-EMF-limited current and speed cutoff; V2/V3 add signed voltage/current behavior and inertia assumptions | Static continuous output-torque saturation; no speed/back-EMF/current state constraint during simulation |
| Energy | Electrical/mechanical proxy, varying by engine version | Positive joint mechanical work plus a Joule-loss proxy |
| Jump validity | Five-step contact debounce, upward-velocity requirement, landing detection, and minimum 80 ms flight | First contact-to-no-contact transition counts as takeoff; no debounce or minimum flight duration |
| Objective | V1/Stage 4 maximize qualified COM rise minus energy, joint-limit, overspeed, and invalid-jump penalties | Minimize `0.65 * height_cost + 0.35 * total_energy`, where height cost uses absolute carriage apex |
| Feasibility | Jump validity, current/speed checks, and joint-limit-duration checks; Stage 4 gives feasible candidates a ranking offset | Rejects non-finite states, no-takeoff candidates, unreachable timed-VMC lengths, and toe/body/rail/link clearance violations |
| Outputs | Best JSON, convergence CSV, trajectory CSV, candidate comparison tables, and staged result folders | All-candidate, generation-best, best-so-far, metadata, summary, best JSON/CSV, telemetry plots, and multi-view MP4s |
| Automated tests | None | Two unit tests covering fixed hardware selection and objective arithmetic |
| Dependency declaration | `requirements.txt` | Dependencies are documented but there is no requirements/lock file |

## Execution and architecture

### `Nishant`

The currently relevant paths are:

1. `optimize.py` calls `jump_engine.py` for the six-variable ratio/PD baseline.
2. `optimize_v2.py`/`jump_engine_v2.py` add assumed gearbox mass, ratio-dependent efficiency, rotor/output inertia, and signed electrical limits.
3. `optimize_v3.py`/`jump_engine_v3.py` add more drive diagnostics, an open-circuit overspeed approximation, clean-jump filtering, and separate best-score/best-height outputs.
4. `compare_compact.py` fixes four precomputed WPG/DSPG candidate pairs and tunes only PD gains.
5. `optimize_joint_codesign.py` varies ratios and gains while holding each candidate's COMPAct mass and efficiency fixed—a screening approximation explicitly acknowledged in the source.
6. `run_stage3_compact.py` invokes an external/local COMPAct checkout for selected B/C ratios.
7. `optimize_stage4.py` reads the resulting gearbox CSVs, fixes the actual ratios/masses/efficiencies, and tunes four PD gains.

The important subtlety is that Stage 4 imports `jump_engine.py`, not V2 or V3. Therefore Stage 4 uses the V1 actuator dynamics: COMPAct mass and efficiency are inserted, but gearbox inertia and the V2/V3 signed drive behavior are not used.

### `single_axis_2bar_codesign`

The fixed-motor path is:

1. `run_cmaes_fixed_link_motor.sh` selects run settings and launches the study.
2. `optimize_fixed_link_motor.py` maps normalized CMA-ES samples into eight physical variables.
3. `model.py` looks up actuator properties and constructs the candidate model.
4. `simulate.py` runs VMC/flight control, torque saturation, energy accounting, takeoff detection, and clearance checks.
5. `kinematics.py` supplies forward kinematics, the Jacobian, VMC torque mapping, clearances, and work helpers.
6. The optimizer records every candidate and retains the best design.
7. `render_best.py`, `render_video.py`, and `plot_best_jump_traces.py` replay and visualize the selected design.

This separation is substantially easier to test and maintain than the nested simulation logic in each `Nishant/jump_engine*.py` file.

## Physics and control differences

### Model and initialization

`Nishant` starts every candidate from the same crouch. The foot begins with 1 mm penetration, and a five-step debounce is used to suppress contact chatter. This makes controller comparisons more controlled, but the initial contact transient can inject sensitivity into force and energy measurements.

The reference implementation optimizes the initial joint pose and payload and starts with the foot tangent to the floor. This expands the design space, but it also means gains/ratios are being compared together with different initial potential energies and carried masses.

`Nishant` enforces MuJoCo joint limits and penalizes time near its limit thresholds. The reference simulation does not impose joint limits; instead, it rejects candidates when robot/rail geometry clearances fall below 4 mm. These are materially different feasibility sets.

### Controller

`Nishant` uses direct joint PD control:

```text
tau_requested = Kp * (q_target - q) - Kd * qdot
```

The target follows a fixed crouch, linear extension, and recovery schedule. Only the ratios and PD gains are normally optimized.

The reference implementation uses task-space VMC in contact:

```text
F_radial = K * (L0 - L) - C * Ldot
T        = T_gain * (theta0 - theta)
tau      = J^T * F
```

In flight it uses fixed joint PD about the optimized initial pose. The two controllers can produce very different torque, contact-force, and energy histories even with identical hardware.

### Actuator representation

`Nishant` has the richer electrical experiments. Its V1 engine limits current using bus voltage and back EMF, enforces a maximum motor speed, and computes an energy proxy from winding losses plus positive mechanical work. V2/V3 add gearbox mass/inertia assumptions and signed voltage/current behavior. However, the main README correctly warns that its motor parameters are not independently verified.

The reference implementation computes continuous motor torque from `Kv` and continuous current, multiplies it by the requested ratio, applies a tabulated efficiency, and clips joint torque. It does not reduce available torque with speed and does not reject current, voltage, or motor-speed violations. Its `joule_energy` is a compact torque-squared proxy rather than a full electrical calculation.

### Jump measurement

`Nishant` reports mass-weighted COM rise from takeoff to apex for the first completed flight that lasts at least 80 ms. The reference study reports carriage apex gain from the initial state and its objective uses absolute carriage apex height. The reference multi-jump mode can continue applying VMC on later contacts; its single-jump mode disables actuation after the first touchdown.

Consequently, a saved `com_rise_m` from `Nishant` and `apex_gain` from the reference are different measurements, not alternate names for the same quantity.

## Saved-result snapshot (not an apples-to-apples benchmark)

The checked-in `Nishant/results_stage4` files report:

| Stage 4 configuration | Strictly feasible | COM rise | Energy proxy | Total actuator mass |
|---|---:|---:|---:|---:|
| B: WPG/WPG | Yes | 0.07682 m | 65.87 J | 1.580 kg |
| C: DSPG/WPG | Yes | 0.06708 m | 66.01 J | 1.809 kg |

The checked-in reference single-jump summary reports a 0.07532 m carriage apex gain, 6.37 J positive mechanical work, and 1.050 kg total actuator mass. The reference multi-jump summary reports 0.29107 m apex gain. These values must not be ranked against Stage 4 because the controller, initial pose, payload, actuator model, energy definition, jump metric, and objective differ.

## Review findings

### High priority

1. **The reference actuator lookup mixes discrete data with a continuous physical ratio.** `model.actuator_for()` selects the nearest SSPG row for mass and efficiency but stores and uses the original requested ratio for torque capacity. Thus a candidate can receive the properties of one realizable SSPG design while being simulated at another, continuously chosen ratio. Use the selected row's `actual_ratio` everywhere, or interpolate a validated continuous model and label it as such.

2. **The compared results use incompatible definitions.** `Nishant` optimizes qualified COM rise from takeoff; the reference objective uses absolute carriage apex and a different energy term. Any claim that one repository performs better would be unsupported until both candidates are replayed through one shared evaluator.

3. **Stage 4 bypasses the more detailed V2/V3 engine.** It imports `jump_engine.py`, so its limitations include no gearbox inertia and the earlier one-sided electrical approximation despite the presence of newer engines. Either make the engine an explicit command-line choice or consolidate the versions into one configurable simulator.

### Medium priority

4. **`Nishant` has no automated tests.** The most important missing cases are takeoff/landing debounce, the 80 ms qualification threshold, current/voltage/speed saturation, mass updates, COMPAct CSV parsing, and deterministic objective calculations.

5. **The reference tests are too narrow.** The two current tests pass, but they only check fixed hardware selection and objective arithmetic. They do not cover kinematics, torque saturation, clearance rejection, takeoff logic, single-jump latching, energy integration, or replay consistency.

6. **Both documentation entry points contain stale commands.** `Nishant/README.md` advertises `optimize.py --view`, but that option does not exist. `single_axis_2bar_codesign/README.md` references missing `generate_xml.py`, `optimize.py`, `preview.py`, and `view_static.py`, and documents launcher defaults of 100 iterations/8 s while the launcher currently uses 1000 iterations/2 s. The detailed fixed-motor README is closer to the current implementation.

7. **`Nishant` mixes active code, backups, archives, and results at the same level.** There are multiple `.bak`, `.save`, `_backup`, `_before_*`, copied V3, and ZIP versions. One backup (`jump_engine_visual_broken_backup.py`) has an `IndentationError`, so repository-wide syntax checks fail. Move historical artifacts outside the importable source tree or rely on Git history.

8. **Experiment metadata is weaker in `Nishant`.** Most runs save the best result and convergence but not every evaluated candidate, Git revision, Python/MuJoCo versions, or full CLI arguments. The reference run directory provides most of these.

9. **Stage 3 depends on an external/local COMPAct checkout.** The final CSVs let Stage 4 run against prior results, but regenerating them requires separately supplied COMPAct code. Record the exact COMPAct revision and environment so the actuator-design stage is reproducible.

### Lower priority

10. **The reference validation error says “length 11” for an eight-element vector.** This is a message-only defect in `optimize_fixed_link_motor.denormalize()`.

11. **Result storage is noisy.** `Nishant` versions many duplicate staged outputs and backups, making it hard to tell which engine and assumptions produced the canonical result. A documented artifact-retention and naming policy would reduce ambiguity.

12. **Packaging is incomplete.** `Nishant` has a requirements file but uses broad version ranges; the reference has no machine-readable dependency file. Neither directory is packaged as an importable Python module, so imports depend on executing scripts from an expected path.

## What each implementation does better

### Strengths worth keeping from `Nishant`

- Contact debounce, landing detection, minimum-flight qualification, and takeoff-to-apex COM measurement.
- Explicit current, voltage, motor-speed, and overspeed diagnostics.
- Separation between best energy-adjusted score and best height in V3.
- Actual COMPAct mass/efficiency/ratio ingestion and validation in Stage 4.
- Clear warnings that the motor and energy models are exploratory.

### Strengths worth keeping from `single_axis_2bar_codesign`

- Modular kinematics/model/simulation/optimization organization.
- Typed immutable design/result records and explicit helper functions.
- Geometry-clearance checks using values read from the generated model.
- Complete candidate logging, metadata, summaries, best-design retention, replay, telemetry, and rendering.
- A launcher that checks required files/dependencies and supports reproducible seed/run settings.

## Recommended convergence plan

1. Define one canonical `Design`, `Actuator`, and `JumpResult` schema.
2. Keep the reference module boundaries, but add a configurable electrical model based on the tested portions of `jump_engine_v3.py`.
3. Load actual gearbox ratio, mass, efficiency, inertia, and torque limits from a versioned actuator record. Do not combine a nearest-row mass/efficiency with an unrelated continuous ratio.
4. Choose one controller for a comparison study: either PD trajectory tracking or VMC. If both are retained, report them as separate experiments.
5. Standardize the metric to mass-weighted COM rise from a debounced takeoff to flight apex, and require a completed minimum-duration flight.
6. Standardize the objective and energy definition, then replay both saved designs through that one evaluator.
7. Add tests for kinematics, electrical limits, flight state transitions, feasibility constraints, energy integration, and deterministic short simulations.
8. Record the exact COMPAct source revision/environment used to generate actuator data.
9. Consolidate or archive obsolete scripts and regenerate the READMEs from the actual supported entry points.

## Verification performed

- Inspected all active Python, shell, config, README, candidate, and representative result files in both directories.
- Ran `python -m unittest discover -s single_axis_2bar_codesign/tests -p 'test_*.py' -v`: both tests passed.
- Ran `bash -n` on both reference launchers: passed.
- Ran short, non-writing smoke simulations through `Nishant/jump_engine.py` and the reference single-jump path: both executed successfully; the intentionally short/default candidates did not produce valid jumps.
- Ran a repository-wide Python compilation check: active code compiled, but `Nishant/jump_engine_visual_broken_backup.py` failed with its existing indentation error.

