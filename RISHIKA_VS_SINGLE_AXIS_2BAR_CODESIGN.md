# Core-logic review: `single_axis_2bar_codesign` vs. Rishika's fixed-link co-design

Review date: 2026-10-28  
Baseline: `Monoped-Co-Design-Actuator-Optimization/single_axis_2bar_codesign`  
Rishika branch: `monoped-rishika/single_axis_2bar_codesign/rishika_fixed_link_codesign` at `d8a2c56aa`

## Scope

This review intentionally ignores model-file and geometry-description differences. It compares controller behavior, jump detection and measurement, actuator and energy logic, optimization variables and objectives, feasibility rules, replay consistency, and tests.

## Executive summary

The two implementations share the same kinematics, VMC equations, torque-saturation helper, geometry feasibility logic, `Design`/`JumpResult` structure, and CMA-ES logging pattern. Rishika's directory is a derivative of the same codebase, not an independent implementation.

The current baseline fixed-motor study is a specialized “authors-style” experiment. It searches eight variables, fixes the virtual-leg target to an unreachable 10 m to create a saturated extension command, runs contact-triggered VMC with joint PD in flight, and minimizes an exponential absolute-height term plus mechanical/Joule energy.

Rishika's branch restores a broader 11-variable VMC co-design. It optimizes a reachable rest length, desired orientation, stiffness, damping, orientation gain, and thrust duration. It defaults to a timed thrust window with zero torque afterward, minimizes negative initial-to-apex gain plus weighted mechanical work, and adds useful controls for payload, gear-ratio limits, stiffness limits, efficiency, minimum apex gain, energy accounting, and contact-triggered control.

Several Rishika modifications are useful and should be retained: reachable VMC targets, per-joint positive-work accounting, configurable study assumptions, multiple-seed studies, and broader kinematics tests. However, the reported apex gain still combines stance extension and ballistic flight, takeoff is accepted after one contact-free sample, advanced experiment settings are not faithfully replayed by the video path, and fixed/unused CMA variables create neutral search dimensions.

## Shared core

The following logic is identical or essentially identical:

- Two-link forward kinematics and analytical Jacobian.
- Virtual-leg state: length, radial velocity, and orientation.
- VMC command:

  ```text
  F_radial = K * (ori_l - L) - C * L_dot
  T        = T_gain * (ori_theta - theta)
  tau      = J^T * F
  ```

- Continuous motor-torque calculation from `Kv` and continuous current.
- Joint-torque clipping followed by gearbox-efficiency scaling.
- Independent optimized initial hip and knee coordinates.
- Non-finite-state rejection and the same clearance checks.
- CMA-ES over a normalized unit cube, with all-candidate, generation-best, and global-best logging.
- Takeoff detection based on transition from foot contact to no contact.
- Apex tracking after the first detected takeoff.

Because these foundations are shared, the differences below are experiment-definition changes rather than small implementation details.

## Side-by-side comparison

| Area | Current baseline | Rishika branch |
|---|---|---|
| Fixed hardware | 0.20 m links, two `MAD_M6C12` motors, SSPG-only lookup | 0.20 m links, two `MAD_M6C12` motors, lookup may choose CPG, SSPG, or WPG |
| Search dimension | 8 | 11 |
| Optimized pose/load | `theta1`, `theta2`, payload | Same, unless payload is overridden |
| Optimized transmission | Hip and knee ratios | Same, with optional common upper bound |
| Optimized VMC terms | `K`, `C`, `T_gain` | `ori_l`, `ori_theta`, `K`, `C`, `T_gain`, `thrust_time` |
| VMC target | `ori_l = 10 m`, `ori_theta = 0`; deliberately unreachable and normally torque-saturating | Reachable `ori_l` and orientation are optimized |
| Default control | VMC while in contact; fixed joint PD in flight; VMC re-engages on later contacts | VMC during optimized time window; zero torque afterward |
| Alternate control | Single-jump latch disables actuation after first touchdown | Contact mode applies VMC whenever contact exists; zero torque in flight; no single-jump latch |
| Simulation horizon | Configurable; launcher currently uses 2.0 s | Fixed optimizer call uses `run_jump()` default of 1.5 s |
| Takeoff | First contact-to-no-contact sample | Same |
| Landing qualification | None | None |
| Height metric | Absolute carriage apex in the objective; apex gain is also logged | Initial carriage height to maximum post-takeoff carriage height |
| Energy | Net positive mechanical work plus torque-squared Joule proxy | Net positive work or sum of per-joint positive work; no Joule/electrical term |
| Objective | `0.65 * 100*exp(-(0.5+apex_height)) + 0.35*total_energy` | `-apex_gain + energy_weight*mechanical_work` |
| Minimum jump gate | Takeoff required, but no minimum gain | Optional `--min-apex-gain`; disabled by default |
| Reachability | Skipped for authors modes because 10 m target is intentional | Always enforced for optimized `ori_l` |
| Efficiency study | Uses table efficiency | Table efficiency or a common override for both joints |
| Tests | 2 fixed-hardware/objective tests | 10 kinematics/general-vector tests |

## Controller logic

### Baseline controller

The fixed-motor optimizer selects one of two closely related authors modes:

1. During contact, apply VMC.
2. During flight, use fixed joint PD about the optimized initial pose:

   ```text
   tau_hip  = 100 * (theta1 - q1) - 10 * qdot1
   tau_knee = 100 * (theta2 - q2) - 10 * qdot2
   ```

3. In multi-jump mode, contact VMC reactivates after every landing.
4. In single-jump mode, the first post-takeoff touchdown permanently disables torque.

The fixed 10 m virtual-leg target is far beyond the 0.40 m physical reach. The controller therefore behaves more like a strongly saturated extension command than a conventional spring around a realizable equilibrium. `K` still affects unsaturated regions, but much of the command can be limited by actuator torque.

### Rishika controller

Rishika supplies two modes:

- `thrust-window`: apply VMC from time zero until optimized `thrust_time`, then command zero torque for the rest of the simulation.
- `contact`: apply VMC whenever the foot has contact and command zero torque in flight.

This is a meaningful change. The timed mode can continue applying VMC briefly after takeoff if takeoff precedes `thrust_time`; after the window it has no flight or landing control. Contact mode has passive flight but re-engages the same extension-oriented VMC at every touchdown, so later rebounds can be powered and counted in the maximum apex.

Optimizing a reachable `ori_l` is physically clearer than the baseline's 10 m target. The final studies converge near the 0.395 m reachable limit and at high `K`, which effectively asks the controller to straighten the leg aggressively without using an impossible reference.

## Jump detection and measurement

Both branches use the same weak takeoff condition:

```text
had_contact and not in_contact and not takeoff
```

There is no debounce, positive vertical-velocity requirement, minimum flight time, or completed-landing requirement. A one-step loss of contact can mark takeoff. Once marked, the maximum carriage height over the remaining simulation is treated as the apex, including any later powered rebound.

Rishika's `--min-apex-gain` prevents CMA-ES from accepting a zero-height contact-loss event, but it does not separate leg straightening from flight. The branch's own README notes that a reported 0.87 m gain contains about 0.31 m of stance extension and about 0.56 m of actual flight. The optimization therefore rewards a deep initial crouch and extension range in addition to ballistic performance.

The baseline objective has a different issue: it uses absolute apex height rather than apex gain. Initial pose and starting height can therefore influence the height term directly.

A scientifically cleaner evaluator would record three separate quantities:

```text
stance_extension = takeoff_COM - initial_COM
flight_rise      = apex_COM - takeoff_COM
total_apex_gain  = apex_COM - initial_COM
```

Jump validity should require debounced contact loss, positive COM vertical velocity, a minimum flight duration, and a completed landing.

## Optimization logic

### Baseline eight-variable study

```text
theta1, theta2, payload,
hip_ratio, knee_ratio,
K, C, T_gain
```

Bounds are comparatively narrow for VMC gains: `K <= 200`, `C <= 10`, and `T_gain <= 20`. The rest length, target orientation, and nominal thrust duration are fixed.

### Rishika 11-variable study

```text
theta1, theta2, payload,
hip_ratio, knee_ratio,
ori_l, ori_theta,
K, C, T_gain, thrust_time
```

Default bounds allow `K <= 1000`, and final studies use `K <= 3000`, ratio `<= 8.7`, `ori_l <= 0.395`, fixed 1 kg payload, common efficiency 0.7, and per-joint work.

The configurable-bound helpers and explicit metadata are good additions. Two neutral-dimension problems remain:

- With `--payload`, the sampled payload coordinate is overwritten, but CMA-ES still searches that dimension.
- In `contact` mode, `thrust_time` is unused, but CMA-ES still searches it.

Neutral variables add noise to covariance adaptation and can slow or destabilize convergence. Fixed or inactive parameters should be removed from the active vector, or their normalized bounds should collapse to one value outside CMA-ES.

The default `ori_l` upper bound is 0.60 m even though two 0.20 m links have a permitted upper reach of 0.395 m after the safety epsilon. Those samples are rejected by reachability logic. Rishika's final `--ori-l-max 0.395` setting is the correct fix and should be the default for the fixed-link study.

## Objective and energy logic

### Baseline

The baseline minimizes an exponential absolute-height cost plus total energy. Its total energy is:

```text
net positive joint work + torque-squared Joule proxy
```

The net-work calculation first sums hip and knee power, then clips the sum at zero. Simultaneous positive work at one joint and negative work at the other can cancel.

### Rishika

Rishika returns to a linear height/work trade-off:

```text
cost = -apex_gain + energy_weight * selected_mechanical_work
```

`per-joint` work is a sound improvement over net work for measuring total positive actuator work:

```text
max(0, tau_hip*qdot_hip) + max(0, tau_knee*qdot_knee)
```

However, Rishika removes the Joule proxy entirely. The result is mechanical-work-aware, not electrical-energy-aware. The report and column names should say that explicitly.

When `per-joint` is selected, the summary still labels `positive_work_j` as “Mechanical energy,” while the objective actually uses `hip_positive_work_j + knee_positive_work_j`. These values can differ because of cross-joint cancellation. The summary should display the exact objective energy and the selected metric.

## Actuator-selection logic

The baseline lookup is restricted to SSPG rows. Rishika uses the full lookup and may switch among CPG, SSPG, and WPG as the requested ratio changes. This makes topology an implicit, discontinuous function of ratio rather than an explicit optimization variable.

Both implementations select the nearest table row for mass, efficiency, and topology but keep the original continuous requested ratio for torque capacity. A simulated actuator can therefore combine one table design's mass/efficiency with a slightly different ratio. Both should use the selected row's `actual_ratio`, or use a validated interpolation model consistently.

Rishika's common efficiency override is useful for sensitivity studies, but it changes only efficiency. Mass and topology remain those of the lookup design optimized under its original assumptions. That is acceptable as a clearly labeled sensitivity analysis, not as a newly validated gearbox design.

## Assessment of Rishika's modifications

### Good changes to retain

- Optimizing a reachable `ori_l` and desired orientation.
- Configurable ratio, stiffness, rest-length, payload, and efficiency assumptions.
- Per-joint positive-work accounting.
- An optional minimum-performance gate.
- Multiple-seed experiments and richer run metadata.
- Seven deterministic kinematics/helper tests plus three general-vector tests.
- Honest documentation that reported apex gain includes leg extension.

### Changes that define a different experiment

- Timed VMC with passive flight instead of contact VMC plus flight PD.
- Linear apex-gain/mechanical-work objective instead of exponential absolute-height/electrical-proxy objective.
- Optimized rest length, orientation, and thrust duration.
- Full CPG/SSPG/WPG lookup instead of SSPG-only hardware.
- Fixed 1 kg payload and common 0.7 efficiency in final studies.
- A 1.5 s fixed horizon instead of the baseline launcher's configurable 2.0 s horizon.

These are legitimate studies, but their results cannot be used as direct improvements over the baseline without replaying both designs under one common evaluator.

### Changes needing correction

- Replace one-sample takeoff with a qualified flight state machine.
- Optimize takeoff-to-apex COM rise, not only initial-to-apex carriage rise.
- Remove fixed/inactive parameters from the CMA vector.
- Make 0.395 m the fixed-link `ori_l` upper bound by construction.
- Record and replay control mode, efficiency override, duration, and energy metric together with the design.
- Display the exact energy used by the objective.
- Use actual lookup ratio consistently.

## Replay inconsistency

`preview.py --from-json` correctly restores saved gearbox efficiencies, but the user must separately supply the correct `--control-mode`; that setting is stored in run metadata rather than the best-design record.

`render_video.py` reconstructs actuators from the lookup table, has no efficiency-override option, and always uses default thrust-window control. Therefore a final study optimized with efficiency 0.7 or `control_mode=contact` is not faithfully replayed by this renderer. The replay entry point should consume the best-design record plus its metadata and reconstruct every evaluation setting automatically.

## Priority findings

1. **Do not compare saved heights or costs directly.** The objective, controller, energy metric, design space, duration, hardware lookup, and payload assumptions differ.
2. **Rishika's reported jump height conflates stance extension and flight.** This is the largest interpretation risk in the saved 0.72–0.87 m results.
3. **Both branches can accept spurious takeoff.** Neither debounces contact or qualifies a completed flight.
4. **Rishika's advanced studies are not fully replayable through `render_video.py`.** Efficiency and control mode can silently differ.
5. **Rishika searches neutral variables in fixed-payload and contact-mode studies.** This wastes CMA-ES capacity.
6. **Both actuator lookups mix discrete design properties with a continuous requested ratio.** Rishika also changes topology discontinuously.
7. **Rishika's per-joint work option is better mechanical accounting, but it is not electrical energy.** The summary does not clearly show the energy actually used in the objective.
8. **Test coverage is complementary but incomplete.** Rishika tests kinematics well; the baseline tests fixed-study configuration. Neither tests control-mode transitions, takeoff qualification, landing behavior, replay equivalence, or a deterministic MuJoCo trajectory.

## Recommended merge direction

Use the current baseline's modular structure, explicit authors/single-jump modes, plotting, and replay pipeline as the base. Port these Rishika features:

1. Reachable optimized `ori_l`, `ori_theta`, and `thrust_time` for a separate timed-VMC study.
2. Configurable bounds, fixed payload, efficiency sensitivity, and minimum-performance options.
3. Per-joint positive-work reporting.
4. Rishika's deterministic kinematics tests.
5. Multiple-seed study scripts and metadata.

Before running a combined study:

1. Add a debounced state machine: stance → qualified flight → landing.
2. Report COM stance extension, ballistic flight rise, and total apex gain separately.
3. Decide whether the controller is timed VMC, contact VMC with flight PD, or single-jump contact VMC; do not mix their results.
4. Define one objective and one energy metric.
5. Build the CMA vector dynamically so fixed or unused variables are absent.
6. Reconstruct replays from both design and run metadata.
7. Use the selected actuator row's actual ratio consistently.

## Verification performed

- Diffed all corresponding Python and shell files, excluding results and model-description files.
- Read the Rishika fixed-study README, optimizer, simulator, actuator lookup, launcher, preview, renderer, tests, metadata, best records, and representative summaries.
- Ran the baseline unit suite: 2 tests passed.
- Ran the Rishika unit suite with bytecode writes disabled: 10 tests passed.
- Confirmed that `kinematics.py` is identical between the two implementations.

