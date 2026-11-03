# Three-way monoped core-logic comparison

Review date: 2026-10-28

Compared implementations:

1. `single_axis_2bar_codesign` — current VMC/paper-style baseline.
2. `Nishant` — scheduled joint-PD and COMPAct/electrical-model experiments.
3. `monoped-rishika/.../rishika_fixed_link_codesign` — reachable/timed-VMC co-design branch.

## Scope

This comparison covers controller logic, jump state logic, optimization, actuator and energy modeling, feasibility, result meaning, and tests. Model-description files are intentionally out of scope.

## Executive conclusion

The three branches should be treated as three different experiments on a similar mechanical plant:

- The current baseline has the cleanest modular workflow and replay/logging pipeline, but its fixed-motor controller uses an intentionally unreachable virtual-leg target, its takeoff logic is weak, and its objective uses absolute carriage apex plus a rough energy proxy.
- Nishant has the strongest flight qualification and the richest electrical diagnostics, but it uses a prescribed joint-PD trajectory rather than VMC, keeps several competing engine versions, and Stage 4 falls back to the older V1 engine.
- Rishika has the most flexible and physically interpretable VMC design search, plus better kinematics tests and multi-seed studies, but its height metric combines leg extension with flight, its advanced replay path is incomplete, and some optimized dimensions become inactive.

The best unified implementation would use the baseline architecture, Rishika's reachable/configurable VMC search and tests, and Nishant's qualified flight state machine and actuator electrical constraints.

## High-level comparison

| Area | Current baseline | Nishant | Rishika |
|---|---|---|---|
| Main controller | Contact VMC; fixed joint PD in flight | Scheduled joint-space PD | Timed VMC by default; contact-only VMC option |
| Single-jump behavior | Optional latch disables actuation after first touchdown | First qualified flight selected; controller returns to crouch after landing | No latch; timed mode becomes passive, contact mode re-engages on every contact |
| Initial pose | Optimized hip/knee angles | Fixed crouch `[0.6, -1.2]` | Optimized hip/knee angles |
| Fixed-study search size | 8 variables | Usually 6; Stage 4 uses 4 | 11 variables |
| Ratios | Optimized | Optimized in early/joint studies; fixed from COMPAct in Stage 4 | Optimized with optional upper bound |
| Controller parameters | `K`, `C`, `T_gain` | Hip/knee `Kp`, `Kd` | `ori_l`, `ori_theta`, `K`, `C`, `T_gain`, `thrust_time` |
| Virtual-leg target | Fixed 10 m, deliberately unreachable | Not applicable | Optimized and reach-checked |
| Takeoff detection | One contact-free sample | 5 ms contact-loss debounce plus positive vertical velocity | One contact-free sample |
| Valid flight | Contact loss only | Completed flight, landing debounce, minimum 80 ms duration | Contact loss; optional minimum initial-to-apex gain |
| Primary height | Absolute carriage apex in objective | COM rise from takeoff to flight apex | Carriage gain from initial crouch to later apex |
| Energy | Net positive work + torque-squared Joule proxy | Electrical/mechanical proxy with current/back-EMF limits | Net or per-joint positive mechanical work |
| Motor dynamics | Static continuous torque limit | V1 speed/back-EMF/current limit; V2/V3 signed drive and inertia assumptions | Static continuous torque limit |
| Gearbox source | SSPG-only lookup | Assumed models, then actual COMPAct WPG/DSPG outputs | Full CPG/SSPG/WPG lookup; optional efficiency override |
| Main objective | Exponential absolute-height cost + total energy | Qualified COM flight rise minus energy and violation penalties | Negative initial-to-apex gain + weighted mechanical work |
| Feasibility | State/clearance/no-takeoff | Jump/current/speed/joint-limit checks | State/clearance/reach/no-takeoff; optional gain gate |
| Tests | 2 | 0 | 10 |

## Controller comparison

### Current baseline: contact VMC plus flight PD

The baseline applies task-space VMC while the foot is in contact. In flight it regulates the joints back toward the optimized initial angles with fixed PD gains. Multi-jump mode reactivates VMC after every landing; single-jump mode permanently disables actuation after the first touchdown.

Its `ori_l = 10 m` target is intentionally unreachable for a 0.40 m leg, producing a strong extension command that is often actuator-saturated. This is closer to a paper-specific thrust approximation than a realizable virtual spring.

### Nishant: prescribed joint PD

Nishant does not use task-space VMC in its primary engine. It holds a fixed crouch, linearly moves the joint targets toward extension, then commands a recovery/crouch posture. CMA-ES tunes the hip and knee ratios and four PD gains; later Stage 4 fixes gearbox ratios/masses/efficiencies from COMPAct and tunes only the gains.

This controller is easier to interpret as a prescribed jump maneuver but does not co-design the reference trajectory. Results isolate gain/transmission effects better than the VMC branches, while exploring a narrower controller family.

### Rishika: reachable timed VMC

Rishika optimizes the virtual-leg rest length, orientation, gains, and thrust duration. Default timed mode applies VMC from simulation start through `thrust_time` and then becomes passive. Contact mode applies VMC only during contact, including landing, with passive flight.

The reachable target is physically clearer than the baseline's 10 m target. However, timed and contact modes are substantially different controllers and should be reported as separate studies.

## Jump-state and metric comparison

Nishant has the strongest state logic:

1. Require five consecutive contact-free samples.
2. Require upward slider velocity above 0.05 m/s.
3. Track takeoff-to-apex hip and mass-weighted COM rise.
4. Require five contact samples for landing.
5. Require at least 80 ms of flight.
6. Score the first completed qualified flight.

The baseline and Rishika branches mark takeoff on the first transition from contact to no contact. They do not debounce contact, require upward velocity, qualify flight duration, or require landing. Their apex remains the maximum carriage height for the rest of the simulation after the first detected takeoff.

The three reported height values therefore mean different things:

```text
baseline: absolute carriage apex drives objective
Nishant:  COM apex - COM at takeoff
Rishika:  carriage apex - carriage height at initial crouch
```

Rishika's value includes stance leg extension. Its README estimates that one 0.87 m result consists of about 0.31 m extension and 0.56 m actual flight. Nishant's COM takeoff-to-apex value is the cleanest ballistic metric, although its contact and COM-velocity calculations still need validation.

## Optimization comparison

### Baseline fixed-motor vector

```text
theta1, theta2, payload,
hip_ratio, knee_ratio,
K, C, T_gain
```

The target length/orientation and nominal duration are fixed. The objective uses absolute apex, so initial pose and payload interact with both the dynamics and height term.

### Nishant vectors

The main and joint-screening vector is:

```text
hip_ratio, knee_ratio,
hip_Kp, hip_Kd, knee_Kp, knee_Kd
```

Fixed-candidate comparison and Stage 4 optimize only the four PD gains. Stage 4 uses actual COMPAct ratios, masses, and efficiencies for B/C topology combinations. This staged workflow separates actuator selection from controller tuning, but it is not one joint end-to-end optimization.

### Rishika fixed-motor vector

```text
theta1, theta2, payload,
hip_ratio, knee_ratio,
ori_l, ori_theta,
K, C, T_gain, thrust_time
```

Rishika offers the broadest VMC search and useful bound overrides. When payload is fixed, its sampled payload coordinate remains neutral; in contact mode, sampled `thrust_time` is neutral. The active vector should be built dynamically.

## Actuator and energy comparison

### Baseline

- Static continuous motor-torque limit scaled by requested ratio and lookup efficiency.
- SSPG-only lookup.
- Net positive mechanical work plus a torque-squared Joule-loss proxy.
- No speed-dependent available torque, voltage, or current constraint.

### Nishant

- V1 reduces current with back EMF, enforces a speed cutoff, and reports peak current/speed.
- V2/V3 add signed voltage/current behavior, assumed ratio-dependent gearbox mass/efficiency, and reflected inertia.
- Stage 4 imports V1, so it does not use the richer V2/V3 inertia/drive model.
- Energy is explicitly labeled a proxy, and motor constants are labeled unverified.

### Rishika

- Same static torque-limit model as the baseline.
- Full gearbox lookup can change CPG/SSPG/WPG topology with ratio.
- Optional common efficiency override.
- Supports net positive work or the better sum of positive work at each joint.
- Removes the Joule term, so the objective is not electrical-energy-aware.

All three need consistent actual-ratio handling. The baseline and Rishika select a discrete row for mass/efficiency/topology but use the continuous requested ratio for torque capacity. Nishant's Stage 4 is better in this respect because it reads actual COMPAct ratios, although its underlying motor data and remote transmission remain approximate.

## Results are not directly comparable

Representative checked-in values illustrate the metric mismatch:

| Branch/result | Reported height | What it means |
|---|---:|---|
| Baseline single-jump | 0.0753 m | Initial-to-apex carriage gain |
| Baseline multi-jump | 0.2911 m | Maximum carriage gain with contact reactivation possible |
| Nishant Stage 4 B | 0.0768 m | Qualified takeoff-to-apex mass-weighted COM rise |
| Rishika final seed 6 | 0.8679 m | Initial crouch-to-apex carriage gain; includes leg extension |

These numbers differ in controller, payload, ratios, efficiency, energy, jump segmentation, simulation horizon, and height definition. They cannot rank the branches.

## What each branch contributes

### Keep from the current baseline

- Modular separation of kinematics, model, simulation, optimization, plotting, and replay.
- Explicit contact-VMC, flight-PD, and single-jump behavior.
- Detailed candidate logging and metadata.
- Geometry and non-finite-state rejection.
- Telemetry and repeatable multi-view replay pipeline.

### Keep from Nishant

- Debounced takeoff and landing state machine.
- Positive-vertical-velocity and minimum-flight-duration qualification.
- Mass-weighted COM takeoff-to-apex measurement.
- Current, voltage, motor-speed, and overspeed diagnostics.
- COMPAct result ingestion and strict actuator feasibility screening.
- Clear separation of valid and invalid optimization candidates.

### Keep from Rishika

- Reachable optimized VMC target and thrust duration.
- Configurable physical/design assumptions.
- Per-joint positive-work accounting.
- Multiple-seed studies.
- Stronger deterministic kinematics/helper tests.
- Documentation separating leg extension from actual flight rise.

## Main risks by branch

| Priority | Baseline | Nishant | Rishika |
|---|---|---|---|
| High | Absolute apex objective and unqualified takeoff | Stage 4 uses older engine; motor constants unverified | Height conflates extension/flight; replay can use wrong settings |
| High | Unreachable 10 m target changes VMC interpretation | Results use different engines/assumptions | One-sample takeoff and no landing qualification |
| Medium | No voltage/current/speed dynamics | Monolithic duplicated engine versions | Neutral CMA dimensions and overly broad default `ori_l` bound |
| Medium | Discrete lookup properties with continuous ratio | No automated tests | Mechanical work presented as energy; no electrical term |
| Medium | Only two narrow tests | Prescribed trajectory is not co-designed | Implicit discontinuous gearbox-topology selection |

## Recommended unified experiment

### 1. Use one canonical simulator and state machine

Build on the baseline module layout, then implement:

```text
STANCE
  -> TAKEOFF_CANDIDATE after contact loss
  -> FLIGHT after debounce + positive COM velocity
  -> LANDED after debounced contact return
```

Require a minimum flight duration and choose whether later hops are ignored or treated as separate events.

### 2. Report three height metrics

```text
stance extension = COM_takeoff - COM_initial
ballistic rise    = COM_apex - COM_takeoff
total apex gain   = COM_apex - COM_initial
```

Use ballistic rise as the primary jump metric. Keep total gain for mechanical-envelope reporting.

### 3. Make controller modes explicit experiments

Support, but do not mix:

- scheduled joint PD;
- timed reachable VMC;
- contact VMC plus flight PD;
- single-jump contact VMC.

Store controller mode and every controller parameter in the best-design artifact and replay them automatically.

### 4. Use one actuator interface

Each actuator record should contain:

```text
actual ratio, topology, mass, efficiency,
continuous/peak torque, rotor and gearbox inertia,
bus voltage, current limit, winding resistance, speed limit
```

Use actual ratio consistently. Separate verified values from sensitivity assumptions.

### 5. Define energy precisely

At minimum report:

- hip positive mechanical work;
- knee positive mechanical work;
- net positive mechanical work;
- winding loss;
- estimated DC-bus energy.

Select one documented term for the objective rather than calling all variants “energy.”

### 6. Build the CMA vector dynamically

Remove fixed or inactive variables. Apply physically reachable bounds before sampling rather than rejecting large parts of the population.

### 7. Combine the tests

Retain Rishika's kinematics tests and the baseline configuration tests, then add:

- flight-state transition tests;
- controller-mode transition tests;
- electrical saturation tests;
- actual-ratio lookup tests;
- replay/evaluation equivalence tests;
- one deterministic short MuJoCo regression per controller mode.

## Recommended immediate decisions

1. Choose whether the next paper result is about ballistic flight rise or total crouch-to-apex displacement.
2. Choose one controller family for the main comparison.
3. Choose mechanical work or electrical energy as the optimization cost.
4. Standardize payload, efficiency, ratio bounds, simulation horizon, and seed set.
5. Replay the best design from each colleague through the canonical evaluator before drawing conclusions.

## Bottom line

No branch is simply “better.” They solve different versions of the problem:

- baseline: paper-style contact actuation with strong software organization;
- Nishant: qualified PD jumping with richer actuator feasibility;
- Rishika: flexible reachable-VMC co-design and sensitivity studies.

The scientifically defensible next step is not to compare their current best heights. It is to merge their strongest logic into one evaluator, hold the experiment definition fixed, and then rerun all designs and seeds.

