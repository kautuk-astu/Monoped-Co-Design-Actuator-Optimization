m

# Controller-enforced vertical toe alignment

This study lives entirely in `single_axis_2bar_codesign`. It retains the
generated two-link MuJoCo model, actuator lookup, torque saturation, CMA-ES
workflow, and vertical carriage slider. The slider is the legitimate test-rig
guide. The model has no toe/hip equality, weld, linkage, or runtime position
lock; toe alignment is produced only by hip and knee motor torque.

## Frames, kinematics, and controller

Joint coordinates are hip angle `q1` and relative knee angle `q2`, both about
MuJoCo +y. In the world x-z plane,

```text
p = toe - hip
  = [-l1 sin(q1) - l2 sin(q1+q2),
     -l1 cos(q1) - l2 cos(q1+q2)]
pdot = J(q) qdot
```

`vertical_leg_ik()` solves `p=[0,-d]` on the negative-hip/positive-knee crouch
branch, applies the actual link lengths, rejects unreachable lengths, and
keeps references 5 mm away from folded/full-extension singularities.
The inherited MuJoCo XML has no hinge stops; rather than silently adding new
hardware, aligned evaluations reject hip states outside ±pi/2 and knee states
outside `[0.03, pi-0.03]` as software feasibility failures.

During stance the original reachable radial spring/damper supplies the world-z
command. Horizontal stabilization is explicit:

```text
Fx = -Kx (x_toe - x_hip) - Dx (vx_toe - vx_hip)
tau_requested = J(q).T @ [Fx, Fz]
```

The existing actuator helper clips output torque and applies the selected SSPG
efficiency. The four phases are:

1. `PREPARE`: motor PD tracks the aligned crouch reference for 50 ms.
2. `STANCE`: finite-duration vertical VMC plus horizontal Cartesian feedback.
3. `FLIGHT`: after 5 ms contact-loss debounce and positive COM velocity, motor
   PD tracks an aligned 0.24 m retraction reference.
4. `LANDING`: after the COM apex, motor PD tracks an aligned 0.31 m reference;
   first contact is debounced for 5 ms and must complete at least 80 ms flight.

The first completed qualified flight is the canonical event. Ballistic height
is mass-weighted COM apex minus COM height at debounced takeoff. The separate
crouch-to-apex metric includes stance extension.

## CMA-ES vector and objective

`optimize_aligned.py` reuses the installed `cma` implementation. Every design
uses a fixed 0.305 m initial base height (0.280 m vertical hip-to-toe distance
plus the 0.025 m foot radius), a fixed 1.0 kg payload, and the same IK-derived
joint pose. Both joints use the fixed `MAD_M6C12` motor. Other fixed values are
`l1=l2=0.20 m`, flight/landing references, preparation interval, and flight
joint-PD gains. Its active vector is:

| Variable                    |        Bounds |
| --------------------------- | ------------: |
| hip SSPG target ratio       |     4.4–6.1 |
| knee SSPG target ratio      |     4.4–6.1 |
| vertical stiffness `K`    | 100–1000 N/m |
| vertical damping `C`      |   0–20 N s/m |
| orientation gain `T_gain` |   0–20 N m/rad |

Initial angles, initial base height, and payload are fixed rather than CMA
variables; both motor selections are fixed as well. A requested SSPG ratio is
snapped to the nearest table entry and the physical model uses that entry's
`actual_ratio`; it never treats an unavailable requested ratio as real
hardware. The VMC rest length (0.38 m), thrust duration (0.15 s), horizontal
stiffness (300 N/m), and horizontal damping (10 N s/m) are fixed. `T_gain`
adds the paper-style virtual orientation spring to the aligned controller.

The minimized objective is

```text
-ballistic_height
+ 200 * MSE(horizontal_error)
+ 400 * prelanding_error**2
+ 0.002 * positive_mechanical_work_J
```

MSE spans reset through first landing. `E` is the repository's existing sum of
non-regenerative positive joint mechanical work in joules. The legacy winding
loss proxy remains logged separately and is explicitly not a validated
electrical/battery model. Missing qualified flight, non-finite state, more than
5 mm toe penetration, unsafe geometry, or invalid IK receives the hard
infeasibility cost rather than a soft successful-jump score.

## Run and review

From this directory:

```bash
python -m pip install -r requirements-aligned.txt
python -m unittest discover -s tests -v
python optimize_aligned.py --maxiter 1 --popsize 4 --seed 5 \
  --simulation-duration 0.8 --output-dir results/aligned/smoke
python optimize_aligned.py --maxiter 100 --popsize 8 --seed 5
python replay_aligned.py results/aligned/<run-directory>
python replay_aligned.py results/aligned/<run-directory> --video

# Requested long end-to-end study: popsize=8, seed=5, 1000 generations,
# 10 seconds per candidate, followed by exact best-candidate replay.
./run_aligned_end_to_end.sh
```

Each run saves metadata with parameter names, bounds, fixed values, objective
weights, and duration; every candidate; the exact best design/XML; replayed
validation metrics; full telemetry CSV; alignment-error/phase plot; and an
optional MP4. `compact_optimization_log.json` is the short review-facing file:
it contains only the five winning variables, optimizer hyperparameters, and
key base/toe results. Telemetry includes world-frame hip/toe position, alignment error
and velocity, joint state, requested/applied torque, saturation, phase,
contact, COM state, and takeoff/apex/landing events.

Motor continuous-current and rated-voltage data exist in `data/motor_config.json`,
but this study does not claim a verified voltage/back-EMF battery model. It
therefore enforces the repository's established continuous output-torque and
gearbox-efficiency limits and reports saturation, without inventing additional
current or speed specifications.
