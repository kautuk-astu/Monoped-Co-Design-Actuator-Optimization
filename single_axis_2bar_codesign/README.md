# Single-axis two-bar jumper

For a detailed explanation of the fixed 0.20 m-link, fixed `MAD_M6C12`
CMA-ES study—including every function on its runtime path, execution steps,
controller equations, outputs, and a section-by-section mapping to
`2510.05923v1`—see
[`README_optimize_fixed_link_motor.md`](README_optimize_fixed_link_motor.md).

This is a self-contained, deliberately small experiment for a *single* two-link
leg attached to a vertical pole.  The pole is modelled as a body that may
translate only along `z`; it supplies whatever lateral reaction is needed to
keep the body on-axis.  It is therefore a jump/actuator test rig, not a
free-balancing robot.

The leg has an actuated hip and knee, a spherical foot, and a horizontal
ground plane. A visible vertical guide pole and carriage bracket make the test
fixture explicit. The physical one-axis constraint comes from the carriage's
single `pole_slide` joint; the pole/bracket are massless and non-colliding, so
they cannot inject contact forces into the jump. A mass-carrying spherical
hip-motor cover sits at the hip hinge and is joined visually to the slider
block; the knee motor mass is placed at the knee pivot. The optimizer uses a
two-link virtual-model controller (VMC) during its commanded thrust phase.
The body remains vertically constrained by `pole_slide`, but the toe is not
required or rewarded to lie directly under the hip.

## Requirements

Run from the repository root using the existing environment:

```bash
pip install numpy pandas mujoco cma
```

The self-contained `data/sspg_actuator_selection.csv` lookup and
`data/motor_config.json` must be present. The fixed-motor launcher uses only
the SSPG rows from this lookup.

## Generate one XML model

```bash
python single_axis_2bar_codesign/generate_xml.py \
  --output single_axis_2bar_codesign/generated/example.xml \
  --thigh 0.25 --shank 0.25 --theta1 -0.25 --theta2 0.50 --payload 1.0 \
  --hip-motor U8 --knee-motor U8 --hip-ratio 10 --knee-ratio 10
```

## Run an optimization

```bash
python single_axis_2bar_codesign/optimize.py --iterations 100 --population 8 --seed 5
```

This produces one generated XML per sampled design and CSV files under
`single_axis_2bar_codesign/results/`.  Start with a short run to check MuJoCo and the
contact behaviour; a realistic CMA-ES study is expensive.

## Run the fixed 2R configuration

Use this launcher for the fixed hardware study requested for the main 2R
co-design. Both links are fixed at 0.20 m and both joints use `MAD_M6C12`;
only SSPG gearbox ratios, initial joint angles, payload, and controller
parameters are optimized:

```bash
bash single_axis_2bar_codesign/run_cmaes_fixed_link_motor.sh
```

The launcher defaults to EGL rendering, seed 5, 100 CMA-ES iterations, a
population of 8, and an 8 s simulation duration. Change those defaults at the
top of `run_cmaes_fixed_link_motor.sh`; `--seed`, `--maxiter`, `--popsize`,
and `--simulation-duration` remain available as optional overrides. The authors' objective weights remain fixed at 0.65 for
height and 0.35 for total energy. Each run keeps only its global-best `best_design.xml`; candidate
XMLs are temporary files.

For each valid best design, the launcher renders `best_jump_front.mp4`,
`best_jump_left_oblique.mp4`, and `best_jump_right_oblique.mp4` in the run
directory, then combines them into `best_jump_collage.mp4` as a horizontal
three-panel comparison video.

## Single powered jump

To optimize and render one powered jump only, use:

```bash
bash single_axis_2bar_codesign/run_cmaes_fixed_link_motor_single_jump.sh
```

After the first touchdown following takeoff, the controller commands zero
torque for the remainder of the simulation. This prevents repeated powered
hops from contributing to the recorded apex or energy.

## View the jumper interactively

Use a desktop environment with MuJoCo and an OpenGL backend, then run:

```bash
python single_axis_2bar_codesign/preview.py
```

It writes `single_axis_2bar_codesign/generated/preview.xml`, initializes the leg at
the supplied independent `theta1`/`theta2` pose, and shows the VMC thrust phase.
The viewer closes after the requested simulated duration. Change the design with the same
options as `generate_xml.py`, for example `--thigh 0.30 --shank 0.22 --duration 3`.

On a headless server, run the optimizer or simulator normally and render a
video with an offscreen MuJoCo backend instead; an interactive window requires
a display.

### Inspect only the robot and guide fixture

This opens a static model and remains open until you close the viewer. It does
not run the controller or physics loop:

```bash
python single_axis_2bar_codesign/view_static.py
```

It accepts geometry and actuator arguments, for example
`--thigh 0.30 --shank 0.22 --hip-ratio 12 --knee-ratio 12`.

## Render a video (recommended for AnyDesk/headless sessions)

This path uses EGL directly on the GPU and does not create an X11/AnyDesk
window:

```bash
MUJOCO_GL=egl python single_axis_2bar_codesign/render_video.py \
  --output single_axis_2bar_codesign/results/jump.mp4
```

It uses the same design options as the interactive preview. For example, use
`--thigh 0.30 --shank 0.22 --duration 3` to render a longer, asymmetric-leg
trial. The renderer requires `imageio` and an H.264-capable ffmpeg plugin.

## CMA-ES co-design vector

The normalized CMA-ES point is denormalized in this fixed order:

`l1, l2, theta1, theta2, payload_mass, hip_motor_idx, knee_motor_idx,
hip_gear_ratio, knee_gear_ratio, ori_l, ori_theta, K, C, T_gain, thrust_time`.

`l1` and `l2` are each bounded to 0.15--0.30 m, centered around the 200 mm
hardware links. `theta1` and `theta2` are independent initial/crouched joint
coordinates. They are never generated by inverse kinematics. The global `ori_l` interval is
0.05--0.60 m, then each candidate is rejected unless it satisfies
`abs(l1-l2)+0.005 <= ori_l <= l1+l2-0.005` m.

The VMC defines the hip-to-toe vector `p`, with `L = ||p||` and
`theta = atan2(p_x, -p_z)`: zero means the toe is vertically below the hip.
During `thrust_time`, it applies `F_radial = K * (ori_l - L) - C * L_dot` and
an orientation moment `T = T_gain * (ori_theta - theta)`. The tangential force
is `T/L` and joint torques are `tau = J^T F`. Positive radial force increases
virtual-leg length. Motor-output torques are saturated before applying gearbox
efficiency.

## Feasibility constraints

The simulation rejects a candidate (cost `1e6`) on non-finite state, malformed
model, impossible actuator lookup, invalid virtual-leg reach, or either of
these explicit geometry checks (initial pose and every simulation step):

- hip-cover sphere to foot sphere clearance;
- foot sphere to the visual pole capsule clearance.

Both require more than the named 4 mm safety margin. Geometry radii and pole
pose are read from the generated MuJoCo model; the pole remains non-colliding.

## Objective

The optimizer minimizes

`cost = -apex_gain + 0.015 * positive_mechanical_work`

where `apex_gain = apex_height - initial_height`; this avoids rewarding a
longer leg just for starting higher. A simulation that never leaves the ground
receives a large penalty. Actuator mass is logged but is not an objective term.
Only positive applied joint mechanical work is accumulated:
`max(0, tau_hip*qdot_hip + tau_knee*qdot_knee) * dt`.

The reported motion is one-dimensional at the body: it cannot translate
horizontally or rotate.  The foot and leg can still generate lateral internal
loads, which are reacted by the virtual pole.
