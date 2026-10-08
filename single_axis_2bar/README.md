# Single-axis two-bar jumper

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
block; the knee motor mass is placed at the knee pivot. A joint-space
controller settles the leg, crouches it, then commands an extension. The
optimizer selects link geometry, payload mass, the motor and gearbox at each
joint, the crouch depth, thrust duration, and two proportional gains.

## Requirements

Run from the repository root using the existing environment:

```bash
pip install numpy pandas mujoco cma
```

The actuator lookup at `results/optimal_gearbox_selection.csv` must be
present.  It is the same precomputed motor/gearbox data used by the five-bar
experiments.

## Generate one XML model

```bash
python single_axis_2bar/generate_xml.py \
  --output single_axis_2bar/generated/example.xml \
  --thigh 0.25 --shank 0.25 --payload 1.0 \
  --hip-motor U8 --knee-motor U8 --hip-ratio 10 --knee-ratio 10
```

## Run an optimization

```bash
python single_axis_2bar/optimize.py --iterations 100 --population 8 --seed 5
```

This produces one generated XML per sampled design and CSV files under
`single_axis_2bar/results/`.  Start with a short run to check MuJoCo and the
contact behaviour; a realistic CMA-ES study is expensive.

## View the jumper interactively

Use a desktop environment with MuJoCo and an OpenGL backend, then run:

```bash
python single_axis_2bar/preview.py
```

It writes `single_axis_2bar/generated/preview.xml`, initializes the leg in a
valid standing pose, and shows the crouch-and-jump motion. The viewer closes
after the requested simulated duration. Change the design with the same
options as `generate_xml.py`, for example `--thigh 0.30 --shank 0.22 --duration 3`.

On a headless server, run the optimizer or simulator normally and render a
video with an offscreen MuJoCo backend instead; an interactive window requires
a display.

### Inspect only the robot and guide fixture

This opens a static model and remains open until you close the viewer. It does
not run the controller or physics loop:

```bash
python single_axis_2bar/view_static.py
```

It accepts geometry and actuator arguments, for example
`--thigh 0.30 --shank 0.22 --hip-ratio 12 --knee-ratio 12`.

## Render a video (recommended for AnyDesk/headless sessions)

This path uses EGL directly on the GPU and does not create an X11/AnyDesk
window:

```bash
MUJOCO_GL=egl python single_axis_2bar/render_video.py \
  --output single_axis_2bar/results/jump.mp4
```

It uses the same design options as the interactive preview. For example, use
`--thigh 0.30 --shank 0.22 --duration 3` to render a longer, asymmetric-leg
trial. The renderer requires `imageio` and an H.264-capable ffmpeg plugin.

## Objective

The optimizer minimizes

`cost = -apex_gain + 0.015 * positive_mechanical_work + 0.05 * actuator_mass`

where `apex_gain` is the body height above its standing initial height.  A
simulation that never leaves the ground receives a large penalty.  The weights
are intentionally exposed as command-line options because they are study
choices, not physical constants.

The reported motion is one-dimensional at the body: it cannot translate
horizontally or rotate.  The foot and leg can still generate lateral internal
loads, which are reacted by the virtual pole.
