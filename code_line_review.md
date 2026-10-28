# Code-Line Review: Monoped Co-Design Optimizer

This document reviews `final_humanoids_paper_results/Final_humanoids_results/cma_ll_gear.py` and the files on its active execution path. It also explains the associated robotics and mechanics concepts for readers without a mechanical background.

The corresponding paper is [`2510.05923v1.pdf`](2510.05923v1.pdf), **A Co-Design Framework for Energy-Aware Monoped Jumping with Detailed Actuator Modeling**.

## 1. Executive summary

`cma_ll_gear.py` implements the paper's Stage-2, Case-C optimization. It jointly searches for:

1. Thigh length
2. Shank/calf length
3. Hip gear ratio
4. Knee gear ratio
5. Virtual linear spring gain, `K`
6. Virtual linear damping gain, `C`
7. Virtual angular spring gain, `T`

For every candidate, the code generates a MuJoCo XML model, simulates the monoped, measures jump height and positive mechanical work, and gives a combined cost to CMA-ES. CMA-ES then changes its sampling distribution so later candidates are more likely to come from promising regions.

The conceptual approach is coherent: design-dependent masses and torque limits are coupled to controller optimization, and a Jacobian-transpose controller maps a virtual-leg force into hip and knee torques.

The checked-in implementation is not yet a clean, reproducible reference implementation. Important concerns include hard-coded paths, wall-clock-dependent simulation duration, paper/code parameter mismatches, an output-logging bug, undocumented knee transmission scaling, and inefficient controller construction.

## 2. End-to-end execution flow

```text
CMA-ES proposes 7 normalized numbers
        |
        v
[thigh length, calf length, hip ratio, knee ratio, K, C, T]
        |
        +-- Look up link masses from CSV tables
        +-- Look up actuator masses from the gear-ratio table
        +-- Compute the initial pose using inverse kinematics
        +-- Rewrite a MuJoCo XML model
        +-- Set hip and knee torque limits
                |
                v
        Run the MuJoCo jump simulation
                |
                +-- Foot touching ground:
                |      virtual spring/damper controller
                |      desired foot force -> Jacobian transpose -> joint torques
                |
                +-- Foot airborne:
                       PD controller retracts the leg
                |
                v
        Measure maximum height and positive mechanical energy
                |
                v
 Cost = 0.65 x height penalty + 0.35 x energy
                |
                v
     CMA-ES updates its mean, covariance, and search radius
```

The optimized variables and code bounds are:

| Index | Code name | Physical meaning | Code bounds |
|---:|---|---|---:|
| 0 | `thigh_length` | Upper-leg link length, `l1` | 0.3-0.5 m |
| 1 | `calf_length` | Lower-leg link length, `l2` | 0.3-0.5 m |
| 2 | `hip_gear_ratio` | Hip actuator reduction | 4-8.7 |
| 3 | `knee_gear_ratio` | Knee actuator reduction | 4-8.7 |
| 4 | `K` / `ac1` | Virtual linear spring gain | 0-200 |
| 5 | `C` / `ac2` | Virtual linear damping gain | 0-10 |
| 6 | `T` / `ac3` | Virtual angular spring gain | 0-20 |

## 3. Mechanical and robotics concepts

### 3.1 The monoped and degrees of freedom

The model is a one-legged robot with:

- A base/body that can move vertically.
- A thigh connected to the base by a hip hinge.
- A shank connected to the thigh by a knee hinge.
- A hip motor and a knee motor.

The leg is called a two-degree-of-freedom, or 2-DOF, mechanism because it has two independent joint coordinates: hip angle and knee angle.

The complete MuJoCo model is more restricted than a freely moving planar robot. The base has only one vertical slide joint. It cannot translate horizontally or rotate, so this model is effectively a guided vertical hopper.

### 3.2 Torque

Torque is rotational force. Pushing a door far from its hinge creates more torque than applying the same force close to the hinge.

Torque is measured in newton-metres, or N m. In this project, hip and knee motor commands are torques.

### 3.3 Gear ratio

A 6:1 gearbox means the motor turns approximately six revolutions for one output revolution. Ideally:

- Output torque becomes roughly six times greater.
- Output speed becomes roughly six times lower.
- The gearbox adds mass.
- Reflected motor inertia increases strongly, typically with the square of the ratio.

The code assumes a motor peak torque of 5.5 N m:

```text
hip peak torque = 5.5 * hip gear ratio
```

For the knee, it adds another transmission factor:

```text
knee peak torque = 5.5 * knee gear ratio * 1.3636
```

At the nominal ratios of 6:1:

- Hip peak torque is 33 N m.
- Knee peak torque is approximately 45 N m.

The extra `1.3636` is approximately `30 / 22`, so it is probably a chain/sprocket ratio. It is not documented in the target script or paper.

### 3.4 Controller

A controller repeatedly reads the robot state and calculates motor commands.

During ground contact, this project uses a **virtual model controller**. Software behaves as though the body and foot were connected by:

- A linear spring
- A linear damper
- A torsional/angular spring

These do not have to be physical metal springs. They are mathematical rules used to generate desired forces and joint torques.

During flight, a separate PD controller pulls the hip and knee toward their initial XML pose.

### 3.5 Spring and damper

A spring creates force proportional to its displacement:

```text
F_spring = K * (l0 - l)
```

where:

- `l` is the current body-to-foot distance.
- `l0` is the desired or resting distance.
- `K` is the spring stiffness/gain.

A damper resists movement:

```text
F_damper = C * l_dot
```

where `l_dot` is the rate at which the virtual leg length changes. A car shock absorber is a familiar physical damper.

The controller combines the two:

```text
F_linear = K * (l0 - l) - C * l_dot
```

### 3.6 PD control

A proportional-derivative controller has the form:

```text
command = Kp * position_error - Kd * velocity
```

The proportional part pulls the joint toward its target. The derivative part resists fast motion and reduces oscillation.

In this code, the flight controller is:

```text
hip torque  = 100 * (0 - hip qpos)  - 10 * hip velocity
knee torque = 100 * (0 - knee qpos) - 10 * knee velocity
```

Zero here means zero displacement from the XML configuration, not a geometrically straight joint.

### 3.7 Forward and inverse kinematics

Forward kinematics asks:

> If the hip and knee angles are known, where is the foot?

Inverse kinematics asks:

> If the desired foot position is known, what hip and knee angles produce it?

The code places the initial foot at:

```text
(x, z) = (0, -0.5)
```

relative to the base. The foot is therefore directly below the base and 0.5 m lower.

For nominal link lengths `l1 = l2 = 0.4 m`, the inverse-kinematics solution is approximately:

- Hip geometric angle: 51.32 degrees
- Knee angle: -102.64 degrees

The negative knee solution is one of two mirror-image ways to reach the requested point.

The law-of-cosines part is:

```text
cos(theta2) = (x^2 + z^2 - l1^2 - l2^2) / (2*l1*l2)
```

If the absolute value of the right side is greater than one, the target is outside the leg's reachable workspace.

### 3.8 Jacobian

The Jacobian is the mathematical bridge between joint motion and foot motion.

For a small joint-angle change:

```text
[delta_x]               [delta_theta1]
[delta_z] = Jacobian *  [delta_theta2]
```

For this leg:

```text
J11 = l1*cos(theta1) + l2*cos(theta1 + theta2)
J12 =                  l2*cos(theta1 + theta2)
J21 = l1*sin(theta1) + l2*sin(theta1 + theta2)
J22 =                  l2*sin(theta1 + theta2)
```

Each entry answers a question such as:

> If this particular joint rotates a tiny amount, how much does the foot move in this direction?

The transpose maps a desired foot force into joint torques:

```text
joint_torque = transpose(Jacobian) * foot_force
```

or:

```text
[hip torque ]         [Fx]
[knee torque] = J^T * [Fz]
```

The transpose follows from conservation of virtual work:

```text
foot force * foot displacement = joint torque * joint displacement
```

A useful intuition is that the Jacobian is a configuration-dependent gearbox between Cartesian foot forces and rotary joint torques.

### 3.9 Mechanical energy

Instantaneous mechanical power is:

```text
power = torque * angular velocity
```

Energy over one simulation step is:

```text
energy_increment = torque * angular_velocity * timestep
```

The code counts only positive values. Negative mechanical power, which might represent recoverable/regenerative energy, is discarded.

This is positive mechanical work, not electrical battery energy. It omits motor efficiency, copper losses, electronics losses, gearbox friction, and regeneration efficiency.

## 4. `cma_ll_gear.py`, block by block

Target: [`cma_ll_gear.py`](final_humanoids_paper_results/Final_humanoids_results/cma_ll_gear.py)

### 4.1 Imports and paths: lines 1-22

Important imports:

- `cma`: CMA-ES optimizer.
- `numpy`: numerical calculations.
- `xml.etree.ElementTree`: XML modification.
- `Counter`: finds a most frequent simulated height.
- `multiprocessing`: evaluates eight candidates concurrently.
- `uuid`: gives each generated XML a unique name.
- `final_rcp`: runs the simulation.
- `pandas`: reads mass tables.
- `interp1d`: interpolates between tabulated data points.

Review observations:

- `numpy`, `pandas`, and `ElementTree` are imported more than once.
- `os` and `sys` are unused.
- Paths are hard-coded to another computer under `/home/stochlab/...`.
- `all_runs_file` and `mass_ik_values_file` are declared but unused.
- The current environment does not have the `cma` package installed.

The file therefore cannot run from this checkout without path/dependency changes.

### 4.2 Optimization bounds: lines 23-31

`original_bounds` defines the physical search space.

The optimizer internally works in a normalized unit cube. These bounds are used to translate between physical variables and numbers from zero to one.

There is a paper/code discrepancy:

- Paper: `K` is 5-200 and `T` is 0-50.
- Code: `K` is 0-200 and `T` is 0-20.

The checked-in script does not exactly implement Equation 13 in the paper.

### 4.3 Link-mass interpolation: lines 33-67

The code loads:

- [`thigh_03_05.csv`](final_humanoids_paper_results/Final_humanoids_results/thigh_03_05.csv)
- [`calf_03_05.csv`](final_humanoids_paper_results/Final_humanoids_results/calf_03_05.csv)

Lengths are stored in millimetres, so dividing by 1,000 converts them to metres.

`interp1d` produces functions that can estimate mass at a length between table entries. For example, a requested length of 0.437 m can be interpolated between its nearest tabulated neighbours.

Representative masses are:

- 0.4 m thigh: 1.2702 kg
- 0.4 m calf: 0.5642 kg
- 0.5 m thigh: 1.5741 kg
- 0.3 m calf: 0.4834 kg

`bounds_error=True` prevents extrapolation outside the table.

Both link tables contain a duplicate entry at exactly 400 mm, with identical mass values.

Comments in these blocks incorrectly say "Sort by gear ratio". These tables are sorted by link length.

### 4.4 Actuator mass interpolation: lines 69-84

[`comb_mass_torque.csv`](final_humanoids_paper_results/Final_humanoids_results/comb_mass_torque.csv) is the output of the paper's Stage-1 actuator optimization.

It maps gear ratio to the lightest actuator found for that ratio. Examples:

- Ratio 4.0: 1.011 kg and 22 N m
- Ratio 6.0: 1.125 kg and 33 N m
- Ratio 8.7: approximately 1.504 kg and 47.85 N m

The code uses only the ratio-to-mass part here; torque is recalculated directly from gear ratio.

Linear interpolation permits continuous ratios between table entries. Numerically this is convenient, but an interpolated ratio/mass point may not correspond to an exact manufacturable Stage-1 gearbox.

### 4.5 Normalization: lines 86-91

Normalization is:

```text
x_normalized = (x - x_min) / (x_max - x_min)
```

This makes every search variable range from zero to one. Without it, a controller gain around 100 would appear numerically much larger than a link length around 0.4 even though both may be equally important.

Denormalization reverses the calculation. It also clips inputs into `[0, 1]`, ensuring that physical values remain inside their bounds.

### 4.6 Inverse kinematics: lines 93-108

The law of cosines calculates `theta2`. `branch` chooses one of two possible bent-knee configurations. The active calls use the negative-knee branch.

The same inverse-kinematics implementation also exists in [`ik.py`](final_humanoids_paper_results/Final_humanoids_results/ik.py), creating duplication and a risk that one copy changes while the other does not.

### 4.7 XML generation: lines 109-176

`modify_link_length_and_angle()` converts a candidate design into a MuJoCo model.

#### Lines 114-118: pose and torque limits

The function:

1. Copies the proposed thigh and calf lengths.
2. Computes hip and knee torque limits.
3. Solves inverse kinematics so the foot starts 0.5 m below the base.

#### Lines 122-126: nominal geometric vectors

`thigh_vec`, `knee_pos`, and `shank_vec` calculate nominal link vectors but are never used later. They can be removed without changing current behavior.

#### Lines 128-139: mass calculation

The template XML is parsed.

Actuator masses are interpolated from their gear ratios. Because the design places both actuators proximally near the body, their sum is assigned to the base:

```python
torso_mass = hip_actuator_mass + knee_actuator_mass
```

The thigh and calf masses are obtained from their length tables.

#### Lines 140-167: geometry and pose updates

The function updates:

- Base height to 0.5 m.
- Base mass to the combined actuator mass.
- Thigh orientation, length, and mass.
- Knee location at the thigh endpoint.
- Calf orientation, length, and mass.
- Joint anchor positions.

The XML capsules point along their local x-axis. Euler rotations convert the inverse-kinematics angles into that XML coordinate convention.

#### Lines 168-175: actuator limits

The hip and knee motor `ctrlrange` values are replaced with calculated peak torques.

The simulation also clips torques in Python, so the limits are enforced in two places.

#### Unused values

`calf_offset`, `calf_x_offset`, `theta1d`, and `theta2d` are calculated but unused.

### 4.8 Old and helper functions: lines 177-231

- `get_mass_and_torque()` rounds a gear ratio to 0.1 and performs an exact CSV lookup. It is unused.
- `process_action()` rounds the three controller gains to one decimal place. It is used.
- `load_mass_ik_values()` loads an older precomputed length-pair table. It is unused.
- `round_to_nearest()` rounds to increments such as 0.005. It is unused.

Rounding controller variables makes the objective discontinuous, but CMA-ES is derivative-free and can still search such a space.

### 4.9 Candidate evaluation: lines 233-276

`get_cost()` is the main objective function.

#### Decode the candidate

The normalized vector is converted to physical variables. The three controller parameters are rounded to one decimal place.

A short UUID is added to the XML filename because eight worker processes may create models simultaneously.

#### Generate and simulate

The candidate XML is generated, then `final_rcp.run()` returns:

- Maximum vertical displacement
- Jump count
- Total positive mechanical energy
- Hip energy
- Knee energy
- Maximum-jump height
- Energy associated with the maximum jump
- Per-jump results

Although the function contains repetition and averaging machinery, `range(1)` means each candidate is simulated exactly once.

#### Mode and averaging

The intended procedure appears to be:

1. Simulate a candidate several times.
2. Find the most frequent height.
3. Average energies from runs with that height.

With one simulation, `mode_height` and `avg_energy` are simply that run's results.

If repetitions are restored, an exact floating-point mode is unreliable. Two physically identical runs can differ by tiny numerical amounts and therefore not compare equal.

#### Height meaning

`mode_height` is `d.qpos[0]`: base displacement relative to its initial 0.5 m XML position. It is not absolute world height.

Therefore:

```text
absolute height = 0.5 + mode_height
```

This explains the cost expression:

```python
height_cost = 100 * np.exp(-(0.5 + mode_height))
```

The CSV column named `Max Height` is misleading because it stores displacement rather than absolute height.

#### Objective function

The height penalty is:

```text
height_cost = 100 * exp(-absolute_height)
```

Higher jumps give a smaller penalty. The final minimized objective is:

```text
final_cost = 0.65 * height_cost + 0.35 * jump_energy
```

The coefficients are not literal percentages of physical importance. One term is a scaled exponential while the other is measured in joules.

Using the paper's values:

- Nominal: 0.887 m and 31.44 J gives an equivalent cost of about 37.78.
- Case C: 0.800 m and 15.36 J gives an equivalent cost of about 34.58.

The lower but much more energy-efficient jump therefore wins.

#### `Jump Results` logging defect

Line 264 zips one run-level height value against individual entries in `jump_results`:

```python
jump_results = [result for height, result in zip(height_values, jump_results)
                if height == mode_height]
```

The two collections represent different levels of nesting. This is why result CSVs can contain `[(np.float64(0.0), 0)]` even when a nonzero height and energy were measured.

This affects logging, not the cost used by CMA-ES.

#### Temporary files

Generated XML files are never deleted. With 1,000 generations and eight candidates, a full run can leave approximately 8,000 XML files.

### 4.10 CMA-ES setup: lines 278-287

The initial physical point is:

```text
[0.4, 0.4, 6.0, 6.0, 100, 5, 10]
```

`sigma0 = 0.1` is the initial normalized search radius. One initial standard deviation corresponds approximately to:

- 0.02 m of link length
- 0.47 of gear ratio
- 20 units of `K`
- 1 unit of `C`
- 2 units of `T`

Options request:

- Up to 1,000 generations
- Eight candidates per generation
- Seven-dimensional `[0, 1]` bounds
- Status display only every 1,000 iterations

Depending on the installed `cma` version, `seed=0` may request a generated/time-based seed rather than a reproducible literal-zero seed.

### 4.11 CMA-ES loop: lines 289-318

Each generation performs:

1. `es.ask()` samples eight candidates from a multivariate normal distribution.
2. `pool.map()` evaluates all eight in parallel.
3. `np.argmin()` identifies the lowest cost in that generation.
4. That generation's best candidate is written to CSV.
5. `es.tell()` gives every candidate and cost back to CMA-ES.

CMA-ES changes:

- Its mean, moving toward good candidates.
- Its overall search radius.
- Its covariance matrix, which learns relationships among variables.

For example, it could learn that a longer thigh works well when paired with a shorter calf and lower spring gain.

The file named `best_results_file` records the best candidate of each generation, not necessarily the global best-so-far.

## 5. MuJoCo XML model

File: [`modified_robot_400_400_60_60.xml`](final_humanoids_paper_results/Final_humanoids_results/modified_robot_400_400_60_60.xml)

Important settings are:

- Gravity: 9.81 m/s^2 downward
- Integrator: fourth-order Runge-Kutta, `RK4`
- Physics timestep: 0.002 s
- Base: one vertical slide joint
- Hip and knee: revolute hinge joints
- Default joint damping: zero
- Default armature: zero
- Links: capsule geometries
- Actuators: torque-controlled motors

The base's `qpos[0]` begins at zero even though its XML world position is 0.5 m. MuJoCo joint positions are displacements relative to the model's XML pose.

The XML contains no named `foot` site. Therefore, the controller's unused `mujoco_jacobian()` method would fail if called.

## 6. Simulation runner: `final_rcp.py`

File: [`final_rcp.py`](final_humanoids_paper_results/Final_humanoids_results/final_rcp.py)

### 6.1 Model setup: lines 16-54

The XML is loaded into:

- `m`: fixed model definition
- `d`: changing simulation state

The constraint solver uses 500 iterations and a very tight tolerance.

Energy counters and jump-detection variables are initialized. Hip and knee actuator IDs are retrieved by name.

The large commented blocks are older mass-handling code. The generated XML now contains the masses directly.

### 6.2 Initial angles: lines 95-96

Inverse kinematics is run again, and the resulting nominal angles are passed to the controller.

The controller needs them because the live MuJoCo hinge positions are offsets from the XML body's initial orientation.

### 6.3 Runtime loop: lines 99-183

The loop condition is:

```python
while time.time() - start < 8:
```

This means eight seconds of real wall-clock time, not eight seconds of simulated time.

This is a major reproducibility issue. A slower computer, a busier worker process, or heavier XML parsing executes fewer 0.002-second physics steps before the wall clock reaches eight seconds.

An offline simulation should normally terminate based on `d.time` or a fixed step count.

### 6.4 Unreachable and stale conditions: lines 108-114 and 158-159

```python
if t_elapsed < 0:
```

can never be true because elapsed time cannot be negative.

The comment says energy is calculated only after three seconds, but the actual condition is `t_elapsed >= 0`; energy is counted immediately.

### 6.5 Contact detection: lines 115-131

A new controller object is created every step. It inspects MuJoCo contacts involving the floor.

If no nonzero floor contact force is found, the robot is considered airborne.

At takeoff:

- `in_air` becomes true.
- The current base displacement begins being tracked as a jump.

### 6.6 Airborne PD controller: lines 121-125

While airborne, the hip and knee are pulled toward zero joint displacement with fixed PD gains of 100 and 10.

The paper says controller torques are applied only when the robot contacts the ground. The code instead applies PD torque in flight and includes positive airborne work in the energy measurement.

The implementation or the paper should be changed so they agree.

### 6.7 Landing and stance: lines 131-151

At the first detected ground contact after flight:

- The preceding jump is recorded.
- Jump count is incremented.
- Maximum height is updated.
- Current-jump energy is reset.

While in contact, the virtual model controller calculates desired joint torques. Hip and knee values are clipped to their physical limits.

### 6.8 Energy: lines 158-174

At every physics step:

```text
hip work  = timestep * hip torque  * hip angular velocity
knee work = timestep * knee torque * knee angular velocity
```

Only positive increments are accumulated.

The code maintains:

- Total hip energy
- Total knee energy
- Combined energy
- Energy for the current jump

### 6.9 Physics step and height: lines 175-188

`mj_step()` advances MuJoCo by 0.002 simulated seconds.

While airborne, the maximum base displacement is updated.

The later `sleep()` tries to make the simulation run in real time. This is unnecessary for an offline optimizer and contributes to the wall-clock dependence.

### 6.10 Selecting a jump: lines 190-194

If several jumps occur, the jump with the maximum height is selected, along with its associated energy.

If no jump is detected, the default is effectively:

```text
height = -1e12
energy = 1e12
```

This gives non-jumping candidates a very bad objective.

## 7. Virtual model controller: `vmc_roc_angle.py`

File: [`vmc_roc_angle.py`](final_humanoids_paper_results/Final_humanoids_results/vmc_roc_angle.py)

### 7.1 Initialization: lines 7-23

The constructor stores the MuJoCo model/state and locates the slide, hip, and knee joints.

Its defaults are:

```python
ori_l = 10.0
ori_theta = 0
```

The angular target of zero means the equivalent leg should point vertically downward.

A 10 m rest length is surprising for a leg shorter than 1 m. At a current length of 0.5 m:

```text
spring force = K * (10 - 0.5) = 9.5*K
```

As a result, `K` behaves more like a force-scaling gain than a directly interpretable physical stiffness. The paper does not document this rest length.

### 7.2 Contact forces: lines 26-62

Every MuJoCo contact involving the floor is inspected.

`mj_contactForce()` returns a six-component contact wrench. The code keeps the first three components.

The value called `body_id` is actually a geometry ID. The caller ignores it, so this naming mistake currently does not affect behavior.

Only the first returned ground contact is examined by `final_rcp.py`. If multiple simultaneous ground contacts existed, the first could be an unreliable representative.

### 7.3 State helper methods: lines 65-203

These methods obtain body names, IDs, velocities, joint positions, and the base position.

There are fragile indexing assumptions:

- Joint IDs are used directly as `qpos` and `qvel` indices.
- That happens to work for this model's three one-DOF joints but is not generally correct.
- MuJoCo's `jnt_qposadr` and `jnt_dofadr` should be used.
- `d.cvel` is sometimes indexed using a joint ID rather than a body ID.
- Spatial velocity component ordering is not handled clearly.

Several of these errors currently feed branches that are identical or terms that cancel, but they make the code unsafe to extend.

### 7.4 Gain selection: lines 151-172

`lin_sd_var()` returns:

```text
K = action[0]
C = action[1]
```

Its upward and downward branches are identical.

`rot_sd_var()` returns `T = action[2]` and chooses angular damping of zero or 50 depending on vertical velocity. That damping value is never used later, so the active implementation has a torsional spring but no effective torsional damper.

### 7.5 XML geometry readers: lines 205-318

The controller parses the XML to recover link lengths and radii.

The controller is reconstructed every simulation step, and these methods parse the XML repeatedly during one torque calculation. This creates substantial avoidable overhead.

The lengths and other fixed geometry should be read once and cached.

`torso_width_xml()` searches for nonexistent bodies named `slot` and `torso`, but it is unused.

### 7.6 Foot position and equivalent leg: lines 320-355

Forward kinematics computes:

```text
foot_x = base_x + l1*sin(theta1) + l2*sin(theta1 + theta2)
foot_z = base_z - l1*cos(theta1) - l2*cos(theta1 + theta2)
```

The virtual leg length is the Euclidean distance from base to foot.

The equivalent orientation is the angle between the body-to-foot vector and downward vertical.

Because the code uses `arccos`, the angle is always nonnegative. It loses the left/right sign of angular deflection, which could matter if the leg crosses vertical.

### 7.7 Leg-length rate: lines 357-441

These methods use the chain rule to calculate the rate of change of the body-to-foot distance:

```text
l_dot =
    (partial l / partial foot_x) * foot_x_velocity
  + (partial l / partial base_x) * base_x_velocity
  + (partial l / partial foot_z) * foot_z_velocity
  + (partial l / partial base_z) * base_z_velocity
```

The damping force is then `C * l_dot`.

### 7.8 Virtual forces: lines 445-489

The linear spring is:

```text
F_spring = K * (l0 - l)
```

The total axial force is:

```text
F_linear = F_spring - F_damper
```

The torsional spring is:

```text
torsional_torque = T * (target_angle - current_angle)
```

These are decomposed into world-frame forces using the paper's equations:

```text
Fz = -F_linear*cos(alpha)
     + (torsional_torque/l)*sin(alpha)
     - mass*gravity

Fx = -F_linear*sin(alpha)
     - (torsional_torque/l)*cos(alpha)
```

### 7.9 Jacobian and joint torque: lines 491-519

The analytical two-link Jacobian is computed and transposed:

```python
joint_torque = np.dot(jacobian_transpose, force_applied_ground)
```

This is the active conversion from virtual foot force to physical hip and knee commands.

`mujoco_jacobian()` is unused and would fail because the XML has no site named `foot`. Its returned dimensions also do not match the active 2-by-2 analytical Jacobian.

## 8. Relationship to the paper

The paper's stages map to the repository as follows:

1. **Actuator optimization:** already performed; its relevant Stage-2 output is `comb_mass_torque.csv`.
2. **Robot/control co-design:** implemented by `cma_ll_gear.py` and sibling scripts.
3. **CAD generation:** described in the paper but not invoked by this script.

The main sibling optimizers correspond to the paper's comparison cases:

- [`cma_gear.py`](final_humanoids_paper_results/Final_humanoids_results/cma_gear.py): Case A, gear ratios plus controller parameters.
- [`cma_ll.py`](final_humanoids_paper_results/Final_humanoids_results/cma_ll.py): Case B, link lengths plus controller parameters.
- [`cma_ll_gear.py`](final_humanoids_paper_results/Final_humanoids_results/cma_ll_gear.py): Case C, link lengths, gear ratios, and controller parameters.

The paper reports:

| Design | Hip ratio | Knee ratio | Thigh | Shank | K | C | T | Height | Energy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Nominal | 6 | 6 | 0.4 | 0.4 | 50 | 2.5 | 10 | 0.887 m | 31.44 J |
| Case A | 4 | 7.9 | 0.4 | 0.4 | 13.7 | 6.5 | 46.5 | 0.743 m | 15.51 J |
| Case B | 6 | 6 | 0.499 | 0.346 | 8.9 | 1.0 | 6.9 | 0.771 m | 17.02 J |
| Case C | 4 | 6 | 0.49 | 0.36 | 7.9 | 2.2 | 8.4 | 0.800 m | 15.36 J |

Case A has `T = 46.5`, which cannot be generated by the current `cma_ll_gear.py` upper bound of 20. This confirms that the checked-in repository contains artifacts from multiple experiment versions rather than one fully synchronized paper implementation.

There is also a notation inconsistency: Equation 12 orders the ratios as `gk, gh`, while the implementation and results tables use hip first and knee second.

## 9. CMA-ES explanation

CMA-ES stands for **Covariance Matrix Adaptation Evolution Strategy**. It is a derivative-free optimizer.

Rather than calculating gradients, it maintains a probability distribution over possible solutions:

1. Sample several candidates from a bell-shaped distribution.
2. Evaluate every candidate.
3. Rank candidates by cost.
4. Move the distribution toward the better candidates.
5. Change the distribution's size and shape.
6. Repeat.

The covariance matrix describes the distribution's shape and correlations. It lets the optimizer discover coupled directions, such as increasing one link length while decreasing another.

This is useful here because:

- MuJoCo contact dynamics are nonsmooth.
- Controller gains are rounded.
- Gear and mass relationships are table-based.
- An analytical derivative of the entire simulation would be difficult to obtain.

## 10. Prioritized review findings

### Critical or high impact

1. **The target script is not portable or runnable as checked out.** It uses hard-coded `/home/stochlab/...` paths, references a missing `design_xmls` directory, and the current environment lacks the `cma` package.

2. **Simulation duration depends on CPU speed and contention.** `final_rcp.py` runs for eight wall-clock seconds rather than a fixed number of MuJoCo steps. Parallel load can therefore change objective values.

3. **The code and paper use different parameter bounds.** In particular, the paper permits `T <= 50`, while the target script permits only `T <= 20`.

4. **The paper's ground-contact-only control statement differs from the implementation.** Airborne PD torque is applied, and its positive work is counted.

5. **The extra knee transmission factor of `1.3636` is undocumented.** It changes a nominal 6:1 knee actuator from 33 N m motor/gearbox torque to approximately 45 N m joint torque.

6. **The virtual spring's rest length is hard-coded to 10 m.** This makes `K` difficult to interpret as an ordinary physical spring stiffness and should be justified or reformulated.

### Correctness and data-quality concerns

7. **The `Jump Results` CSV value is corrupted by a nesting/zip bug.** The primary cost is unaffected, but logged diagnostic data are unreliable.

8. **The `Max Height` CSV column stores displacement, not absolute height.** Readers can misinterpret values by 0.5 m.

9. **Only one simulation repetition is performed.** The surrounding mode/averaging logic gives a false impression of robustness testing.

10. **Exact floating-point modes would be unreliable if repetitions were restored.** Heights should be binned, rounded, or statistically summarized.

11. **The equivalent angular displacement loses its sign.** `arccos` cannot distinguish left from right of vertical.

12. **MuJoCo joint/body indexing is fragile.** Joint IDs should not generally be used directly as `qpos`, `qvel`, or body indices.

### Reliability, performance, and maintainability

13. **Eight worker processes append to one CSV without a lock.** Complete rows often survive, but concurrent append behavior should not be relied upon for experiment integrity.

14. **There is no `if __name__ == "__main__":` guard.** Multiprocessing can recursively start the optimizer on spawn-based platforms.

15. **Generated XML files are never removed.** A full run can leave thousands of files.

16. **The controller is reconstructed every step and repeatedly reparses XML.** Fixed geometry should be loaded once and cached.

17. **The simulation deliberately sleeps in an offline optimization.** Removing real-time throttling would make experiments much faster and allow fixed simulated-time termination.

18. **The generation-best CSV is not a global-best log.** Its name can mislead downstream analysis.

19. **The optimizer does not explicitly save the final CMA-ES state/result.** It relies on per-generation CSV files.

20. **Several imports, variables, helper functions, and commented blocks are dead code.** They make the active behavior harder to audit.

21. **Inverse kinematics is duplicated.** There should be one tested implementation.

22. **The seeded Case-C script is currently a test stub.** [`seed_cma_ll_gear.py`](final_humanoids_paper_results/Final_humanoids_results/seed_cma_ll_gear.py) hard-codes 0.4 m links, 6:1 ratios, and `[20, 2.5, 10]`, so it does not genuinely optimize its seven declared variables.

## 11. Recommended correction order

A sensible order for improving the implementation is:

1. Replace all hard-coded absolute paths with `pathlib.Path(__file__)`-relative paths.
2. Add a main guard and create required output directories explicitly.
3. Run simulations for a fixed simulated duration or fixed number of steps.
4. Remove `time.sleep()` from offline optimization.
5. Synchronize all bounds, ratio ordering, cost weights, and torque equations with the paper.
6. Document or parameterize the knee chain ratio.
7. Decide whether airborne PD control and its energy belong in the published objective.
8. Fix jump-result logging and rename displacement/height columns accurately.
9. Cache XML-derived geometry and reuse one controller during a simulation.
10. Correct MuJoCo address and velocity indexing.
11. Protect or centralize CSV writes.
12. Delete temporary XML files after each evaluation, preferably in a `finally` block.
13. Add deterministic tests for IK, forward kinematics, Jacobian finite differences, energy integration, and one fixed candidate simulation.
14. Store experiment metadata: code revision, dependency versions, seed, bounds, timestep, simulated duration, and complete final CMA-ES result.

## 12. Suggested tests

### Inverse kinematics round trip

For several link lengths and reachable targets:

1. Calculate angles using inverse kinematics.
2. Feed those angles into forward kinematics.
3. Confirm that the reconstructed foot position matches the target.

### Jacobian finite-difference test

1. Compute the analytical Jacobian.
2. Perturb each joint angle by a very small amount.
3. Recompute foot position.
4. Confirm that numerical position derivatives match the Jacobian columns.

### Energy test

Use constant torque and angular velocity for a known number of timesteps. Confirm:

```text
energy = torque * angular_velocity * total_time
```

Also verify that negative power is intentionally ignored.

### Fixed simulated-time test

Run the same candidate under different CPU loads. Once the loop is based on `d.time`, the resulting trajectory, height, and energy should be identical or extremely close.

### XML generation test

For a known candidate, verify:

- Link endpoints match requested lengths.
- Masses match interpolation-table values.
- Base world height is 0.5 m.
- The foot initially lies at `(0, -0.5)` relative to the base.
- Hip and knee control ranges match their formulas.

## 13. Final interpretation

The code is optimizing more than robot dimensions. It is searching for a matched combination of:

- Geometry: how the leg is shaped
- Actuation: how much torque and mass each transmission adds
- Control: how strongly the software-generated spring and damper push

That interaction is the central meaning of **co-design**. A mechanically strong design is not automatically optimal if its gearbox is heavy, and a good controller cannot overcome unsuitable link geometry or insufficient torque. CMA-ES searches all of these choices together.

The research idea is strong, but experiment reproducibility presently depends on undocumented implementation details. Addressing the prioritized findings above would make the repository much easier to validate, maintain, and share as the implementation accompanying the paper.
