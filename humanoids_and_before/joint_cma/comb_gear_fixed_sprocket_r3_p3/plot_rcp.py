import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random

def run(xml_path, action, mass, ik_value):

    # Set fixed seed for reproducibility
    np.random.seed(0)
    random.seed(0)

    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, mass)

    # Reset and configure simulation
    mj.mj_resetData(m, d)
    m.opt.iterations = 500
    m.opt.tolerance = 1e-10

    # Initialize controller
    controller = vmc_ra.Controller(xml_path, m, d)

    # Get leg lengths and joint angles
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(ik_value, 0, l1, l2)

    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0

    hip_actuator_id = m.actuator('torque1').id
    knee_actuator_id = m.actuator('torque2').id

    # Energy and torque tracking
    total_energy = hip_energy = knee_energy = jump_energy = 0
    torque1_store = []
    torque2_store = []
    jump_results = []

    # Jump tracking
    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False

    # Height recording for first jump
    recording_jump = False
    t0_jump = 0
    initial_base_height = None
    base_heights_first_jump = []
    timestamps_first_jump = []
    peak_reached = False
    jump_phase_started = False

    # Run simulation
    start = time.time()
    while time.time() - start < 10:
        step_start = time.time()
        t_elapsed = time.time() - start

        # PD control before 3s
        if t_elapsed < 3:
            hip_error = theta1 - d.qpos[1]
            knee_error = theta2 - d.qpos[2]
            hip_torque = 100 * hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
        else:
            contact_force = controller.get_ground_contact_forces()
            forces = contact_force[0][0] if contact_force else np.zeros(3)
            on_ground = not np.all(forces == 0)

            if not jump_phase_started and on_ground:
                # First ground contact after 3s
                jump_phase_started = True
                recording_jump = True
                t0_jump = t_elapsed
                initial_base_height = d.qpos[0]  # height before adding +1
                base_heights_first_jump = [d.qpos[0] + 1]
                timestamps_first_jump = [0.0]
            elif recording_jump:
                height_now = d.qpos[0] + 1
                time_now = t_elapsed - t0_jump
                base_heights_first_jump.append(height_now)
                timestamps_first_jump.append(time_now)

                if (d.qpos[0] <= initial_base_height) and in_air:
                    recording_jump = False  # Stop recording once we land below/at starting height

            in_air = not on_ground

            # Torque generation logic
            if np.all(forces == 0):
                hip_error = theta1 - d.qpos[1]
                knee_error = theta2 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]

                if not in_air:
                    in_air = True
                    current_jump_max = d.qpos[0]
                    jump_energy = 0
            else:
                if in_air:
                    in_air = False
                    jump_count += 1
                    jump_results.append((current_jump_max, jump_energy))
                    max_height_after_control = max(max_height_after_control, current_jump_max)
                    current_jump_max = d.qpos[0]
                    jump_energy = 0

                controller.lin_sd_var(action)
                controller.rot_sd_var(action)
                controller.linear_spring_force(action)
                controller.linear_damper_force(action)
                controller.torsional_spring_force(action)
                controller.total_linear_force(action)
                controller.fz_by_ee(action)
                controller.fx_by_ee(action)
                controller.force_applied_ground(action)
                joint_torque = controller.joint_torque(action)
                hip_torque = np.clip(joint_torque[0], -30.8, 30.8)
                knee_torque = np.clip(joint_torque[1], -65.24826, 65.24826)

        # Apply torques
        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        torque1_store.append(hip_torque)
        torque2_store.append(knee_torque)

        # Energy accounting
        if t_elapsed >= 3:
            ha = m.opt.timestep * hip_torque * d.qvel[1]
            ka = m.opt.timestep * knee_torque * d.qvel[2]
            if ha > 0:
                hip_energy += ha
                total_energy += ha
                jump_energy += ha
            if ka > 0:
                knee_energy += ka
                total_energy += ka
                jump_energy += ka
        if t_elapsed >= 3:
            if not recording_jump:
                # Start tracking when we hit 3s mark
                recording_jump = True
                t0_jump = t_elapsed
                initial_base_height = d.qpos[0]
                base_heights_first_jump = []
                timestamps_first_jump = []
                peak_reached = False

        # Track the height relative to 3s mark
            current_height = d.qpos[0]
            base_heights_first_jump.append(current_height + 1)  # add 1 as reference
            timestamps_first_jump.append(t_elapsed - t0_jump)

            # Detect when height has gone above initial and returned
            if current_height > initial_base_height + 0.01:
                peak_reached = True

            if peak_reached and current_height <= initial_base_height + 0.0000005:
                break

        # Step simulation
        mj.mj_step(m, d)

        # Update max height while in air
        if t_elapsed >= 3 and in_air:
            current_jump_max = max(current_jump_max, d.qpos[0])

        # Sync with real-time
        time_left = m.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)

    if in_air:
        jump_results.append((current_jump_max, jump_energy))
        max_height_after_control = max(max_height_after_control, current_jump_max)

    # Final output
    max_jump = max(jump_results, key=lambda x: x[0], default=(None, None))
    max_jump_height, max_jump_energy = max_jump

    return (
        max_height_after_control,
        jump_count,
        total_energy,
        hip_energy,
        knee_energy,
        max_jump_height,
        max_jump_energy,
        torque1_store,
        torque2_store,
        timestamps_first_jump,
        base_heights_first_jump,
    )
import matplotlib.pyplot as plt
action = np.array([62.3,2.6,12.3])
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_03_05/modified_robot_500_500.xml'
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/modified_robot_437_346_55_86.xml'

# Call the run function and get the data
_, _, _, _, _, _, max_jump_energy, _, _, time_stamps, heights = run(xml_path, action, 4.492614814814815, 0.6)
print(max_jump_energy)

# Plot base height over time during first jump
plt.plot(time_stamps, heights)
plt.title("Base Height During First Jump (Starting at t=3s)")
plt.xlabel("Time since 3s (s)")
plt.ylabel("Base Height (m)")
plt.grid(True)
plt.show()