import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random
def run(xml_path, action, mass, ik_value):
    

    # Set fixed random seed
    np.random.seed(0)
    random.seed(0)

    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, mass)
    mj.mj_resetData(m, d)

    m.opt.iterations = 500
    m.opt.tolerance = 1e-10

    controller = vmc_ra.Controller(xml_path, m, d)
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(ik_value, 0, l1, l2)

    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0

    hip_actuator_id = m.actuator('torque1').id
    knee_actuator_id = m.actuator('torque2').id

    total_energy = 0
    hip_energy = 0
    knee_energy = 0

    torque1_store = []
    torque2_store = []

    base_heights_second_jump = []
    timestamps_second_jump = []

    first_jump_completed = False
    after_first_return = False
    recording_jump = False
    peak_reached = False

    initial_base_height = None
    t0_jump = 0

    start = time.time()

    while time.time() - start < 15:
        step_start = time.time()
        t_elapsed = time.time() - start

        contact_force = controller.get_ground_contact_forces()
        on_ground = len(contact_force) > 0 and not np.all(contact_force[0][0] == 0)

        current_height = d.qpos[0]

        # 1. Initialize reference height after 3s when on ground
        if t_elapsed > 3 and initial_base_height is None and on_ground:
            initial_base_height = current_height
            print(f"Initial base height set: {initial_base_height:.3f}")

        # 2. Check and log key jump stages
        if initial_base_height is not None and t_elapsed > 3:
            h = current_height

            if not first_jump_completed:
                if abs(h - initial_base_height) < 0.01 and on_ground:
                    first_jump_completed = True
                    print(f"[{t_elapsed:.2f}s] First jump completed.")

            elif first_jump_completed and not after_first_return:
                if h > initial_base_height + 0.03:
                    after_first_return = True
                    print(f"[{t_elapsed:.2f}s] Started rising for second jump.")

            elif after_first_return and not recording_jump:
                if abs(h - initial_base_height) < 0.01 and on_ground:
                    recording_jump = True
                    t0_jump = time.time()
                    print(f"[{t_elapsed:.2f}s] Second jump recording started.")

            elif recording_jump:
                base_heights_second_jump.append(h + 1)
                timestamps_second_jump.append(time.time() - t0_jump)

                if len(base_heights_second_jump) >= 2:
                    if not peak_reached and base_heights_second_jump[-1] < base_heights_second_jump[-2]:
                        peak_reached = True
                        print(f"[{t_elapsed:.2f}s] Second jump peak reached.")

                if peak_reached and abs(h - initial_base_height) < 0.01 and on_ground:
                    print(f"[{t_elapsed:.2f}s] Second jump ended.")
                    break

        # Torque decisions
        if t_elapsed < 3:
            hip_error = theta1 - d.qpos[1]
            knee_error = theta2 - d.qpos[2]
            hip_torque = 100 * hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
        else:
            if on_ground:
                hip_error = theta1 - d.qpos[1]
                knee_error = theta2 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
            else:
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

        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        torque1_store.append(d.ctrl[hip_actuator_id])
        torque2_store.append(d.ctrl[knee_actuator_id])

        if t_elapsed > 3:
            ha = m.opt.timestep * hip_torque * d.qvel[1]
            ka = m.opt.timestep * knee_torque * d.qvel[2]
            if ha > 0:
                hip_energy += ha
                total_energy += ha
            if ka > 0:
                knee_energy += ka
                total_energy += ka

        mj.mj_step(m, d)
        time_to_wait = m.opt.timestep - (time.time() - step_start)
        if time_to_wait > 0:
            time.sleep(time_to_wait)

    return (
        None, None, total_energy, hip_energy, knee_energy,
        None, None, torque1_store, torque2_store,
        timestamps_second_jump, base_heights_second_jump
    )

import matplotlib.pyplot as plt
action = np.array([62.3,2.6,12.3])
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_03_05/modified_robot_500_500.xml'
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/modified_robot_437_346_55_86.xml'


import matplotlib.pyplot as plt

# Run simulation
_, _, _, _, _, _, _, _, _, time_stamps, heights = run(xml_path, action, 4.492614814814815, 0.6)

# Plot if there's data
if len(heights) > 0:
    plt.figure(figsize=(8, 4))
    plt.plot(time_stamps, heights, label='Second Jump Height')
    plt.xlabel("Time (s)")
    plt.ylabel("Base Height (m)")
    plt.title("Second Jump Height vs Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("No height data recorded for second jump.")

