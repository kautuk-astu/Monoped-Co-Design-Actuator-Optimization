import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random

np.random.seed(0)
random.seed(0)

def run(xml_path, action, mass, ik_value):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, mass)

    mj.mj_resetData(m, d)
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10

    total_energy = 0
    hip_energy = 0
    knee_energy = 0
    jump_energy = 0
    hip_energy_jump = 0
    knee_energy_jump = 0
    jump_results = []

    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id

    controller = vmc_ra.Controller(xml_path, m, d)

    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(ik_value, 0, l1, l2)

    torque1_store = []
    torque2_store = []

    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0

    start = time.time()
    while True:
        step_start = time.time()
        t_elapsed = time.time() - start

        if t_elapsed < 3:
            hip_error = theta1 - d.qpos[1]
            knee_error = theta2 - d.qpos[2]
            hip_torque = 100 * hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
        else:
            contact_force = controller.get_ground_contact_forces()
            forces = np.array([0, 0, 0])  
            if len(contact_force) > 0:
                forces = contact_force[0][0]

            if np.all(forces == 0):  # in air
                hip_error = theta1 - d.qpos[1]
                knee_error = theta2 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]

                if not in_air:
                    in_air = True
                    current_jump_max = d.qpos[0]
                    jump_energy = 0
                    hip_energy_jump = 0 
                    knee_energy_jump = 0
            else:  # on ground
                if in_air:
                    in_air = False
                    jump_count += 1
                    jump_results.append((current_jump_max, jump_energy))
                    max_height_after_control = max(max_height_after_control, current_jump_max)

                    if jump_count == 2:
                        break  # stop simulation after two jumps

                    current_jump_max = d.qpos[0]
                    jump_energy = 0
                    hip_energy_jump = 0
                    knee_energy_jump = 0

                # Apply controller torques
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
                hip_torque = np.clip(joint_torque[0], -30.25, 30.25)
                knee_torque = np.clip(joint_torque[1], -64.49828, 64.49828)

        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        torque1_store.append(d.ctrl[hip_actuator_id])
        torque2_store.append(d.ctrl[knee_actuator_id])

        if t_elapsed >= 3:
            hip_power = hip_torque * d.qvel[1]
            knee_power = knee_torque * d.qvel[2]

            if hip_power > 0:
                e_hip = hip_power * m.opt.timestep
                hip_energy += e_hip
                total_energy += e_hip
                hip_energy_jump += e_hip
                jump_energy += e_hip

            if knee_power > 0:
                e_knee = knee_power * m.opt.timestep
                knee_energy += e_knee
                total_energy += e_knee
                knee_energy_jump += e_knee
                jump_energy += e_knee

        mj.mj_step(m, d)

        if t_elapsed >= 3 and in_air:
            current_jump_max = max(current_jump_max, d.qpos[0])

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    if in_air:
        jump_results.append((current_jump_max, jump_energy, hip_energy_jump, knee_energy_jump))

        max_height_after_control = max(max_height_after_control, current_jump_max)

    max_jump = max(jump_results, key=lambda x: x[0], default=(None, None))
    max_jump_height, max_jump_energy = max_jump

    return (
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results, torque1_store, torque2_store
    )
action = np.array([62.3,2.6,12.3])
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_03_05/modified_robot_500_500.xml'
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/modified_robot_437_346_55_86.xml'
max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results, torque1_store, torque2_store = run(xml_path, action, 4.492614814814815, 0.6)
#print(f"Total Energy: {e_total:.2f}, Max Height: {h_max:.3f}, Jumps: {n_jumps}")
#print(f"Max Jump Energy: {e_max:.2f}, Hip Energy: {e_hip:.2f}, Knee Energy: {e_knee:.2f}, Max Height After Control: {max_h:.3f}")
print(f"Max Height After Control: {max_height_after_control:.3f}")
print(f"Jump Count: {jump_count}")
print(f"jump_results: {jump_results}")

