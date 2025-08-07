import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random
import vmc_roc_angle as vmc_ra_angle
import pandas as pd
import glfw

# Set fixed random seed for deterministic behavior
np.random.seed(0)
random.seed(0)

def run(xml_path, action, ik_value, hip_peak_torque, knee_peak_torque):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10
    total_energy = 0
    hip_energy = 0
    knee_energy = 0
    jump_energy = 0  
    jump_results = []  
    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False  
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id    
    l1n = 0.470947604311891
    l2n = 0.3860421694932592
    theta1, theta2 = ik.inverse_kinematics(0, ik_value, l1n, l2n)
    omega1_store = []
    omega2_store = []
    v0_store = []
    theta1_store = []
    theta2_store = []
    base_store = []
    torque1_store = []
    torque2_store = []
    time_store = []    
    theta1d = np.rad2deg(theta1)
    theta2d = np.rad2deg(theta2)
    start = time.time() 
    while time.time() - start < 8:
        step_start = time.time()
        t_elapsed = time.time() - start        
        if t_elapsed < 0:
            hip_error = 0 - d.qpos[1]
            knee_error = 0 - d.qpos[2]
            hip_torque = 100 * hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
        else:
            controller = vmc_ra_angle.Controller(xml_path, m, d, theta1, theta2)
            contact_force = controller.get_ground_contact_forces()
            forces = np.array([0, 0, 0])  
            if len(contact_force) > 0:
                forces = contact_force[0][0]
            if np.all(forces == 0):
                hip_error = 0 - d.qpos[1]
                knee_error = 0 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
                
                if not in_air:
                    in_air = True
                    current_jump_max = d.qpos[0]
                    #jump_energy = 0
            else:
                if in_air:
                    in_air = False
                    jump_count += 1
                    jump_results.append((current_jump_max, jump_energy))
                    max_height_after_control = max(max_height_after_control, current_jump_max)
                    current_jump_max = d.qpos[0]
                
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
                hip_torque = np.clip(joint_torque[0], -hip_peak_torque, hip_peak_torque)
                knee_torque = np.clip(joint_torque[1], -knee_peak_torque, knee_peak_torque)
        
        # Apply Control Torques
        hip_torque = np.clip(hip_torque, -hip_peak_torque, hip_peak_torque)
        knee_torque = np.clip(knee_torque, -knee_peak_torque, knee_peak_torque)
        
        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque

        
        # Calculate energy only after 3 sec
        if t_elapsed >= 0:
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
        
        # Step simulation
        omega1_store.append(d.qvel[1])
        base_store.append(d.qpos[0])
        torque1_store.append(hip_torque)
        torque2_store.append(knee_torque)
        theta1_store.append(d.qpos[1])
        theta2_store.append(d.qpos[2])
        omega2_store.append(d.qvel[2])
        v0_store.append(d.qvel[0])
        time_store.append(t_elapsed)
        mj.mj_step(m, d)
        
        
        if t_elapsed >= 0 and in_air:
            current_jump_max = max(current_jump_max, d.qpos[0])
        
        # Respect simulation timestep
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    # In case the simulation ends while still in air, record the last jump.
    if in_air:
        jump_results.append((current_jump_max, jump_energy))
        max_height_after_control = max(max_height_after_control, current_jump_max)

    # Obtain the jump with max height and its energy
    max_jump = max(jump_results, key=lambda x: x[0], default=(-0.8,1e6))
    max_jump_height, max_jump_energy = max_jump
    print(time.time() - start)

    return max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store,time_store


# Define Action & XML Path
action = np.array([138.9,4.6,7.9])
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/xmls/seed_10_comb.xml'
max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store, time_store= run(xml_path, action,-0.5, 4.0005439106249*5.5,7.4519014370975984*5.5*1.3636)
data = {
    "omega2_store": omega2_store,
    "omega1_store": omega1_store,
    "v0_store": v0_store,
    "theta1_store": theta1_store,
    "theta2_store": theta2_store,
    "base_store": base_store,
    "torque1_store": torque1_store,
    "torque2_store": torque2_store,
    "time_store": time_store
}

# Create DataFrame and save to CSV
df = pd.DataFrame(data)
df.to_csv("/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/trajectories/seed_10.csv", index=False)

# Print Results
print(max_jump_height, max_jump_energy)
height_cost = 100*np.exp(-(0.5+max_height_after_control))
final_cost = 0.65*(height_cost) + 0.35*(max_jump_energy)
print(final_cost)



