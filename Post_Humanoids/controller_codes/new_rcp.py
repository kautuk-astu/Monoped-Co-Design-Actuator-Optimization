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
from scipy.interpolate import interp1d

# Set fixed random seed for deterministic behavior
np.random.seed(0)
random.seed(0)

def run(xml_path, action, ik_value, hip_peak_torque, knee_peak_torque):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    af = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/comb_mass_torque.csv')

# Sort by gear ratio to avoid interpolation issues
    af = af.sort_values(by='gear_ratio')

    # Extract gear ratio and mass
    gear_ratios = af['gear_ratio'].values
    act_masses = af['mass'].values

    # Create a linear interpolation function without extrapolation
    act_interp_func = interp1d(
        gear_ratios,
        act_masses,
        kind='linear',
        bounds_error=True  # This raises an error if input is out of bounds
    )

    # hip_actuator_mass = act_interp_func(4.000146190606526)
    # knee_actuator_mass = act_interp_func(5.999997890823783)
    
    hip_actuator_mass = act_interp_func(6)
    knee_actuator_mass = act_interp_func(6)
    
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
    l1n = 0.4
    l2n = 0.4
    dft = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/thigh_03_05.csv')

# Sort by gear ratio to avoid interpolation issues
    dft = dft.sort_values(by='Link Length (mm)')

    # Extract gear ratio and mass
    thigh_lengths = dft['Link Length (mm)'].values/1000
    thigh_masses = dft['Calculated Mass (kg)'].values

    # Create a linear interpolation function without extrapolation
    thigh_interp_func = interp1d(
        thigh_lengths,
        thigh_masses,
        kind='linear',
        bounds_error=True  # This raises an error if input is out of bounds
    )


    dfc = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/calf_03_05.csv')

    # Sort by gear ratio to avoid interpolation issues
    dfc = dfc.sort_values(by='Link Length (mm)')

    # Extract gear ratio and mass
    calf_lengths = dfc['Link Length (mm)'].values/1000
    calf_masses = dfc['Calculated Mass (kg)'].values

    # Create a linear interpolation function without extrapolation
    calf_interp_func = interp1d(
        calf_lengths,
        calf_masses,
        kind='linear',
        bounds_error=True  # This raises an error if input is out of bounds
    )

    thigh_link_mass = thigh_interp_func(l1n) 
    calf_link_mass = calf_interp_func(l2n)
    #mj.mj_setTotalmass(m, hip_actuator_mass + knee_actuator_mass + thigh_link_mass + calf_link_mass)
    print(hip_actuator_mass+ knee_actuator_mass+ thigh_link_mass+ calf_link_mass)
    #exit()

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
    store_com = []   
    store_com_vel = []
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
        com_z = d.subtree_com[1][2]
        store_com.append(com_z)
        #com_vel_z = d.subtree_linvel[1][2]
        
        mj.mj_step(m, d)
        masses = m.body_mass
        velocities = d.cvel[:, :3]   # linear part of each body’s CoM velocity (Nx3)

        total_mass = np.sum(masses)
        com_vel = np.sum(masses[:, None] * velocities, axis=0) / total_mass
        com_vel_z = com_vel[0]

        store_com_vel.append(com_vel_z)
        
        
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
    max_jump = max(jump_results, key=lambda x: x[0], default=(-1e12,1e12))
    max_jump_height, max_jump_energy = max_jump
    print(time.time() - start)
    #command to get total mass
    print(mj.mj_getTotalmass(m))

    return max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store,time_store, store_com, store_com_vel


# Define Action & XML Path
action = np.array([50,2.5,10])
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/main xmls/old_std.xml'
max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store, time_store, store_com, store_com_vel = run(xml_path, action,-0.5, 6*5.5,6*5.5*1.3636)
data = {
    "omega2_store": omega2_store,
    "omega1_store": omega1_store,
    "v0_store": v0_store,
    "theta1_store": theta1_store,
    "theta2_store": theta2_store,
    "base_store": base_store,
    "torque1_store": torque1_store,
    "torque2_store": torque2_store,
    "time_store": time_store,
    "store_com": store_com,
    "store_com_vel": store_com_vel
}

# Create DataFrame and save to CSV
df = pd.DataFrame(data)
df.to_csv("/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/nodampnominal.csv", index=False)

# Print Results
print(max_jump_height, max_jump_energy)
height_cost = 100*np.exp(-(0.5+max_height_after_control))
final_cost = 0.65*(height_cost) + 0.35*(max_jump_energy)
print(final_cost)



