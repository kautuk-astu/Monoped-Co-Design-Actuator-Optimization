import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random

# Set fixed random seed for deterministic behavior
np.random.seed(0)
random.seed(0)

def run(xml_path, action, mass, ik_value):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, mass)
    
    # Reset simulation state for consistency
    mj.mj_resetData(m, d)
    
    # Increase solver precision for more stable results
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10
    
    # Energy variables
    total_energy = 0
    hip_energy = 0
    knee_energy = 0
    jump_energy = 0  
    jump_results = []  

    # Tracking jump state
    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False  
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id
    
    # Initialize Controller
    controller = vmc_ra.Controller(xml_path, m, d)
    
    # Compute Initial Joint Angles
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(ik_value, 0, l1, l2)
    
    # Set Initial Joint Angles
    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0
    
    # Start Simulation
    start = time.time()
    while time.time() - start < 15:
        step_start = time.time()
        t_elapsed = time.time() - start
        
        # Compute Torques
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
                hip_torque = np.clip(joint_torque[0], -50, 50)
                knee_torque = np.clip(joint_torque[1], -50, 50)
        
        # Apply Control Torques
        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        
        # Calculate energy only after 3 sec
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
        
        # Step simulation
        mj.mj_step(m, d)
        
        # Update current jump max height if in air
        if t_elapsed >= 3 and in_air:
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
    max_jump = max(jump_results, key=lambda x: x[0], default=(None, None))
    max_jump_height, max_jump_energy = max_jump

    return max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy
