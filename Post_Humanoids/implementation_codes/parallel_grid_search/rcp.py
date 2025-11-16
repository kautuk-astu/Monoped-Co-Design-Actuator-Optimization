import numpy as np
import mujoco as mj
import time
import sys, os
import ik as ik
import vmc_roc_action as vmc_ra_angle
import pandas as pd
from scipy.interpolate import interp1d

# Set fixed random seed for deterministic behavior
np.random.seed(0)
#random.seed(0)

def run(xml_path, action, thigh_length, calf_length, ik_value, hip_peak_torque, knee_peak_torque):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)        
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10
    
    # Energy tracking per jump
    jump_energy_current = 0
    jump_joule_current = 0
    jump_mech_current = 0
    
    total_energy = 0
    hip_energy = 0
    knee_energy = 0
    jump_results = []  
    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False  
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id    
    l1n = thigh_length
    l2n = calf_length
    theta1, theta2 = ik.inverse_kinematics(0, ik_value, thigh_length, calf_length)  
    
    # Start Simulation
    start = time.time()

    while time.time() - start < 8:

        step_start = time.time()
        t_elapsed = time.time() - start
        
        # Compute Torques
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
            else:
                if in_air:
                    in_air = False
                    jump_count += 1
                    # Store jump results with separate energy values
                    jump_results.append({
                        'height': current_jump_max,
                        'total_energy': jump_energy_current,
                        'joule_energy': jump_joule_current,
                        'mech_energy': jump_mech_current
                    })
                    max_height_after_control = max(max_height_after_control, current_jump_max)
                    current_jump_max = d.qpos[0]
                    # Reset energy counters for new jump
                    jump_energy_current = 0
                    jump_joule_current = 0
                    jump_mech_current = 0
                    
                # if jump_count == 1:
                #     break
                
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
            joule_increment = (0.0955*hip_torque*hip_torque + 0.0955*knee_torque*knee_torque)*m.opt.timestep
            
            # Update cumulative totals (for entire simulation)
            if ha > 0:
                hip_energy += ha
                total_energy += ha
            if ka > 0:
                knee_energy += ka
                total_energy += ka
            
            # Update current jump energies only during GROUND CONTACT (when not in air)
            if not in_air and t_elapsed >= 0:  # Only during ground contact phase
                jump_joule_current += joule_increment
                if ha > 0:
                    jump_mech_current += ha
                    jump_energy_current += ha
                if ka > 0:
                    jump_mech_current += ka
                    jump_energy_current += ka
                # Also add joule energy to total jump energy
                jump_energy_current += joule_increment
        
        # Step simulation
        mj.mj_step(m, d)
        if t_elapsed >= 0 and in_air:
            current_jump_max = max(current_jump_max, d.qpos[0])
        
        # Respect simulation timestep
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    # In case the simulation ends while still in air, record the last jump.
    if in_air:
        jump_results.append({
            'height': current_jump_max,
            'total_energy': jump_energy_current,
            'joule_energy': jump_joule_current,
            'mech_energy': jump_mech_current
        })
        max_height_after_control = max(max_height_after_control, current_jump_max)

    # Find the jump with maximum height and extract its energy values
    if jump_results:
        max_jump = max(jump_results, key=lambda x: x['height'])
        max_jump_height = max_jump['height']
        max_jump_energy = max_jump['total_energy']
        max_jump_joule = max_jump['joule_energy']
        max_jump_mech = max_jump['mech_energy']
    else:
        max_jump_height = -1e12
        max_jump_energy = 1e12
        max_jump_joule = 1e12
        max_jump_mech = 1e12

    return (max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, 
            max_jump_height, max_jump_energy, max_jump_joule, max_jump_mech, jump_results)