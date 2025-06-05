import numpy as np
import mujoco as mj
import vmc_mujoco as vmc
import time
import mujoco.viewer as mj_viewer
import len_rad_id as lri
import vmc_hopper_roc as vmc_hr
import vmc_roc_damper as vmc_rd
import vmc_roc_action as vmc_ra
import final_vmc as fvmc
import ik as ik
import csv
import random

# Set fixed random seed for deterministic behavior
np.random.seed(0)
random.seed(0)

def run(xml_path, action):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, 2.9)
    
    # Reset simulation state for consistency
    mj.mj_resetData(m, d)
    max_height_after_control = float('-inf')

    # Increase solver precision for more stable results
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10
    
    # Data Storage
    base_height_store = []
    grf_z = []
    height_reward = []
    base_vel_store = []
    hip_torque_store = []
    knee_torque_store = []

    # Get Joint & Actuator IDs
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id
    
    # Initialize Controller
    controller = vmc_ra.Controller(xml_path, m, d)
    
    # Compute Initial Joint Angles
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(0.6, 0, l1, l2)   
    print("theta1:", theta1)
    print("theta2:", theta2)

    # Set Initial Joint Angles
    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0

    # Initialize Energy Variables
    total_energy = 0
    knee_energy = 0
    hip_energy = 0
    
    # Start Simulation with Viewer
    with mj_viewer.launch_passive(m, d) as viewer:
        start = time.time()
        while time.time() - start < 15:
            step_start = time.time()
            
            # Compute Torques
            if time.time() - start < 3:
                # PD Control (First 3 Sec)
                hip_error = theta1 - d.qpos[1]
                knee_error = theta2 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
            else:
                # Check Ground Contact
                contact_force = controller.get_ground_contact_forces()
                forces = np.array([0, 0, 0])  # Default to no contact
                if len(contact_force) > 0:
                    forces = contact_force[0][0]

                if np.all(forces == 0):
                    # No contact, use PD control
                    hip_error = theta1 - d.qpos[1]
                    knee_error = theta2 - d.qpos[2]
                    hip_torque = 100 * hip_error - 10 * d.qvel[1]
                    knee_torque = 100 * knee_error - 10 * d.qvel[2]
                else:
                    # Contact detected, use controller
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

            # Compute Energy (Before mj_step)
            ha = m.opt.timestep * hip_torque * d.qvel[1]
            ka = m.opt.timestep * knee_torque * d.qvel[2]
            if ha > 0:
                hip_energy += ha
                total_energy += ha
            if ka > 0:
                knee_energy += ka
                total_energy += ka
            
            # Step Simulation
            mj.mj_step(m, d)

            # Update Viewer
            viewer.sync()

            # Update Max Height after Control Starts
            if time.time() - start >= 3:
                max_height_after_control = max(max_height_after_control, d.qpos[0])

            # Store Data (After mj_step)
            base_height_store.append(d.qpos[0])  
            height_reward.append(100 * np.exp(-d.qpos[0]))
            base_vel_store.append(d.qvel[0])

            # Respect Simulation Timestep
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    return base_height_store, height_reward, total_energy, hip_energy, knee_energy, max_height_after_control
action = np.array([9.93,0.0,0.01,9.93,0.0,0.01])
xml_path = 'modified_robot_400_400.xml'

# Load MuJoCo Model

#d.rng = np.random.default_rng(0)

# Run Simulation
base_height_store, height_reward, te, he, ke,max_height_after_control = run(xml_path, action)
# Print Results
print(max_height_after_control, 100 * np.exp(-max_height_after_control) + (1/3)*te, te, he, ke)
print(100 * np.exp(-max_height_after_control) + (1/3)*te)
