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

def run(xml_path, action, mass, ik_value, hip_peak_torque, knee_peak_torque):
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
    
    # xml_new_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/nominal_design_xml/modified_robot_500_500.xml'
    # m_n = mj.MjModel.from_xml_path(xml_new_path)
    # d_n = mj.MjData(m_n)
    # lp = lri.len_rad_id(xml_new_path, m_n, d_n)
    #mj.mj_resetData(m_n, d_n)
    mj.mj_resetData(m, d)
#     # Increase solver precision for more stable results
    # l1n = lp.thigh_total_length()
    # l2n = lp.calf_total_length()
    l1n = 0.4
    l2n = 0.4
    #print(f"l1n: {l1n}, l2n: {l2n}")
    theta1, theta2 = ik.inverse_kinematics(0, ik_value, l1n, l2n)
    controller = vmc_ra_angle.Controller(xml_path, m, d, theta1, theta2)
    omega1_store = []
    omega2_store = []
    v0_store = []
    theta1_store = []
    theta2_store = []
    base_store = []
    torque1_store = []
    torque2_store = []
    time_store = []
    #print(theta1, theta2)
    theta1d = np.rad2deg(theta1)
    theta2d = np.rad2deg(theta2)
    #print(theta1d, theta2d)
    #exit()
    # Compute Initial Joint Angles
    # lp = lri.len_rad_id(xml_path, m, d)
    # l1 = lp.thigh_total_length()
    # l2 = lp.calf_total_length()
    # theta1, theta2 = ik.inverse_kinematics(0, ik_value, l1, l2)
    
    # Set Initial Joint Angles
    # d.qpos[1] = theta1
    # d.qpos[2] = theta2
    # d.qvel[:] = 0
    
    # Start Simulation
    start = time.time()
    with mj.viewer.launch_passive(m, d) as viewer:
       
        cam_id = m.camera('cam0').id
        viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED  # ✅ Correct enum
        viewer.cam.fixedcamid = cam_id
      


        while viewer.is_running() and time.time() - start < 8:
            step_start = time.time()
            t_elapsed = time.time() - start
            
            # Compute Torques
            if t_elapsed < 0:
                hip_error = 0 - d.qpos[1]
                knee_error = 0 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
            else:
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
                        jump_energy = 0
                        if jump_count > 1:
                            break
                    
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
                    #print(ha, in_air)
                    
                    total_energy += ha
                    jump_energy += ha
                if ka > 0:
                    knee_energy += ka
                    #print(ka, in_air)
                    #print(d.qpos[0])
                    total_energy += ka
                    jump_energy += ka
            
            # Step simulation
            omega1_store.append(d.qvel[1])
            #print(d.qvel[1], d.qvel[2],d.qvel[0])
            #print(d.qpos[0], d.qpos[1], d.qpos[2])
            base_store.append(d.qpos[0])
            torque1_store.append(hip_torque)
            torque2_store.append(knee_torque)
            theta1_store.append(d.qpos[1])
            theta2_store.append(d.qpos[2])
            omega2_store.append(d.qvel[2])
            v0_store.append(d.qvel[0])
            time_store.append(t_elapsed)
            mj.mj_step(m, d)
            
            #print("angles", np.rad2deg(d.qpos[1]+theta1), np.rad2deg(d.qpos[2]+theta2))
            #print("knee angle", np.rad2deg(d.qpos[2]))
            viewer.sync()
            #print(d.qpos[0], d.qpos[1], d.qpos[2])
            
            # Update current jump max height if in air
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

    return max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store,time_store


# Define Action & XML Path
action = np.array([50,2.5,10])
#9,2.2,8.4
#7.9,2.2,8.4
#14.1,6.0,8.2
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_03_05/modified_robot_500_500.xml'
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/modified_robot_437_346_55_86.xml'
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/nominal_design_xml/modified_robot_400_400.xml'
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/design_xmls/modified_robot_399_357_52_42_fa9103fe.xml' 
#xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/design_xmls/modified_robot_407_411_51_58_0dbfb0db.xml'
#0.9,0.1xml_path='modified_robot_configured_497_346.xml', 4.646125,-0.5,28.05,65.24826,23.7,9.9,44.4,0.659984392339849 51.3004994444009 - works with even lesser cost
# Run Simulation
#0.7,0.3xml_path = 'modified_robot_configured_498_446.xml', 4.337833333333333,-0.5, 26.4,44.24882,7.9,6.1,20.4, 0.3824350736897383 31.75053451604305 - works with same height diff energy
#xml_path = 'modified_robot_configured_400_400.xml', 4.0844, -0.5, 33,33*1.3636,50,2.5,10, 0.3866466870185899 31.501544647902673
#0.65,0.35xml_path = 'modified_robot_configured_500_307.xml', 4.464600000000001,-0.5,22.0,58.498439999999995, 14.0,6.1,8.1, 0.3507568921776634 22.421980943135672
#0.65,0.35xml_path = 'modified_robot_configured_499_360.xml', 4.239,-0.5, 22.0,44.998799999999996, 0.33096132335302103 16.94609371751552, 34.247102520884766 - works fine with 49,36 l1l2,4.2107
xml_path = 'modified_robot_configured_400_400_nominal.xml'
max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results,omega2_store,omega1_store, v0_store, theta1_store, theta2_store, base_store, torque1_store, torque2_store, time_store= run(xml_path, action, 4.0844,-0.5, 6*5.5,6*5.5*1.3636)
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
df.to_csv("nominal_jump.csv", index=False)

# Print Results
print(max_jump_height, max_jump_energy)
# print(hip_energy)
# print(knee_energy)
# #print(jump_results)
# print(total_energy)
# print(jump_count)
#height_cost = np.exp(-np.round(max_jump_height,2))
height_cost = 100*np.exp(-(0.5+max_height_after_control))
final_cost = 0.65*(height_cost) + 0.35*(max_jump_energy)
print(final_cost)
#print(omega2_store)

# import matplotlib.pyplot as plt
# import numpy as np
# #plot torque1 and torque2
# plt.plot(torque1_store, label='Torque 1')
# plt.plot(torque2_store, label='Torque 2')
# plt.xlabel('Time Step')
# plt.ylabel('Torque')
# plt.title('Torque over Time')
# plt.legend()
# plt.show()


