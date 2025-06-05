#NOTE: WORKING CONTROL TEST WITH 1.5 KG NOMINAL WEIGHT AND 10, 20, 30 KP INCREASING HEIGHT 0.1, 0.4, 0.5 M AND HOPPER ROC
import numpy as np
import mujoco as mj
import vmc_mujoco as vmc
import time
import mujoco.viewer as mj_viewer
import len_rad_id as lri
import vmc_hopper_roc as vmc_hr
import vmc_roc_damper as vmc_rd
import vmc_roc_action as vmc_ra
import matplotlib.pyplot as plt
import csv
import ik as ik
#xml_path = "/home/stochlab/Aastha_Coopt_Monoped/mujoco_tests/modified_robot.xml"
xml_path = "/home/stochlab/Aastha_Coopt_Monoped/maxh_with_ik/thigh_60/575/modified_robot.xml"
#55, 55 1.13,; 525, 575 - 1.19(very high reproducibility); 575, 60 - 1.2, 525, 525 - 1.21(very high reproducibility)
#525, 55 - 0.84/1.03 , 0.6, 0.6 - 1.15 (very high reproducibility)
#50, 50 - 0.89/1.004(very different from optimised value), 50, 60 - 1.04 (very high reproducibility)
#50, 525 - 0.7/1.17 (very low reproducibility), 55, 60 - 0.94/1.26(50% reproducibility)
#525, 60 - 1.04/1.18(50% reproducibility), 40, 35 - 0.62 (very high reproducibility)
#40, 40 - 0.82, 698.35
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)

mj.mj_setTotalmass(m,3.275) # Set the total mass of the robot to 1.5 kg
ef=[]
joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "slidez")
base_height_store = []
grf_z = []   
force_index = m.jnt_dofadr[joint_id]
hip_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge1")
hip_force_index = m.jnt_dofadr[hip_joint_id]
hip_actuator_id = m.actuator('torque1').id    
knee_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge2")
knee_force_index = m.jnt_dofadr[knee_joint_id]
knee_actuator_id = m.actuator('torque2').id
hip_torque_store = []
knee_torque_store = []    
hip_torque_calc_store = []
knee_torque_calc_store = []
knee_power_store = []
hip_power_store = []
controller = vmc_rd.Controller(xml_path, m, d)
hip_angle_store = []
grf_z = []
knee_angle_store = []  
base_height = []
base_vel_store = []  
height_reward = []
hc = 0 
kc = 0
tc = 0       
# lri = lri.len_rad_id(xml_path, m, d)     -1.1, 2.16
#-1.0239453760989525 2.047890752197905
#-0.9361123332086306 2.2183034643951447
#-1.052687452403107 2.105374904806214
#-1.1084866825417978 2.132769735169058
#-0.9865427541187348 2.233020897808028
#-1.0481145038501272 2.1815417198880813
#-0.9977784633936242 2.0784668168567038

#thigh_length + calf+ radius works
#-1.0784131865189037 2.1568263730378074
#-1.0175162368586064 2.206931352439912
#1.1384102656587565 2.109366499119172
#-1.16826237675474 2.086622379441024 - gives more reasonable positioning but can do better calf + 2 and thigh -1 
#-1.1083332031082482 2.0520120144887715
#-1.1656995088733182 2.167561489018619 - 0.6, 0.6 - 2.1 m (0.54 for ik) - works with 15s
#-1.1083332031082482 2.0520120144887715 - 0.525 each - 2.175m (0.52 for ik) - works with 15s
#-0.8765125664056651 1.6140240740875673
#-0.9183364294768195 1.6871299784680978
#-0.9389067268954726 1.7228096777353672 - 0.4, 0.4 - 1.81m but (just -0.49) (best for video)
#-0.9491176389378801 1.7404482561644785
#-0.9286473466720578 1.705038444362078  - 0.4, 0.4(400 Kr) - 0.484 - 1.80m (best for video) (0.49 in ik) - works with 15s
#-1.0892154496939663 2.0125833049126016
#-1.0817988413099815 2.000021808286157 - 0.5, 0.5 - 2.13/2.19m (good for video) (0.515 in ik), 1.9m - 10s, works for 15s tho
#-1.143452205851491 2.0382134996717363
#-1.1356668926577347 2.0259826016568776 - 0.5, 0.525 - 1.82m (0.52 in ik) - works with 10,15
lp = lri.len_rad_id(xml_path, m, d)
l1 = lp.thigh_total_length()
l2 = lp.calf_total_length()
theta1, theta2 = ik.inverse_kinematics(0.5355, 0, l1, l2)
# theta1 = -1.05
# theta2 = 2.0
d.qpos[1] = theta1
d.qpos[2] = theta2
te = 0
he = 0
ke = 0
kea=0
hea=0
start = time.time() 
total_energy = 0
knee_energy = 0
hip_energy = 0 
total_actual = 0 
base_plot_store = []
time_store = []
with mj.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while time.time() - start < 15:   
        step_start = time.time()         
        if time.time() - start < 3:
            desired_hip_angle = theta1
            desired_knee_angle = theta2
            hip_error = desired_hip_angle - d.qpos[1] 
            knee_error = desired_knee_angle - d.qpos[2]
            hip_torque = 100* hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
            d.ctrl[hip_actuator_id] = hip_torque
            d.ctrl[knee_actuator_id] = knee_torque
            mj.mj_step(m, d)
            base_height_store.append(d.qpos[0])
            print("base_height",d.qpos[0])
            if d.qvel[0] > 0.1:
                #print("base_vel",d.qvel[0])
                print("base_vel",d.qvel[0], time.time()-start)
                
            base_height.append(d.qpos[0])
            height_reward.append(100*np.exp(-(d.qpos[0])))
            base_vel_store.append(d.qvel[0])
            #hip_torque_store.append(d.qfrc_actuator[hip_actuator_id])
            #knee_torque_store.append(d.qfrc_actuator[knee_actuator_id])
            
        else: 
            
            contact_force = controller.get_ground_contact_forces()
            if len(contact_force)==0:
                contact_force = [(np.array([0, 0, 0]), 0)]

            first_tuple = contact_force[0]
            forces = first_tuple[0]           
            grf_z.append(forces[0])         

            if forces[0]==0 and forces[1]==0 and forces[2]==0:                                
                desired_hip_angle = theta1
                desired_knee_angle = theta2
                hip_error = desired_hip_angle - d.qpos[1]
                knee_error = desired_knee_angle - d.qpos[2]
                hip_torque = 100* hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
                d.ctrl[hip_actuator_id] = hip_torque
                d.ctrl[knee_actuator_id] = knee_torque
                knee_angle_store.append(d.qpos[2])
                hip_angle_store.append(d.qpos[1])
                base_plot_store.append(d.qpos[0])
                time_store.append(time.time()-start-3)
                hip_torque_store.append(d.qfrc_actuator[hip_actuator_id])
                knee_torque_store.append(d.qfrc_actuator[knee_actuator_id])
                #print("hip_vel",d.qvel[1])
                #print("knee_vel",d.qvel[2])
            
            else:
                base_new_height = d.qpos[0]                            
                joint_torque = controller.joint_torque() 
                hip_torque = np.clip(joint_torque[0], -50, 50)
                knee_torque = np.clip(joint_torque[1], -50, 50)
            d.ctrl[hip_actuator_id] = hip_torque
            d.ctrl[knee_actuator_id] = knee_torque

        # Step simulation
        mj.mj_step(m, d)

        # Store simulation data
        base_height_store.append(d.qpos[0])
        height_reward.append(100 * np.exp(-d.qpos[0]))
        base_vel_store.append(d.qvel[0])
        ha = m.opt.timestep * d.qfrc_actuator[hip_actuator_id+1] * d.qvel[1]
        ka = m.opt.timestep * d.qfrc_actuator[knee_actuator_id+1] * d.qvel[2]
        if ha > 0:
            he += ha
            te += ha
        if ka > 0:
            ke += ka
            te += ka    

                                
                            
                    
                            
                                    
                                            
                        
                        
                        
                        
                        
                        
                        
                        

                            
                    
                
        with open("ht1.csv", mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time.time() - start - 3, d.qpos[0], d.qfrc_actuator[hip_actuator_id+1], d.qfrc_actuator[knee_actuator_id+1]])                     
            
        

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
                

    # Apply control torques before stepping
           
            #time.sleep(1)
print(total_energy)
print("total from iters", tc)
print("total new", te)
print(total_actual)
print(np.max(base_height_store))
with open("data.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["knee", "hip", "base", "time","hip torque", "knee torque"])  # Header
    for a, b,c, d,e,f in zip(knee_angle_store, hip_angle_store, base_plot_store, time_store, hip_torque_store, knee_torque_store):
        writer.writerow([a, b, c, d,e,f])

plt.figure()
plt.plot(knee_angle_store)
plt.plot(hip_angle_store)
plt.plot(base_plot_store)
plt.legend(["knee angle", "hip angle", "base position"])
plt.xlabel("Time(s)")
plt.ylabel("Angle(rad)")
#plt.show()
plt.savefig("angle_plot_45.png")
plt.figure()
#give code for this to be a new figure with only these two values plotted

plt.plot(knee_torque_store)
plt.plot(hip_torque_store)
plt.legend(["knee_torque", "hip_torque"])
plt.xlabel("Time(s)")
plt.ylabel("Torque(Nm)")
#plt.show()
plt.savefig("torque_plot_45.png")
