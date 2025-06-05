import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time
import xml.etree.ElementTree as ET
#import len_rad_id as lri
xml_path = '/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/thigh_60/60/modified_robot.xml'
m = mujoco.MjModel.from_xml_path(xml_path)
d = mujoco.MjData(m)
mujoco.mj_setTotalmass(m, 3.3) # Set the total mass of the robot to 1.5 kg
ef=[]
joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slidez")
#print("joint_id:", joint_id)
force_index = m.jnt_dofadr[joint_id]
hip_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge1")
hip_force_index = m.jnt_dofadr[hip_joint_id]
hip_actuator_id = m.actuator('torque1').id
#print("hip_actuator_id:", hip_actuator_id)
knee_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge2")
knee_force_index = m.jnt_dofadr[knee_joint_id]
knee_actuator_id = m.actuator('torque2').id
#print("knee_actuator_id:", knee_actuator_id)

total_forces = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied
hip_angle_store = []
grf_z = []
knee_angle_store = []   
#lri = lri.len_rad_id(xml_path, m, d)
#d.qpos[1] = 1.3 # Assuming hinge1 is at index 1
#d.qpos[2] = -2.5
desired_hip_angle = 0
height = 0.51
thigh_length = 0.5
shin_length = 0.5
remaining_length = height - thigh_length*np.cos(desired_hip_angle)
desired_knee_angle = np.arccos((remaining_length) / shin_length)
desired_knee_angle = desired_knee_angle
print("desired_knee_angle:", desired_knee_angle)
desired_knee_angle = 1.06
d.qpos[1]=0.0
d.qpos[2]= 0
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time() 
  
  while viewer.is_running() and time.time() - start < 3:
    #give  PD control code to fix the robot's joint angles at certain values
    #PD control code to fix the robot's joint angles at certain values

    #NOTE: 0.2, 0.3 M 0.7, -1.0 desired 50 kp for pos ctrl AND 100 KP DURING CONTROLLER
    #NOTE: 0.3, 0.4 M 0.7, -1.1 desired 50 kp for pos AND 100 KP DURING CONTROLLER
    #NOTE: 0.4, 0.4 M 0.7, -1.1 desired 50 kp for pos AND 100 KP DURING CONTROLLER
    #NOTE: 0.4, 0.5 M 0.7, -1.2 desired 50 kp for pos AND 100 KP DURING CONTROLLER  
    desired_hip_angle = -1.2
    desired_knee_angle = 2.17
    hip_error = desired_hip_angle - d.qpos[1]
    knee_error = desired_knee_angle - d.qpos[2]
    hip_torque = 100 * hip_error - 10 * d.qvel[1]
    knee_torque = 100 * knee_error - 10 * d.qvel[2]
    ##hip_torque = -5
    #knee_torque = 10
    #print(d.qpos[1])
    #print(d.qpos[2])
    d.ctrl[hip_actuator_id] = hip_torque
    d.ctrl[knee_actuator_id] = knee_torque
    print(d.qpos[0])

    step_start = time.time()  
    #d.ctrl[hip_actuator_id] = 0
    #d.ctrl[knee_actuator_id] = 0
    mujoco.mj_step(m, d)
    viewer.sync()
    #print(d.qpos[0])
    #print(time.time() - start)
    
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)