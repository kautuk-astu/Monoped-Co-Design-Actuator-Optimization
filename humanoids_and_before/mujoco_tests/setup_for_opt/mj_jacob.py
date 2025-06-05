import mujoco 
import mujoco.viewer
import numpy as np

# Load MuJoCo model
m = mujoco.MjModel.from_xml_path("/home/stochlab/Aastha_Coopt_Monoped/mujoco_tests/hopper_roc.xml")  # Replace with your file
d = mujoco.MjData(m)

# Select the body for which you need the Jacobian (e.g., foot)
body_name = "link2"  # Replace with the correct body name
body_id = m.body(body_name).id
mujoco.mj_setTotalmass(m, 2.6) # Set the total mass of the robot to 1.5 kg
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

# Allocate space for Jacobian
Jp = np.zeros((3, m.nv))  # Linear velocity Jacobian
Jr = np.zeros((3, m.nv))
import time

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time() 
  
  
  
  while viewer.is_running() and time.time() - start < 5:
    #give  PD control code to fix the robot's joint angles at certain values
    #PD control code to fix the robot's joint angles at certain values

    #NOTE: 0.2, 0.3 M 0.7, -1.0 desired 50 kp for pos ctrl AND 100 KP DURING CONTROLLER
    #NOTE: 0.3, 0.4 M 0.7, -1.1 desired 50 kp for pos AND 100 KP DURING CONTROLLER
    #NOTE: 0.4, 0.4 M 0.7, -1.1 desired 50 kp for pos AND 100 KP DURING CONTROLLER
    #NOTE: 0.4, 0.5 M 0.7, -1.2 desired 50 kp for pos AND 100 KP DURING CONTROLLER


    desired_hip_angle = -0.78
    desired_knee_angle = 1.5
    
    hip_error = desired_hip_angle - d.qpos[1]
    knee_error = desired_knee_angle - d.qpos[2]
    hip_torque = 100 * hip_error - 10 * d.qvel[1]
    knee_torque = 100 * knee_error - 10 * d.qvel[2]
    d.ctrl[hip_actuator_id] = hip_torque
    d.ctrl[knee_actuator_id] = knee_torque

    step_start = time.time()  
    #d.ctrl[hip_actuator_id] = 0
    #d.ctrl[knee_actuator_id] = 0
    mujoco.mj_step(m, d)
    viewer.sync()
    print(d.qpos[0])
    print(time.time() - start)
    
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)  # Angular velocity Jacobian

# Compute the Jacobian
mujoco.mj_jacBody(m, d, Jp, Jr, body_id)

# Display Jacobians
print("Linear Velocity Jacobian (Jp):")
print(Jp)

print("Angular Velocity Jacobian (Jr):")
print(Jr)
hip_pos = d.qpos[1]
knee_pos = d.qpos[2]
thigh_length = 0.5
leg_length = 0.5
J11 =  0.5*np.cos(hip_pos)+0.5*np.cos(knee_pos+hip_pos)
J12 = 0.5*np.cos(knee_pos+hip_pos)
J21 = thigh_length*np.sin(hip_pos) + leg_length*np.sin(knee_pos + hip_pos)
J22 = leg_length*np.sin(knee_pos + hip_pos)
jacobian = np.array([[J11, J12], [J21, J22]])
print(jacobian)
