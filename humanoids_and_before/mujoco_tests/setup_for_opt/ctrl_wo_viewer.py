import numpy as np
import mujoco as mj
import vmc_mujoco as vmc
import time
import mujoco.viewer as mj_viewer
import len_rad_id as lri
import vmc_hopper_roc as vmc_hr
import vmc_roc_damper as vmc_rd
#import len_rad_id as lri
xml_path = 'test_pb/hopper_roc.xml'
m = mj.MjModel.from_xml_path('test_pb/hopper_roc.xml')
d = mj.MjData(m)
mj.mj_setTotalmass(m, 3) # Set the total mass of the robot to 1.5 kg
ef=[]
joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "slidez")
base_height_store = []
grf_z = []
#print("joint_id:", joint_id)
force_index = m.jnt_dofadr[joint_id]
hip_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge1")
hip_force_index = m.jnt_dofadr[hip_joint_id]
hip_actuator_id = m.actuator('torque1').id
#print("hip_actuator_id:", hip_actuator_id)
knee_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge2")
knee_force_index = m.jnt_dofadr[knee_joint_id]
knee_actuator_id = m.actuator('torque2').id
hip_torque_store = []
knee_torque_store = []
#print("knee_actuator_id:", knee_actuator_id)
hip_torque_calc_store = []
knee_torque_calc_store = []
controller = vmc_rd.Controller(xml_path, m, d)

total_forces = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied
hip_angle_store = []
grf_z = []
knee_angle_store = []  
base_height = []
base_vel_store = []         
lri = lri.len_rad_id(xml_path, m, d)
#Positioning the robot's joint angles at certain values
d.qpos[1] = 0.7854  # Assuming hinge1 is at index 1
d.qpos[2] = -1.5708


  # Close the viewer automatically after 30 wall-seconds.
start = time.time()
  
while time.time() - start < 100:    
    #PD control code to fix the robot's joint angles at certain values upto 5 seconds of spawning
    if time.time() - start < 3:
        desired_hip_angle = 0.78
        desired_knee_angle = -1.57    
        hip_error = desired_hip_angle - d.qpos[1] 
        knee_error = desired_knee_angle - d.qpos[2]
        hip_torque = 100* hip_error - 10 * d.qvel[1]
        knee_torque = 100 * knee_error - 10 * d.qvel[2]
        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        mj.mj_step(m, d)
        base_height_store.append(d.qpos[0])
        base_vel_store.append(d.qvel[0])
        hip_torque_store.append(d.qfrc_actuator[hip_actuator_id])
        knee_torque_store.append(d.qfrc_actuator[knee_actuator_id])
    else:
        
        
        contact_force = controller.get_ground_contact_forces()
        if len(contact_force)==0:
           contact_force = [(np.array([0, 0, 0]), 0)]

        first_tuple = contact_force[0]
        forces = first_tuple[0]
        #print("forces:", forces)
        grf_z.append(forces[0])
        #print(f"forces: {forces} at time {time.time() - start}")
        #print(d.qvel[0])
        # print(d.qvel[1])
        # print(d.qvel[2])
        # print(d.qfrc_actuator[hip_actuator_id])
        
        
        #print("forces:", forces)
        #print("base velocity:", d.qvel[0])
        #print("base height:", d.qpos[0])

        if forces[0]==0 and forces[1]==0 and forces[2]==0:
            for _ in range(5):             
                desired_hip_angle = 0.78
                desired_knee_angle = -1.57
            
                hip_error = desired_hip_angle - d.qpos[1]
                knee_error = desired_knee_angle - d.qpos[2]
                hip_torque = 100* hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]
                d.ctrl[hip_actuator_id] = hip_torque
                d.ctrl[knee_actuator_id] = knee_torque
               
        else:
            for _ in range(5):  
                joint_torque = controller.joint_torque()
                #print("joint_torque:", joint_torque)
                hip_torque_calc_store.append(joint_torque[1])
                knee_torque_calc_store.append(joint_torque[0])       
                if joint_torque[1] > 50 and joint_torque[0] > 50:
                    d.ctrl[hip_actuator_id] = 50
                    d.ctrl[knee_actuator_id] = 50
                   
                else:
                    d.ctrl[hip_actuator_id] = joint_torque[1]
                    d.ctrl[knee_actuator_id] = joint_torque[0]
                
        mj.mj_step(m, d)
        print("force_z",forces[0])
        
        base_height_store.append(d.qpos[0])
        base_vel_store.append(d.qvel[0])
        #hip_torque_store.append(d.qfrc_actuator[hip_actuator_id])
        #knee_torque_store.append(d.qfrc_actuator[knee_actuator_id])
        #print("hip torque:", d.qfrc_actuator[hip_actuator_id])
        #print("knee torque:", d.qfrc_actuator[knee_actuator_id])


     
    
    step_start = time.time()
    
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
      #time.sleep(1)
import matplotlib.pyplot as plt
plt.plot(base_height_store)
plt.plot(base_vel_store)

#plt.plot(hip_torque_calc_store)
#plt.plot(grf_z)
plt.show()

#plt.plot(knee_torque_calc_store)
plt.show()
