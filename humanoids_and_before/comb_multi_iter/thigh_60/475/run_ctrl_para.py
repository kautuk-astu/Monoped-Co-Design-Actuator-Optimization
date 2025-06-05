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

def run(xml_path, m, d, action):
    mj.mj_setTotalmass(m, 3.175)     
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
    ht = []
    kt = []
    hv =[]
    kv = []
    hp = []
    kp = []
    bp = []
    bv = [] 
    te = 0 
    he = 0
    ke = 0
    knee_torque_store = []    
    hip_torque_calc_store = []
    knee_torque_calc_store = []
    knee_power_store = []
    hip_power_store = []
    controller = vmc_ra.Controller(xml_path, m, d)
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(0.52, 0, l1, l2)   
    print("theta1:", theta1)
    print("theta2:", theta2) 
    hip_angle_store = []
    grf_z = []
    knee_angle_store = []  
    base_height = []
    base_vel_store = []  
    height_reward = []            
    d.qpos[1] = theta1  # Assuming hinge1 is at index 1
    d.qpos[2] = theta2     
    total_energy = 0
    knee_energy = 0
    hip_energy = 0   
    
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
            height_reward.append(100*np.exp(-(d.qpos[0])))
            base_vel_store.append(d.qvel[0])
            hip_torque_store.append(d.qfrc_actuator[hip_actuator_id])
            knee_torque_store.append(d.qfrc_actuator[knee_actuator_id])
            #print(d.qpos[0])
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


                
            
            else:
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
                         
                        
                if abs(joint_torque[1]) > 50 or abs(joint_torque[0]) > 50:
                    if joint_torque[1] < -50 and joint_torque[0] < -50:
                        d.ctrl[hip_actuator_id] = -50
                        d.ctrl[knee_actuator_id] = -50
                        hip_applied_torque = d.ctrl[hip_actuator_id] 
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                        
                        
                    elif joint_torque[1] < -50 and -50<joint_torque[0]<50:
                        d.ctrl[knee_actuator_id] = -50
                        d.ctrl[hip_actuator_id] = joint_torque[0]
                        hip_applied_torque = d.ctrl[hip_actuator_id]                        
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                        
                        
                        
                    elif joint_torque[1] <-50 and joint_torque[0] > 50:
                        d.ctrl[hip_actuator_id] = 50
                        d.ctrl[knee_actuator_id] = -50
                        hip_applied_torque = d.ctrl[hip_actuator_id]                        
                        knee_applied_torque = d.ctrl[knee_actuator_id] 
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                                               
                        

                    elif joint_torque[1] > 50 and joint_torque[0] < -50:
                        d.ctrl[hip_actuator_id] = -50
                        d.ctrl[knee_actuator_id] = 50
                        hip_applied_torque = d.ctrl[hip_actuator_id]                        
                        knee_applied_torque = d.ctrl[knee_actuator_id] 
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                                               
                        

                    elif joint_torque[1] > 50 and -50<joint_torque[0]< 50:
                        d.ctrl[knee_actuator_id] = 50
                        d.ctrl[hip_actuator_id] = joint_torque[0]
                        hip_applied_torque = d.ctrl[hip_actuator_id]
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                        
                       
                    
                    elif joint_torque[1]>50 and joint_torque[0]>50:
                        d.ctrl[hip_actuator_id] = 50
                        d.ctrl[knee_actuator_id] = 50
                        hip_applied_torque = d.ctrl[hip_actuator_id]
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(hip_applied_torque)
                        kt.append(knee_applied_torque)
                        
                        
                    
                    elif joint_torque[0]<-50 and -50<joint_torque[1]<50:
                        d.ctrl[hip_actuator_id] = -50
                        d.ctrl[knee_actuator_id] = joint_torque[1]
                        hip_applied_torque = d.ctrl[hip_actuator_id]
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                        
                        
                    
                    elif joint_torque[0]>50 and -50<joint_torque[1]<50:
                        d.ctrl[hip_actuator_id] = 50
                        d.ctrl[knee_actuator_id] = joint_torque[1]
                        hip_applied_torque = d.ctrl[hip_actuator_id]
                        knee_applied_torque = d.ctrl[knee_actuator_id]
                        ht.append(abs(hip_applied_torque))
                        kt.append(abs(knee_applied_torque))
                        
                                           

                    
                else:
                    d.ctrl[hip_actuator_id] = joint_torque[0]                    
                    d.ctrl[knee_actuator_id] = joint_torque[1]
                    if d.ctrl[hip_actuator_id] ==0 or d.ctrl[knee_actuator_id] ==0:
                        if d.ctrl[hip_actuator_id] ==0:
                            print("hip_torque is ZERO")
                        if d.ctrl[knee_actuator_id] ==0:
                            print("knee_torque is ZERO")
                        
                    hip_applied_torque = d.ctrl[hip_actuator_id]                     
                    knee_applied_torque = d.ctrl[knee_actuator_id]
                    ht.append(abs(hip_applied_torque))
                    kt.append(abs(knee_applied_torque))
                        
                                        
                    
                    
                    
                    
                    
                    
                    
                    

                
            mj.mj_step(m, d)                       
            ha = m.opt.timestep * d.qfrc_actuator[hip_actuator_id+1] * d.qvel[1]
            ka = m.opt.timestep * d.qfrc_actuator[knee_actuator_id+1] * d.qvel[2]
            if ha > 0:
                he += ha
                te += ha
            if ka > 0:
                ke += ka
                te += ka
            base_height_store.append(d.qpos[0])
            height_reward.append(100*np.exp(-(d.qpos[0])))
            base_vel_store.append(d.qvel[0])          
                
        

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            #time.sleep(1)
    return base_height_store, height_reward, te, he, ke, ht, kt