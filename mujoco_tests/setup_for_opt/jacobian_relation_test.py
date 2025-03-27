import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time
import xml.etree.ElementTree as ET
import len_rad_id as lri
xml_path = 'test_pb/hopper_roc_angle.xml'
m = mujoco.MjModel.from_xml_path('test_pb/hopper_roc_angle.xml')
d = mujoco.MjData(m)
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
lri = lri.len_rad_id(xml_path, m, d)


def get_ground_contact_forces(model, data):
  """
  Extracts ground contact forces from MuJoCo simulation data.

  Args:
    model: MuJoCo model object.
    data: MuJoCo data object.

  Returns:
    A list of contact forces, where each element is a tuple 
    containing the contact force vector and the corresponding 
    body index.
  """

  ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")  # Assuming "floor" is the ground geom
  ground_contact_forces = []

  for i in range(data.ncon):
    con = data.contact[i]
    if con.geom1 == ground_id or con.geom2 == ground_id:
      # Calculate the contact force vector
      force = np.zeros(6, dtype=np.float64)
      mujoco.mj_contactForce(model, data, i, force) 

      # Extract linear force (first 3 elements)
      contact_force = force[:3] 

      # Get the body index 
      body_id = con.geom1 if con.geom1 != ground_id else con.geom2

      ground_contact_forces.append((contact_force, body_id)) #this gives a tuple of contact force(array) and the body index  

  return ground_contact_forces



def obtain_grf(ground_forces):
    #ground_forces = get_ground_contact_forces(m, d)
    first_tuple = ground_forces[0]
    forces= first_tuple[0]
    force_z = forces[0]
    force_x = forces[1]
    force = np.array([force_x, force_z])
    
    return force
  

def joint_angles(joint_id, hip_joint_id, knee_joint_id):
    joint_angle = d.qpos[joint_id]
    hip_angle = d.qpos[hip_joint_id]
    knee_angle = d.qpos[knee_joint_id]
    return joint_angle, hip_angle, knee_angle

def rotation_matrix(joint_id, hip_joint_id, knee_joint_id):
   joint_angle, hip_angle, knee_angle = joint_angles(joint_id, hip_joint_id, knee_joint_id)
   knee_angle = np.abs(knee_angle)
   total_angle =  -hip_angle + knee_angle
   #total_angle = np.abs(total_angle)
   rotation_matrix = np.array([[np.cos(total_angle), -np.sin(total_angle)], [np.sin(total_angle), np.cos(total_angle)]])
   return rotation_matrix
#print("rotation_matrix:", rotation_matrix(joint_id, hip_joint_id, knee_joint_id))
def jacobian(joint_id, hip_joint_id, knee_joint_id):      
    joint_angle, hip_angle, knee_angle = joint_angles(joint_id, hip_joint_id, knee_joint_id)
    hip_angle = np.abs(hip_angle)
    knee_angle = np.abs(knee_angle)
    thigh_total_lengt = lri.thigh_total_length()
    calf_total_lengt = lri.calf_total_length()
    J11 =  -thigh_total_lengt*np.cos(hip_angle)-calf_total_lengt*np.cos(knee_angle-hip_angle)
    J12 = calf_total_lengt*np.cos(knee_angle-hip_angle)
    J21 = -thigh_total_lengt*np.sin(hip_angle) + calf_total_lengt*np.sin(knee_angle - hip_angle) 
    J22 = -calf_total_lengt*np.sin(knee_angle - hip_angle) 
    jacobian = np.array([[J11, J12], [J21, J22]])    
    #jacobian = np.array([[thigh_length*np.cos(hip_pos)+leg_length*np.cos(knee_pos+hip_pos), leg], [0, 1, 0], [thigh_length*np.sin(hip_pos) + leg_length*np.sin(knee_pos), 0, 0]])
    return jacobian

def calc_inverse(joint_id, hip_joint_id, knee_joint_id):
    jacobian_matrix = jacobian(joint_id, hip_joint_id, knee_joint_id)
    J11 = jacobian_matrix[1][1]
    J12= -jacobian_matrix[0][1]
    J21 = -jacobian_matrix[1][0]
    J22 = jacobian_matrix[0][0]
    det = J11*J22 - J12*J21
    inverse = np.array([[J11, J12], [J21, J22]])/det
    print("det:", det)
    return inverse
#print("jacobian:", jacobian(joint_id, hip_joint_id, knee_joint_id))
base_height_store = []
def jacobian_transpose(joint_id, hip_joint_id, knee_joint_id):
    jacobian_matrix = jacobian(joint_id, hip_joint_id, knee_joint_id)
    jacobian_transpose = np.transpose(jacobian_matrix)
    return jacobian_transpose

def pseudoinverse(joint_id, hip_joint_id, knee_joint_id):    
    jacobian_transpose_matrix = jacobian_transpose(joint_id, hip_joint_id, knee_joint_id)
    jacobian_pseudoinverse = np.linalg.pinv(jacobian_transpose_matrix)
    return jacobian_pseudoinverse


mujoco.mj_setTotalmass(m, 1)
mj_getTotalmass = mujoco.mj_getTotalmass(m)
#print("total_mass:", mj_getTotalmass) 




#print("pseudoinverse:", pseudoinverse(joint_id, hip_joint_id, knee_joint_id))
#print("actuator_id", m.actuator('torque2').id)

#print("controllable modes", m.nu)
ef=[]
det=[]
difference=[]
tau=np.zeros((2, 1))
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  mujoco.mj_step(m, d)
  ground_forces = get_ground_contact_forces(m, d) 
  if len(ground_forces)==0:
       ground_forces = [(np.array([0, 0, 0]), 0)]      
  #print("ground_forces:", ground_forces)
  rotation_matrix1 = rotation_matrix(joint_id, hip_joint_id, knee_joint_id)
  transformed_ground_forces = np.dot(rotation_matrix1, obtain_grf(ground_forces)) 
  grf_z.append(transformed_ground_forces[1])
  #print("transformed_ground_forces:", transformed_ground_forces)
  #print force on link2 com using xfrc_applied
  #print("xfrc_applied:", d.xfrc_applied)

  first_tuple1 = ground_forces[0]
  #print("first_tuple:", first_tuple1)
  forces1= first_tuple1[0]
  force_z1 = forces1[0]
  mass_cal = np.zeros((2, 1))
  mass_cal[1] = force_z1
  #print("force_z:", force_z1)
  
  while viewer.is_running() and time.time() - start < 50:
    step_start = time.time()
    #print("slider force:", d.qfrc_applied[force_index])

# Define the desired torque
    continuous_torque_1 = 90
    continuous_torque_2 = 10
    continuous_torque = 300   
       

# Simulation loop # Simulate for 100 steps
    # Apply torque continuously within the loop
    #d.qfrc_applied[force_index] = continuous_torque 
    #d.qfrc_applied[knee_force_index] = continuous_torque_2
    #for _ in range(10):
       
    d.ctrl[0] = continuous_torque_2
    #d.ctrl[1] = -continuous_torque_2
    #d.ctrl[1] = continuous_torque_2
    #print("d.qfrc_actuator", d.qfrc_actuator)
    mujoco.mj_step(m, d)
    
    tau[0]= d.qfrc_actuator[hip_actuator_id+1]

    tau[1]= d.qfrc_actuator[knee_actuator_id+1]
    
    #print("tau:", tau)        
    
    
    #print("expected_force:", expected_force)

    ground_forces = get_ground_contact_forces(m, d) 
    if len(ground_forces)==0:
        ground_forces = [(np.array([0, 0, 0]), 0)]      
    #print("ground_forces:", ground_forces)
    rotation_matrix1 = rotation_matrix(joint_id, hip_joint_id, knee_joint_id)
    transformed_ground_forces = np.dot(rotation_matrix1, obtain_grf(ground_forces)) 
    print("grf in contact frame:", obtain_grf(ground_forces))
    print("transformed_ground_forces:", transformed_ground_forces)
    
    expected_force=np.dot(pseudoinverse(joint_id, hip_joint_id, knee_joint_id), tau) + np.array([[0], [9.81]])
    ef.append(-expected_force[1])
    grf_z.append(transformed_ground_forces[1])
    print("expected_force:", expected_force)
       
       #print("ground_forces:", ground_forces)
       #first_tuple = ground_forces[0]
       #print("first_tuple:", first_tuple)
    #    forces= first_tuple[0]
    #    force_z = forces[0]
       #print("force_z:", force_z)
    #    grf_z.append(force_z)

    #    difference.append(force_z - expected_force[1])
    #    print("difference:", force_z - expected_force[1])
       
       

       
       #print("determinant of jacobian:", np.linalg.det(jacobian(joint_id, hip_joint_id, knee_joint_id)))
       
       #print("pseudoinverse_matrix:", pseudoinverse_matrix)
       #print("tau:", tau)

    
              
    #    ground_forces = get_ground_contact_forces(m, d)
    #    #ground_forces = np.array(ground_forces)
    #    print("ground_forces:", ground_forces)
    #    first_tuple = ground_forces[0]
    #    print("first_tuple:", first_tuple)
    #    forces= first_tuple[0]
    #    force_z = forces[0]
    #    grf_z.append(force_z)
    
    #print("expected_force:", expected_force)
    det.append(np.linalg.det(jacobian(joint_id, hip_joint_id, knee_joint_id)))
    #print("hip_joint_angles", d.qpos[hip_joint_id])
    hip_angle_store.append(d.qpos[hip_joint_id])
    #print("knee_joint_angles", d.qpos[knee_joint_id])
    #print("inverse:", np.transpose(calc_inverse(joint_id, hip_joint_id, knee_joint_id))) 
    
    #print("pseudoinverse:", pseudoinverse(joint_id, hip_joint_id, knee_joint_id))
    knee_angle_store.append(d.qpos[knee_joint_id])
    # if expected_force[1] > 200:
    #     expected_force[1] = 200
    ef.append(-expected_force[1])
    #print("expected_force:", -expected_force[1])
    ground_forces = get_ground_contact_forces(m, d)
       #ground_forces = np.array(ground_forces)
    
    if len(ground_forces)==0:
       ground_forces = [(np.array([0, 0, 0]), 0)]
    
    #print("ground_forces:", ground_forces)
    #first_tuple = ground_forces[0]
    #second_tuple = ground_forces[1]
    #print("first_tuple:", first_tuple)
    #forces= first_tuple[0]
    
    #print("forces:", forces)
    #force_z = forces[0]
    #print("force_z:", force_z)
    #grf_z.append(force_z)
    #print("force_z:", force_z)
    #print("expected_force:", expected_force[1])
    #print("difference:", force_z + expected_force[1])
    #difference.append(force_z + expected_force[1])

        # d.qfrc_actuator[hip_actuator_id] = continuous_torque_2
        # d.qfrc_actuator[knee_actuator_id] = continuous_torque_1
    #print("actuator_torques", d.qfrc_actuator)
    # print(d.qpos[hip_joint_id])
    # print(d.qpos[knee_joint_id])
    #print(d.qpos[joint_id])
    #print(jacobian(joint_id, hip_joint_id, knee_joint_id))
    base_height_store.append(d.qpos[joint_id])
    #hip_angle_store.append(d.qpos[hip_joint_id])  #d.qpos gives angles in radians of the joints
    #base_height_store.append(d.qpos[joint_id])
    
    ground_forces = get_ground_contact_forces(m, d)
    #if there's no contact detected, set geom1 to 0
    #print(d.contact[0].geom1)
    #print("pseudoinverse:", pseudoinverse(joint_id, hip_joint_id, knee_joint_id))

    #print("ground_forces:", ground_forces)
    # first_tuple = ground_forces[0]
    # forces= first_tuple[0]
    # force_z = forces[0]/1000000
    # grf_z.append(force_z)


# Print the contact forces (optional)
    # for force, body_id in ground_forces:
    #     print(f"Contact force on body {body_id}: {force}")
 
    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
 
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
 
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
      #time.sleep(1)
    # if time.time() - step_start > 2:
    #     exit()

#plot the base height wrt time
import matplotlib.pyplot as plt
#plt.plot(grf_z)
# plt.plot(ef)
# #plt.legend(["grf_z", "expected_force"])
# plt.show()
#plt.plot(knee_angle_store)
plt.plot(ef)

#plt.savefig('test_pb/ef.png')
plt.plot(grf_z)
plt.show()
#plt.savefig('test_pb/grf_z.png')
#plt.plot(difference)

#plt.plot(base_height_store)  
#plt.plot(hip_angle_store)
#plt.show()
