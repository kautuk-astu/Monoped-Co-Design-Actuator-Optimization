import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time
import len_rad_id as lri

m = mujoco.MjModel.from_xml_path('/home/stochlab/Aastha_Coopt_Monoped/maxh_with_ik/thigh_60/60/modified_robot.xml')
d = mujoco.MjData(m)

joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slidez")
force_index = m.jnt_dofadr[joint_id]
hip_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge1")
hip_force_index = m.jnt_dofadr[hip_joint_id]
hip_actuator_id = m.actuator('torque1').id
knee_actuator_id = m.actuator('torque2').id
knee_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge2")
knee_force_index = m.jnt_dofadr[knee_joint_id]

total_forces = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied


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

  ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "slot_geom")  # Assuming "floor" is the ground geom
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

      ground_contact_forces.append((contact_force, body_id))

  return ground_contact_forces

def joint_angles(joint_id, hip_joint_id, knee_joint_id):
    joint_angle = d.qpos[joint_id]
    hip_angle = d.qpos[hip_joint_id]
    knee_angle = d.qpos[knee_joint_id]
    return joint_angle, hip_angle, knee_angle

def jacobian(joint_id, hip_joint_id, knee_joint_id):      
    joint_angle, hip_angle, knee_angle = joint_angles(joint_id, hip_joint_id, knee_joint_id)
    thigh_total_lengt = lri.thigh_total_length()
    calf_total_lengt = lri.calf_total_length()
    J11 =  thigh_total_lengt*np.cos(hip_angle)+calf_total_lengt*np.cos(knee_angle+hip_angle)
    J12 = calf_total_lengt*np.cos(knee_angle+hip_angle)
    J21 = thigh_total_lengt*np.sin(hip_angle) + calf_total_lengt*np.sin(knee_angle + hip_angle) 
    J22 = calf_total_lengt*np.sin(knee_angle + hip_angle) 
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



#print("pseudoinverse:", pseudoinverse(joint_id, hip_joint_id, knee_joint_id))
#print("actuator_id", m.actuator('torque2').id)

#print("controllable modes", m.nu)
det=[]
difference=[]
tau=np.zeros((2, 1))
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 10:
    step_start = time.time()
    #print("slider force:", d.qfrc_applied[force_index])

# Define the desired torque
    continuous_torque_1 = 90
    continuous_torque_2 = 0.01
    continuous_torque = 300 # Example: Apply a continuous torque of 0.5 Nm

# Simulation loop # Simulate for 100 steps
    # Apply torque continuously within the loop
    #d.qfrc_applied[force_index] = continuous_torque 
    #d.qfrc_applied[knee_force_index] = continuous_torque_2
    for _ in range(10):
       d.ctrl[0] = continuous_torque_2
       #d.ctrl[1] = -continuous_torque_2
       #d.ctrl[1] = continuous_torque_2
       #print("d.qfrc_actuator", d.qfrc_actuator)
       mujoco.mj_step(m, d)
       
       tau[0]= d.qfrc_actuator[hip_actuator_id +1]
    
       tau[1]= d.qfrc_actuator[knee_actuator_id+1]
       #print("tau:", tau)      
       
       
       
       expected_force=np.dot(pseudoinverse(joint_id, hip_joint_id, knee_joint_id), tau) 
       print("expected_force:", expected_force)

       ground_forces = get_ground_contact_forces(m, d)
       
       print("ground_forces:", ground_forces)
       first_tuple = ground_forces[0]
       print("first_tuple:", first_tuple)
       forces= first_tuple[0]
       force_z = forces[0]
       print("force_z:", force_z)
       grf_z.append(force_z)
       exit()
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
    print("expected_force:", -expected_force[1])
    ground_forces = get_ground_contact_forces(m, d)
       #ground_forces = np.array(ground_forces)
    
    if len(ground_forces)==0:
       ground_forces = [(np.array([0, 0, 0]), 0)]
    
    #print("ground_forces:", ground_forces)
    first_tuple = ground_forces[0]
    #second_tuple = ground_forces[1]
    #print("first_tuple:", first_tuple)
    forces= first_tuple[0]
    
    #print("forces:", forces)
    force_z = forces[0]
    print("force_z:", force_z)
    grf_z.append(force_z)
    #print("force_z:", force_z)
    #print("expected_force:", expected_force[1])
    #print("difference:", force_z + expected_force[1])
    difference.append(force_z + expected_force[1])

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
    #print("pseudoinverse:", pseudoinverse(joint_id, hip_joint_id, knee_joint_id))

    #print("ground_forces:", ground_forces)
    # first_tuple = ground_forces[0]
    # forces= first_tuple[0]
    # force_z = forces[0]/1000000
    # grf_z.append(force_z)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    #print("slider force:", d.qfrc_applied[force_index])

# Define the desired torque
    continuous_torque = 10000 # Example: Apply a continuous torque of 0.5 Nm

# Simulation loop # Simulate for 100 steps
    # Apply torque continuously within the loop
    #d.qfrc_applied[force_index] = continuous_torque 
    d.ctrl[hip_actuator_id] = continuous_torque
    d.ctrl[knee_actuator_id] = continuous_torque
    #d.qfrc_applied[knee_force_index] = continuous_torque
    #d.qfrc_applied[hip_force_index] = continuous_torque
    #print(d.qpos[hip_joint_id])
    #print(d.qpos[knee_joint_id])
    print(d.ctrl[hip_actuator_id])

    #print(d.qfrc_applied[hip_force_index])

    mujoco.mj_step(m, d)
    d.qfrc_actuator[hip_actuator_id] = -continuous_torque
    ground_forces = get_ground_contact_forces(m, d)

# Print the contact forces (optional)
    # for force, body_id in ground_forces:
    #     print(f"Contact force on body {body_id}: {force}")
 
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
 
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
 
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    print(f"simulation working{step_start}")
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
      #time.sleep(0.05)

