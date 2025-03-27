import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time
import xml.etree.ElementTree as ET
import len_rad_id as lri

xml_path = 'test_pb/hopper_roc.xml'
m = mujoco.MjModel.from_xml_path('test_pb/hopper_roc.xml')
d = mujoco.MjData(m)
ef=[]

joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slidez")
#print("joint_id:", joint_id)
force_index = m.jnt_dofadr[joint_id]

hip_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge1")
hip_force_index = m.jnt_dofadr[hip_joint_id]
hip_actuator_id = m.actuator('torque1').id
knee_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge2")
knee_force_index = m.jnt_dofadr[knee_joint_id]
knee_actuator_id = m.actuator('torque2').id

total_forces = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied
hip_angle_store = []
grf_z = []
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

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 10:
    step_start = time.time()
    #print("slider force:", d.qfrc_applied[force_index])

# Define the desired torque
    continuous_torque_1 = 90
    continuous_torque_2 = 1
    continuous_torque = 300 # Example: Apply a continuous torque of 0.5 Nm

# Simulation loop # Simulate for 100 steps
    # Apply torque continuously within the loop
    #d.qfrc_applied[force_index] = continuous_torque 
    #d.qfrc_applied[knee_force_index] = continuous_torque_2
    for _ in range(10):
       d.ctrl[0] = continuous_torque_2
       #d.ctrl[1] = continuous_torque_2
       #print("d.qfrc_actuator", d.qfrc_actuator)
       
      
       ground_forces = get_ground_contact_forces(m, d)
       #print("ground_forces:", ground_forces)
       first_tuple = ground_forces[0]   
       print("first_tuple:", first_tuple)
    #    print("ground_forces:", ground_forces)
    #    first_tuple = ground_forces[0]
    #    forces= first_tuple[0]
    #    force_z = forces[0]
    #    grf_z.append(force_z)
       

        # d.qfrc_actuator[hip_actuator_id] = continuous_torque_2
        # d.qfrc_actuator[knee_actuator_id] = continuous_torque_1
    #print("actuator_torques", d.qfrc_actuator)
    # print(d.qpos[hip_joint_id])
    # print(d.qpos[knee_joint_id])
    #print(d.qpos[joint_id])
    #print(jacobian(joint_id, hip_joint_id, knee_joint_id))
    
    #hip_angle_store.append(d.qpos[hip_joint_id])  #d.qpos gives angles in radians of the joints
    #base_height_store.append(d.qpos[joint_id])
    mujoco.mj_step(m, d)
    ground_forces = get_ground_contact_forces(m, d)
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