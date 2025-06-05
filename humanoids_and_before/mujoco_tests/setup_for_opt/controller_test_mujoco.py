import numpy as np
import mujoco as mj
import vmc_mujoco as vmc
import time
import mujoco.viewer as mj_viewer

#xml_path = 'test_pb/monoped_hopper_inspired.xml'
xml_path = 'test_pb/sleeping_monoped_3.xml' 
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)
# Initialize the controller
controller = vmc.Controller(xml_path, m, d)
joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "rootz")
force_index = m.jnt_dofadr[joint_id]
hip_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "thigh_joint")
hip_force_index = m.jnt_dofadr[hip_joint_id]
knee_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "leg_joint")
knee_force_index = m.jnt_dofadr[knee_joint_id]
gcf_store = []
base_height_store = []
hip_angle_store = []
knee_angle_store = []
hip_torque_store = []
knee_torque_store = []
orientation_store = []
distance_store = []


# Main simulation loop
with mj.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:

    step_start = time.time()    
    #print(time.time())
    
    mj.mj_step(m, d)
    #initialize the controller
    
    #knee_pos = controller.knee_position()
    #length = controller.thigh_length()
    #child = controller.obtain_child()
    #root, tree = controller.xml_parser()
    #if d.xpos[2][2] < 0.5:
    joint_torque = controller.joint_torque()
    #print("joint_torque:", joint_torque)
    hip_torque_store.append(joint_torque[0])
    knee_torque_store.append(joint_torque[1])
    for _ in range(10):
      d.qfrc_applied[hip_force_index] = joint_torque[0]
      d.qfrc_applied[knee_force_index] = joint_torque[1]
      #mj.mj_step(m, d)
    base_height_store.append(d.qpos[joint_id])
    #JOINT ANGLE UPDATE CHECKED!! WORKS FINE 
    hip_angle_store.append(d.qpos[hip_joint_id])
    knee_angle_store.append(d.qpos[knee_joint_id])

    gcf = controller.get_ground_contact_forces()
    print("gcf:", gcf)
    #ORIENTATION CHECKED!! WORKS FINE
    #print("equivalent orientation", controller.equivalent_orientation())
    orientation_store.append(controller.equivalent_orientation())
    #DISTANCE CHECKED!! WORKS FINE
    #print("distance:", controller.distance())
    distance_store.append(controller.distance())

    #print("gcf:", gcf)
    applied_force = controller.force_applied_ground() 
    #print("applied_force:", applied_force)
    #exit()

    gcf_store.append(gcf)

    # force_applied_ground = controller.force_applied_ground() 
    # print("force_applied_ground:", force_applied_ground)
    # jacobian_transpose = controller.jacobian_transpose()
    # print("jacobian_transpose:", jacobian_transpose)
    #controller.obtain_thigh_element()
    # print("root:", root)
    # print("tree:", tree)
    # #print("child", child)
    # #print("length:", length)
    # print("knee_pos:", knee_pos)
    
 
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
 
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
 
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      #time.sleep(time_until_next_step)
      time.sleep(1)

#plot the base height wrt time
import matplotlib.pyplot as plt
#print(gcf_store)
#plt.plot(hip_torque_store)
# plt.plot(hip_angle_store)
# plt.plot(knee_angle_store)
# plt.show()
# plt.plot(gcf_store[0])
# plt.show()
plt.plot(base_height_store)
#plt.plot(gcf_store)
plt.show()
# plt.plot(gcf_store)
# plt.show()
# #plt.plot(hip_angle_store)
# plt.show()
