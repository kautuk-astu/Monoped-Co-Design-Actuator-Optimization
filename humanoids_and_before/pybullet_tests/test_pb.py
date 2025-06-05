import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
import numpy as np

# Connect to PyBullet physics engine (GUI mode)
p.connect(p.GUI)
# Optionally, add PyBullet's default data directory which contains some URDF models
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Load the plane URDF (or any other URDF you want to load)
plane_id = p.loadURDF("plane.urdf")
# Load a robot URDF (you can replace this with your own URDF file)
angle = p.getQuaternionFromEuler([0, np.pi/4, np.pi/4])
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", [0,0,0], useFixedBase=False)
#robotnew_id = p.loadMJCF("/home/stochlab/Aastha_pybullet/test_pb/newhopper.xml")
# Specify the indices for the hip and knee joints
num_joints = p.getNumJoints(robot_id)
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_id = joint_info[0]  # The joint ID
    print(f"Joint {joint_index}: ID = {joint_id}, Name = {joint_info[1].decode('utf-8')}") 

p.setGravity(0, 0, -9.81)
prismatic_joint = 0
hip_joint = 1
knee_joint = 2
# prismatic_joint = 1
# hip_joint = 2
# knee_joint = 3

store_base_z = []
store_hip_applied_torque = []
store_knee_applied_torque = []
store_hip_pos = []
store_knee_pos = []
store_base_applied_torque = []
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=prismatic_joint,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0) 
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=hip_joint,
    controlMode=p.VELOCITY_CONTROL,
    force=0)
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=knee_joint,
    controlMode=p.VELOCITY_CONTROL,
    force=0)

# p.setJointMotorControl2(
#     bodyIndex=robot_id,
#     jointIndex=hip_joint,
#     controlMode=p.TORQUE_CONTROL,
#     force=0)
# p.setJointMotorControl2(
#     bodyIndex=robot_id,
#     jointIndex=knee_joint,
#     controlMode=p.TORQUE_CONTROL,
#     force=0)
# p.setJointMotorControl2(
#     bodyIndex=robot_id,
#     jointIndex=prismatic_joint,
#     controlMode=p.TORQUE_CONTROL,
#     force=0)


# 

# Simulate for 1000 timesteps
for i in range(10000000):
    # knee_joint = 2
    # knee_torque = p.getJointState(robot_id, knee_joint)[3]
    # #print("knee_torque:",knee_torque)
    # hip_joint = 1
    # hip_torque = p.getJointState(robot_id, hip_joint)[3]
    # #print("hip_torque:",hip_torque)
    # torque = 0 

    base_joint = 0
    #base_pos = p.getBasePositionAndOrientation(robot_id)[0][2]
    #print("base_pos:",base_pos)
    # prism_pos = p.getJointState(robot_id, base_joint)[0]
    # print("prism_pos:",prism_pos)
    base_torque = p.getJointState(robot_id, base_joint)[3]
    print("base_force:",base_torque)
    hip_joint = 1
    hip_torque = p.getJointState(robot_id, hip_joint)[3]
    print("hip_force:",hip_torque)
    knee_joint = 2
    knee_torque = p.getJointState(robot_id, knee_joint)[3]
    print("knee_force:",knee_torque)
    # print("base_torque:",base_torque)
    for _ in range(2):
        torque = -0
        p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=knee_joint,
        controlMode=p.TORQUE_CONTROL,
        force=torque) 
        p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=hip_joint,
        controlMode=p.TORQUE_CONTROL,
        force=torque)
        p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=prismatic_joint,
        controlMode=p.TORQUE_CONTROL,
        force=-35)
     # This is the torque to be applied to the joint
    # base_pos = p.getBasePositionAndOrientation(robot_id)[0][2]
    # #print("base_pos:",base_pos)
    # store_base_z.append(base_pos)
    # prismatic_force = p.getJointState(robot_id, prismatic_joint)[3]
    # print("prismatic_force:",prismatic_force)

    #base_torque = p.getJointState(robot_id, prismatic_joint)[3]
    # #print("base_torque:",base_torque)
    # #store_base_applied_torque.append(base_torque)
    # hip_applied_torque = p.getJointState(robot_id, hip_joint)[3]
    # hip_pos = p.getJointState(robot_id, hip_joint)[0]
    # print("hip_pos:",hip_pos)
    # store_hip_pos.append(hip_pos)
    # print("hip_applied_torque:",hip_applied_torque)
    # store_hip_applied_torque.append(hip_applied_torque)
    # knee_applied_torque = p.getJointState(robot_id, knee_joint)[3]
    # knee_pos = p.getJointState(robot_id, knee_joint)[0]
    # print("knee_pos:",knee_pos)
    # store_knee_pos.append(knee_pos)
    # print("knee_applied_torque:",knee_applied_torque)
    # store_knee_applied_torque.append(knee_applied_torque)
    # hip_joint = 1
    # #print all joint indices
    # print(p.getNumJoints(robot_id))
    # #print all joint info
    # print(p.getJointInfo(robot_id, hip_joint))
    # p.setJointMotorControl2(
    #     bodyIndex=robot_id,
    #     jointIndex=hip_joint,
    #     controlMode=p.POSITION_CONTROL,
    #     targetPosition=0.2,
    #     force=500)

    p.stepSimulation()  # Step the simulation forward
    time.sleep(1./240)  # Sleep to make the simulation run at real-time speed
 
# Disconnect when done
p.disconnect()

# plt.plot(store_base_z)
# plt.xlabel('time')
# plt.ylabel('base position')
# plt.title('Base z position')
# plt.show()
# plt.savefig('base_z_freefall_1m.png')
# plt.plot(store_hip_applied_torque)
# plt.xlabel('time')
# plt.ylabel('hip applied torque')
# plt.title('Hip applied torque')
# plt.savefig('hip_applied_torque_freefall_1m.png')
# plt.show()
# plt.plot(store_knee_applied_torque)
# plt.xlabel('time')
# plt.ylabel('knee applied torque')
# plt.title('Knee applied torque')
# plt.savefig('knee_applied_torque_freefall_1m.png')
# plt.show()
# plt.plot(store_hip_applied_torque)
# plt.plot(store_base_z)
# plt.plot(store_knee_applied_torque)
# plt.xlabel('time')
# plt.ylabel('base position, hip applied torque, knee applied torque')
# plt.title('Base z position and hip applied torque and knee applied torque')
# plt.legend(['hip_applied_torque', 'base_position', 'knee_applied_torque'])
# plt.savefig('base_z_and_hip_knee_applied_torque_freefall_1m.png')
# plt.show()
# plt.plot(store_hip_pos)
# plt.plot(store_knee_pos)
# plt.plot(store_base_z)
# plt.xlabel('time')
# plt.ylabel('hip position, knee position, base position')
# plt.title('Hip,knee and base position')
# plt.legend(['hip_pos', 'knee_pos', 'base_pos'])
# plt.savefig('hip_knee_base_position_freefall_1m.png')
# plt.show()
