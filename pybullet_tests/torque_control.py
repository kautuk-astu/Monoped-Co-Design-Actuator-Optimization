import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet (GUI mode)
p.connect(p.GUI)

# Optionally set search path to default PyBullet data (for plane, etc.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane as the ground (optional)
plane_id = p.loadURDF("plane.urdf")

# Load your robot URDF (replace with actual path to your URDF file)
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/planar_2R_robot.urdf", [0, 0, 0], useFixedBase=False)
# Set gravity in the environment
p.setGravity(0, 0, -9.8)
store = []
store_hip_torque = []
store_knee_pos = []
store_knee_torque = []
store_hip_pos = []

# Enable torque control for the revolute joint (index assumed to be 0)
joint_index = 2
hip_index = 1 # Replace with the actual index of your revolute joint
prismatic_index = 0

# Disable the default motor control (so we can apply custom torques)
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_index,
    controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
    force=0  # Set force to 0 to disable it initially
)

p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=hip_index,
    controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
    force=0  # Set force to 0 to disable it initially
)

# Simulation loop to apply torque to the joint
for step in range(1000):
    # Apply a torque value to the revolute joint (in Nm)
    torque1 = 10
    torque2 = -10 # Adjust torque value to your needs (positive or negative)
    
    # Use p.applyExternalTorque or setJointMotorControl2 with TORQUE_CONTROL
    """ p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=joint_index,
        controlMode=p.TORQUE_CONTROL,
        force=torque1  # This is the torque to be applied to the joint
    ) """
    for i in range(2):
        
        p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=hip_index,
        controlMode=p.TORQUE_CONTROL,
        force=torque1  # This is the torque to be applied to the joint
        )
        p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=joint_index,
        controlMode=p.TORQUE_CONTROL,
        force=torque2  # This is the torque to be applied to the joint
        )
        applied_hip_torque = p.getJointState(robot_id, hip_index)[3]
        print("torque1:", torque1)
        print("hip_applied_torque:", applied_hip_torque)
        store_hip_torque.append(applied_hip_torque)
        applied_knee_torque = p.getJointState(robot_id, joint_index)[3]
        print("torque2:", torque2)
        print("knee_applied_torque:", applied_knee_torque)    
        store_knee_torque.append(applied_knee_torque)
        p.stepSimulation()
    
    # Slow down the simulation for visualization (optional)
        time.sleep(1/24)
    hip_pos = p.getJointState(robot_id, hip_index)[0]
    
    #print(joint_torque)
    
    store.append(hip_pos)
    knee_pos = p.getJointState(robot_id, joint_index)[0]
    store_knee_pos.append(knee_pos)
    # Step the simulation
    

# Disconnect after simulation
p.disconnect()
import matplotlib.pyplot as plt
#plt.plot(store)
plt.plot(store)
plt.show()
plt.savefig('hip_pos.png')
plt.plot(store_hip_torque)
plt.savefig('hip_torque.png')
plt.show()


