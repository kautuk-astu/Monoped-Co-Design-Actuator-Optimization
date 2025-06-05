import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet (GUI mode)
p.connect(p.GUI)
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.8)

hip_joint_index = 1
knee_joint_index = 2
# position control for hip and knee joints
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=hip_joint_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.5,  # Target position to reach
    force=1  # Maximum force to apply
)
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=knee_joint_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.5,  # Target position to reach
    force=1  # Maximum force to apply
)
p.setRealTimeSimulation(1)
time.sleep(10)  # Run simulation for 10 seconds
p.disconnect()