import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt

# Connect to PyBullet physics engine (GUI mode)
p.connect(p.GUI)

# Optionally, add PyBullet's default data directory which contains some URDF models
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Load the plane URDF (or any other URDF you want to load)
plane_id = p.loadURDF("plane.urdf")
# Load a robot URDF (you can replace this with your own URDF file)
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/planar_2R_robot.urdf", useFixedBase=False)
print("robot_id:",robot_id)
#obtain links information using getlink state
num_links = p.getNumJoints(robot_id)
for link_index in range(num_links):
    link_info = p.getLinkState(robot_id, link_index)
    print("Link", link_index, ":", link_info) 


p.setGravity(0, 0, -9.8)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./1000)

# Disconnect from PyBullet physics engine
p.disconnect()