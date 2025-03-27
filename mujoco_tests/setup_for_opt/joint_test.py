import pybullet as p
import pybullet_data
import time

# Connect to PyBullet physics engine (GUI mode)
p.connect(p.GUI)

# Optionally, add PyBullet's default data directory which contains some URDF models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane URDF (or any other URDF you want to load)
plane_id = p.loadURDF("plane.urdf")
# Load a robot URDF (you can replace this with your own URDF file)
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", [0, 0, 0], useFixedBase=True)

# Set gravity for the simulation
p.setGravity(0, 0, -9.8)

# Define the prismatic joint index or name
prismatic_joint_index = 0 # You can change this based on your URDF setup
hip_joint = 1  
knee_joint = 2

# Print joint information to find the prismatic joint (for debugging)
""" for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {joint_info[1].decode('utf-8')}") """

# Start simulation
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)  # Use step simulation mode

# Define the desired position for the prismatic joint (in meters)
desired_position = 0.52
desired_pos_2 = -0.52*2  # Set this to the desired position along the prismatic axis

# Define the control loop to move the prismatic joint
for step in range(100000):
    # Apply position control to the prismatic joint
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=knee_joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=desired_pos_2,
        force=200 # Maximum force the motor can apply
    )
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=prismatic_joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=desired_position,
        #force=200 # Maximum force the motor can apply
    )
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=hip_joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=desired_position,
        #force=200 # Maximum force the motor can apply
    )

    # Step the simulation
    p.stepSimulation()
    
    # Sleep to slow down the simulation to real-time (optional)
    time.sleep(1./240.)

# Disconnect after simulation

base_joint_state = p.getJointState(robot_id, prismatic_joint_index)
base_position, base_velocity, base_reaction_forces, base_applied_torque = base_joint_state


print("Base Joint:")
print("Position:", base_position)
print("Velocity:", base_velocity)
print("Reaction Forces:", base_reaction_forces)
print("Applied Torque:", base_applied_torque)

p.disconnect()