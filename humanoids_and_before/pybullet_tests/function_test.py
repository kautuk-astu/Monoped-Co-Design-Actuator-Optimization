import pybullet as p
import pybullet_data
import time
import numpy as np
import xml.etree.ElementTree as ET
# Connect to PyBullet physics engine (GUI mode)
p.connect(p.GUI)

# Optionally, add PyBullet's default data directory which contains some URDF models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane URDF (or any other URDF you want to load)
plane_id = p.loadURDF("plane.urdf")
# Load a robot URDF (you can replace this with your own URDF file)
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", [0, 0, 0], useFixedBase=True)
urdf = "/home/stochlab/Aastha_pybullet/test_pb/final.urdf"
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
for step in range(1000):
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
        jointIndex=hip_joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=desired_position,
        #force=200 # Maximum force the motor can apply
    )

    # Step the simulation
    p.stepSimulation()
    
    # Sleep to slow down the simulation to real-time (optional)
    time.sleep(1./240.)

def calculate_distance_base_to_foot(robot_id, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
    
    # Get the world position of the base link
        base_link_state = p.getLinkState(robot_id, base_link_index)
        base_position = np.array(base_link_state[0])  # Position is the first element
        base_position[2] = base_position[2] -0.025
        base_quat = base_link_state[1]         # Orientation is the second element
        base_orientation = p.getEulerFromQuaternion(base_quat)  

        # Get the world position and orientation of the calf link
        calf_link_state = p.getLinkState(robot_id, calf_link_index)
        calf_position = np.array(calf_link_state[0])    # position of inertial frame w.r.t. world frame
        calf_quat = calf_link_state[1]           # World orientation (quaternion) of intertial frame
        calf_orientation = p.getEulerFromQuaternion(calf_quat)  # Convert quaternion to Euler angles

        knee_joint_state = p.getJointState(robot_id, knee_joint_index) #theta2
        knee_position = knee_joint_state[0]

        hip_joint_state = p.getJointState(robot_id, hip_joint_index)   #theta1
        hip_position = hip_joint_state[0]

        # Calculate the foot coordinates in the world frame using calf link position and orientation
        foot_x = calf_position[0]
        foot_y = calf_position[1] + 0.1 * np.sin(knee_position + hip_position)
        foot_z = calf_position[2] - 0.1 * np.cos(knee_position + hip_position)
        foot_offset_world = [foot_x, foot_y, foot_z]

        
        foot_position_world = np.array(foot_offset_world)
        
        # Calculate the Euclidean distance from the base to the foot position
        distance = np.linalg.norm(foot_position_world - base_position)
        z_axis_vector = np.array([0, 0, -1]) #negative z axis
        base_to_foot_unit_vector = (foot_position_world - base_position) / distance

    # Calculate the angle between the base_to_foot_unit_vector and z-axis using the dot product
        dot_product = np.dot(base_to_foot_unit_vector, z_axis_vector)
        angle_with_z = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        
        return distance,angle_with_z

def urdf_parser(urdf, axis = 'z'):
        tree = ET.parse(urdf)   # Parse an XML file into an ElementTree object
        root = tree.getroot() 
        return root, tree 

def obtain_calf_length(urdf, axis = 'z'):
    root, tree = urdf_parser(urdf)
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]  # Get the index of the specified axis
    for link in root.findall('link'):
        if link.get('name') == 'calf_link':
                for element in link.findall('visual'):
                    geometry = element.find('geometry')
                    if geometry is not None:
                        cylinder = geometry.find('cylinder')
                        if cylinder is not None:
                            length = cylinder.get('length') #because cylinder has length and radius attribute
                            return length
    return length

def obtain_thigh_length(urdf, axis = 'z'):
    root, tree = urdf_parser(urdf)
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]  # Get the index of the specified axis
    for link in root.findall('link'):
        if link.get('name') == 'thigh_link':
                for element in link.findall('visual'):
                    geometry = element.find('geometry')
                    if geometry is not None:
                        cylinder = geometry.find('cylinder')
                        if cylinder is not None:
                            length = cylinder.get('length') #because cylinder has length and radius attribute
                            return length
    return length

#distance,angle_with_z = calculate_distance_base_to_foot(robot_id)
#print("Distance from base to foot:", distance)
#print("Angle with z-axis:", angle_with_z)

#root, tree = urdf_parser(urdf)
#print(root)
#print(tree)

calf = obtain_calf_length(urdf)
thigh = obtain_thigh_length(urdf)
print("Length of calf link:", calf)
print("Length of thigh link:", thigh)

base_link_state = p.getLinkState(robot_id, 0)
print("Base Link State:", base_link_state)
""" base_position = np.array(base_link_state[0]) # Position is the first element
base_position[2] = base_position[2] -0.025  
base_quat = base_link_state[1]         # Orientation is the second element
base_orientation = p.getEulerFromQuaternion(base_quat)  
print("Base Position:", base_position)
print("Base Orientation:", base_orientation) """

""" base_joint_state = p.getJointState(robot_id, prismatic_joint_index)
base_position, base_velocity, base_reaction_forces, base_applied_torque = base_joint_state


print("Base Joint:")
print("Position:", base_position)
print("Velocity:", base_velocity)
print("Reaction Forces:", base_reaction_forces)
print("Applied Torque:", base_applied_torque) """

p.disconnect()