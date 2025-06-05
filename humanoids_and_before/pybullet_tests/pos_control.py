import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet (GUI mode)
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Load a plane as the ground (optional)
plane = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.8)

hip_joint_index = 1
knee_joint_index = 2

# Simulation parameters
timestep = 1.0 / 240.0
p.setTimeStep(timestep)

# Desired position and control gains for the PD controller
desired_positions = {hip_joint_index: 0.0, knee_joint_index: 0.0}
Kp = 1.0  # Proportional gain
Kd = 0.1  # Derivative gain

# Sinusoidal trajectory parameters
amplitude = 0.1
frequency = 0.5

# Main simulation loop
for t in range(1000):
    # Update the base position with a sinusoidal trajectory
    base_position = [0, 0, amplitude * np.sin(2 * np.pi * frequency * t * timestep)]
    p.resetBasePositionAndOrientation(robot_id, base_position, [0, 0, 0, 1])

    # Define the target foot position relative to the base
    target_foot_position = [base_position[0] + 0.2, base_position[1], base_position[2] - 0.2]

    # Inverse kinematics to calculate joint angles for the desired foot position
    joint_angles = p.calculateInverseKinematics(
        robot_id,
        knee_joint_index,      # End effector (foot) link index
        target_foot_position,  # Desired foot position in world coordinates
    )

    # Apply position control to reach the calculated joint angles
    p.setJointMotorControl2(robot_id, hip_joint_index, p.POSITION_CONTROL, targetPosition=joint_angles[hip_joint_index], force=100, positionGain=Kp, velocityGain=Kd)
    p.setJointMotorControl2(robot_id, knee_joint_index, p.POSITION_CONTROL, targetPosition=joint_angles[knee_joint_index], force=100, positionGain=Kp, velocityGain=Kd)

    # Step the simulation
    p.stepSimulation()
    time.sleep(timestep)

# Disconnect after the loop
p.disconnect()
