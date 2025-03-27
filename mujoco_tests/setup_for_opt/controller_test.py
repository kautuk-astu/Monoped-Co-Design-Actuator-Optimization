import numpy as np
import pybullet as p
import pybullet_data
import vmc_jump as vmc
import vmc_jump_var as vmc_var
import time
# Example Usage:
# Initialize PyBullet simulation and load a robot URDF
p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane as the ground (optional)
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("/home/stochlab/Aastha_pybullet/test_pb/final.urdf", [0,0,0],useFixedBase=False)
urdf = "/home/stochlab/Aastha_pybullet/test_pb/final.urdf"
store_knee_pos = []
store_i = []    
store_knee_torque = []
torque1 =[]
torque2 = []
base_pos = []
fz_store = []
# Specify the indices for the hip and knee joints
hip_joint_index = 1
knee_joint_index = 2
p.setGravity(0, 0, -9.8)
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=knee_joint_index,
    controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
    force=0  # Set force to 0 to disable it initially
)

p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=hip_joint_index,
    controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
    force=0  # Set force to 0 to disable it initially
)
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=0,
    controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
    force=0  # Set force to 0 to disable it initially
)

# Create a controller for the 2-DOF leg

#controller = vmc.Controller(urdf, robot_id)
controller = vmc_var.Controller(urdf, robot_id)


# Main simulation loop
for _ in range(2000):
    #p.stepSimulation()
    
    # Update controller
    #height = controller.step()
    #print(height)
    #controller.test()
    """ base_position, base_orientation = controller.obtain_base_posorn()
    print(base_position)
    print(base_orientation) """
    """ distance, angle_with_vertical = controller.calculate_distance_base_to_foot()
    print(distance)
    print(angle_with_vertical)
     """
    J = controller.jacobian()
    #print(J)
    tau = controller.joint_torques()
    Fz, Fy = controller.foot_force()
    fz_store.append(Fz/100)
    #print("hip torque:",tau[0])
    print("knee torque:",tau[1])
    """ if tau[1] > 50:
        tau[1] = 50
    elif tau[1] < -50:
        tau[1] = -50

    if tau[0] > 50:
        tau[0] = 50
    elif tau[0] < -50:
        tau[0] = -50
 """





    """ p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=hip_joint_index,
        controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
        force=0  # Set force to 0 to disable it initially
        )

    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=knee_joint_index,
        controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
        force=0  # Set force to 0 to disable it initially
        ) """
    
    """ p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=hip_joint_index,
        controlMode=p.TORQUE_CONTROL,
        force=tau[0]
    ) """
    for step in range(2):
        hip_state = p.getJointState(robot_id, hip_joint_index)
        base_link_state = p.getLinkState(robot_id, 0)
        prismatic_joint_pos = p.getJointState(robot_id, 0)[0]
        #print("prismatic_joint_pos:",prismatic_joint_pos)
        print("base_link_state:",base_link_state[0])
        base_pos.append(base_link_state[0][2])

        #print(base_link_state[0])
        #print(hip_state[0])
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=0,
            controlMode=p.TORQUE_CONTROL,
            force=-35
        )
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=hip_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=tau[0]
        )
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=knee_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=tau[1]
            )
        hip_state = p.getJointState(robot_id, hip_joint_index)
        knee_state = p.getJointState(robot_id, knee_joint_index)
        #base_pos.append(prismatic_joint_pos)
        torque2.append(tau[1])
        torque1.append(tau[0])
        store_knee_torque.append(knee_state[3])
        #print("torque1:", tau[0])
        #print("torque2:", tau[1])
        print("torque1", "torque2", "hip_pos", "knee_pos", "hip_torque", "knee_torque")
        print(tau[0], tau[1], hip_state[0], knee_state[0], hip_state[3], knee_state[3])

        #print("hip_torque:", hip_state[3])
        #print("hip_state:", hip_state[0])
        #print("knee_state:", knee_state[0])
        #print("knee_torque:", knee_state[3])
        store_knee_pos.append(hip_state[0])

        p.stepSimulation()
        # (Optional) Add some delay for better visualization in GUI mode
        time.sleep(1/240)

    #p.stepSimulation()
    #time.sleep(1/240)

# Disconnect PyBullet
p.disconnect()

#plotting the knee joint position
import matplotlib.pyplot as plt
plt.plot(base_pos)
plt.xlabel('time')
plt.ylabel('base position')
plt.title('Base position')
plt.show()
# plt.plot(fz_store)
# plt.xlabel('time')
# plt.ylabel('foot force')
# plt.title('Foot force')
# plt.show()
# #plot to show both base position and fz in one plot
# plt.plot(base_pos)
# plt.plot(fz_store)
# plt.xlabel('time')
# plt.ylabel('base position and foot force')
# plt.title('Base position and Foot force')
# plt.show()

# plt.plot(store_knee_torque)
# plt.xlabel('time')
# plt.ylabel('knee applied torque')
# plt.title('Knee applied torque')
# plt.show()
plt.plot(torque2)
plt.xlabel('time')
plt.ylabel('knee calculated torque')
plt.title('Knee calculated torque')
plt.show()
# plt.plot(torque1)
# plt.xlabel('time')
# plt.ylabel('hip calculated torque')
# plt.title('Hip calculated torque')
# plt.show()
# plt.plot(base_pos)
# plt.plot(store_knee_torque)
# plt.plot(torque2)
# plt.xlabel('time')
# #plt.legend(['base position', 'knee applied torque', 'knee calculated torque'])
# plt.title('Torque applied only to knee joint - Observations')
# plt.show()
# plt.savefig('torque_only_knee.png')

