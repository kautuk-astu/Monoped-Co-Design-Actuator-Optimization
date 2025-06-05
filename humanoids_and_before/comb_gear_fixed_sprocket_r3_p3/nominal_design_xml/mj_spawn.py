import mujoco as mj
import mujoco.viewer
import numpy as np

import mujoco as mj
import numpy as np
import len_rad_id as lri

ixml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/nominal_design_xml/modified_robot_400_400.xml'
m = mj.MjModel.from_xml_path(ixml_path)
d = mj.MjData(m)
lp = lri.len_rad_id(ixml_path, m, d)
l1n = lp.thigh_total_length()
l2n = lp.calf_total_length()
mj.mj_resetData(m, d)
# def inverse_kinematics(x, y, l1, l2):
#     dist = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
#     if abs(dist) > 1:
#         raise ValueError("Target out of reach")
#     theta2 = np.arctan2(np.sqrt(1 - dist**2), dist)
#     theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
#     return theta1, theta2
def inverse_kinematics(x, y, l1, l2):
    """
    Solves for the joint angles (theta1, theta2) of a 2-DOF planar manipulator given end-effector position (x, y)
    relative to the base.
    
    :param x: X-coordinate of end-effector
    :param y: Y-coordinate of end-effector
    :param l1: Length of first link
    :param l2: Length of second link
    :return: Tuple (theta1, theta2) in radians
    """
    
    # Compute inverse kinematics using the law of cosines
    dist = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    if abs(dist) > 1:
        raise ValueError("No solution: Target is out of reach")
    
    theta2 = np.arctan2(np.sqrt(1 - dist**2), dist)  # Elbow-up solution
    theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    
    return theta1, theta2

def test_model(xml_path):
    # Load model and data
    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)

    # Set initial state
    mj.mj_resetData(model, data)

    # Set target joint positions (current position as target)
    theta1, theta2 = inverse_kinematics(0.5, 0, l1n, l2n)
    #print(f"Computed angles: theta1={theta1}, theta2={theta2}")  
    target_qpos = data.qpos.copy()
    print(data.qpos)
    theta1 = target_qpos[1]
    theta2 = target_qpos[2]

    print(f"Target angles: theta1={theta1}, theta2={theta2}")

    # PD gains for each joint (tune as needed)
    #kp = np.array([50.0] * model.nu)   # Proportional gains
    #kd = np.array([2.0] * model.nu)
    #     # Derivative gains
    kp = 100
    kd = 10

    # Viewer
    with mj.viewer.launch_passive(model, data) as viewer:
        print("Viewer launched. Press ESC to quit.")

        while viewer.is_running():
            # PD Control
            #qpos_err = np.array([theta1,theta2]) - data.qpos
            hip_error = 0 - data.qpos[1]
            knee_error = 0 - data.qpos[2]
            hip_torque = 100 * hip_error - 10 * data.qvel[1]
            knee_torque = 100 * knee_error - 10 * data.qvel[2]
            torque = np.array([hip_torque, knee_torque])
            #print(torque)
            # # print(data.qpos[0])
            # # print(data.qpos[1])
            # # print(data.qpos[2])

            # # Apply torques
            data.ctrl[:] = torque
            
            # print(d.ctrl[0])
            # print(d.ctrl[1])

            # Step simulation
            mj.mj_step(model, data)
            #print(data.qvel[0])
            #print(data.qpos[0])
            #print(data.qpos[0])
            #print(data.qpos[1], data.qpos[2])
            print(data.qpos[2])
            #print(d.qpos[0])
            viewer.sync()

if __name__ == "__main__":
    test_model("modified_robot_configured_2_60.xml")
