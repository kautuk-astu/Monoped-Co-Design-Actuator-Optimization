import numpy as np
import mujoco as mj 
import len_rad_id as lri
import time

def inverse_kinematics(x, z, l1, l2, branch=1):
    c2 = (x**2 + z**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(c2) > 1:
        raise ValueError("Target out of reach")
    if branch == 1:
        theta2 = np.arctan2(-np.sqrt(1 - c2**2), c2)
    else:   
        theta2 = np.arctan2(np.sqrt(1 - c2**2), c2)
    #theta1 = np.arctan2(z, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    A = l1 + l2 * c2
    B = l2 * np.sin(theta2)
    theta1 = np.arctan2(A*x + B*z, x*B - A*z)

    #print(f"theta1: {theta1}, theta2: {theta2}, theta1n: {theta1}")
    return theta1, theta2


# Example usage
#l1, l2 = 1.0, 1.0  # Link lengths
# x, y = 0.52, 0  # Desired end-effector position
# xml_path = '/home/stochlab/Aastha_Coopt_Monoped/maxh_with_ik/thigh_50/525/modified_robot.xml'
# m = mj.MjModel.from_xml_path(xml_path)
# d = mj.MjData(m)
# lri = lri.len_rad_id(xml_path, m, d)
# l1 = lri.thigh_total_length()
# l2 = lri.calf_total_length()

# #l1, l2 = lri.len_rad_id('/home/stochlab/Aastha_Coopt_Monoped/mujoco_tests/hopper_roc.xml', mj, d)

# theta1, theta2 = inverse_kinematics(x, y, l1, l2)
# print (theta1, theta2)
# print(f"Theta1: {np.degrees(theta1):.2f} degrees, Theta2: {np.degrees(theta2):.2f} degrees")
# dist = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
# print(dist)
