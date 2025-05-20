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

    print(f"theta1: {theta1}, theta2: {theta2}, theta1n: {theta1}")
    return theta1, theta2



#Use the inverse_kinematics function to compute the angles for given lengths
l1 = 0.5
l2 = 0.5
x = 0
z = -0.5
theta1, theta2 = inverse_kinematics(x, z, l1, l2)
theta1 = theta1 * 180 / np.pi
theta2 = theta2 * 180 / np.pi
print(f"Computed angles: theta1={theta1}, theta2={theta2}")
# thigh_vec = [l1 * np.sin(theta1), 0, -l1 * np.cos(theta1)]
# knee_pos = thigh_vec
# shank_vec = [-l2 * np.sin(-theta1 + theta2), 0, 
#                  -l2 * np.cos(-theta1 + theta2)]
# print(f"Thigh vector: {thigh_vec}")
# print(f"Knee position: {knee_pos}")
# print(f"Shank vector: {shank_vec}")
# print(f"Computed angles: theta1={theta1}, theta2={theta2}")

