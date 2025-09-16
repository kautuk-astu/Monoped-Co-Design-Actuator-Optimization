import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
import len_rad_id as lri
from scipy.interpolate import interp1d
import pandas as pd


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

dft = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/thigh_03_05.csv')

# Sort by gear ratio to avoid interpolation issues
dft = dft.sort_values(by='Link Length (mm)')

# Extract gear ratio and mass
thigh_lengths = dft['Link Length (mm)'].values/1000
thigh_masses = dft['Calculated Mass (kg)'].values

# Create a linear interpolation function without extrapolation
thigh_interp_func = interp1d(
    thigh_lengths,
    thigh_masses,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)


dfc = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/calf_03_05.csv')

# Sort by gear ratio to avoid interpolation issues
dfc = dfc.sort_values(by='Link Length (mm)')

# Extract gear ratio and mass
calf_lengths = dfc['Link Length (mm)'].values/1000
calf_masses = dfc['Calculated Mass (kg)'].values

# Create a linear interpolation function without extrapolation
calf_interp_func = interp1d(
    calf_lengths,
    calf_masses,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)

af = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Final_humanoids_results/comb_mass_torque.csv')

# Sort by gear ratio to avoid interpolation issues
af = af.sort_values(by='gear_ratio')

# Extract gear ratio and mass
gear_ratios = af['gear_ratio'].values
act_masses = af['mass'].values

# Create a linear interpolation function without extrapolation
act_interp_func = interp1d(
    gear_ratios,
    act_masses,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)


# def modify_link_length_and_angle(xml_file, output_file, 
#                                  thigh_length, calf_length,
#                                 thigh_gear_ratio, knee_gear_ratio,
#                                  target_base_height=-0.5):
    
#     # Compute joint angles
#     l1, l2 = thigh_length, calf_length
#     hip_peak_torque = thigh_gear_ratio*5.5
#     knee_peak_torque = knee_gear_ratio*5.5*1.3636
#     theta1, theta2 = inverse_kinematics(0, target_base_height, l1, l2)
#     #print(f"Computed angles: theta1={theta1}, theta2={theta2}")
#     #print(theta2-theta1)

#     # Compute (x, z) vector for thigh and shank using rotation
#     thigh_vec = [l1 * np.sin(theta1), 0, -l1 * np.cos(theta1)]
#     knee_pos = thigh_vec
#     shank_vec = [l2 * np.sin(theta1 + theta2), 0, 
#                  -l2 * np.cos(theta1 + theta2)]

#     tree = ET.parse(xml_file)
#     root = tree.getroot()
#     calf_offset = -0.025*np.cos(theta1 + theta2)
#     calf_x_offset = 0.025*np.sin(theta1 + theta2)
#     theta1d = theta1 * 180 / np.pi
#     #print(f"theta1d: {theta1d}")
#     theta2d = theta2 * 180 / np.pi
#     hip_actuator_mass = act_interp_func(thigh_gear_ratio)
#     knee_actuator_mass = act_interp_func(knee_gear_ratio)
#     torso_mass = hip_actuator_mass  + knee_actuator_mass
#     # torso_mass = 0.0
#     # thigh_link_mass = 0
#     thigh_link_mass = thigh_interp_func(thigh_length) 
#     calf_link_mass = calf_interp_func(calf_length) 
#     #calf_link_mass = 0
#     for worldbody in root.findall('worldbody'):
#         for torso in worldbody.findall('body'):
#             torso.set('pos', f"0 0 {-target_base_height}")
#             for geom in torso.findall('geom'):
#                 if torso_mass:
#                     geom.set('mass', str(torso_mass))
#             for thigh in torso.findall('body'):
#                 if thigh.get('name') == "link1":
#                     thigh.set('pos', "0 0 0")
#                     thigh.set('euler', f"0 {(np.pi*0.5)-theta1} 0")
#                     for geom in thigh.findall('geom'):
#                         if geom.get('name') == "thigh":                            
#                             geom.set('fromto', f"0 0 0 {l1} 0 0")
#                             if thigh_link_mass:
#                                 geom.set('mass', str(thigh_link_mass))
#                     for joint in thigh.findall('joint'):                        
#                         joint.set('pos', "0 0 0")
#                 for calf in thigh.findall('body'):
#                     if calf.get('name') == "link2":                        
#                         calf.set('pos', f"{l1} 0 0")
#                         calf.set('euler', f"0 {-theta2} 0")                        
#                         for geom in calf.findall('geom'):
#                             if geom.get('name') == "shank":                            
#                                 geom.set('fromto', f"0 0 0 {l2} 0 0")                            
#                                 if calf_link_mass:
#                                         geom.set('mass', str(calf_link_mass))                            
#                         for joint in calf.findall('joint'):                            
#                             joint.set('pos', "0 0 0")
#     # Update ctrlrange if needed
#     for motor in root.findall(".//motor"):
#         if motor.get("name") == "torque1" and hip_peak_torque:
#             motor.set("ctrlrange", f"-{hip_peak_torque} {hip_peak_torque}")
#         elif motor.get("name") == "torque2" and knee_peak_torque:
#             motor.set("ctrlrange", f"-{knee_peak_torque} {knee_peak_torque}")

#     tree.write(output_file)

def modify_link_length_and_angle(xml_file, output_file, 
                                    thigh_length, calf_length,
                                    thigh_gear_ratio, knee_gear_ratio,
                                    target_base_height=-0.5):
        
        # Compute joint angles
        l1, l2 = thigh_length, calf_length
        #l1, l2 = 0.4, 0.4  # Default values for testing
        hip_peak_torque = thigh_gear_ratio*5.5
        knee_peak_torque = knee_gear_ratio*5.5*1.3636
        theta1, theta2 = inverse_kinematics(0, target_base_height, l1, l2)   

        # Compute (x, z) vector for thigh and shank using rotation
        thigh_vec = [l1 * np.sin(theta1), 0, -l1 * np.cos(theta1)]
        knee_pos = thigh_vec
        shank_vec = [l2 * np.sin(theta1 + theta2), 0, 
                    -l2 * np.cos(theta1 + theta2)]

        tree = ET.parse(xml_file)
        root = tree.getroot()
        calf_offset = -0.025*np.cos(theta1 + theta2)
        calf_x_offset = 0.025*np.sin(theta1 + theta2)
        theta1d = theta1 * 180 / np.pi
        theta2d = theta2 * 180 / np.pi
        hip_actuator_mass = act_interp_func(thigh_gear_ratio)
        knee_actuator_mass = act_interp_func(knee_gear_ratio)
        torso_mass = hip_actuator_mass + knee_actuator_mass
        #torso_mass = 0.0
        # thigh_link_mass = 0
        # calf_link_mass = 0
        thigh_link_mass = thigh_interp_func(thigh_length) 
        calf_link_mass = calf_interp_func(calf_length)
        for worldbody in root.findall('worldbody'):
            for torso in worldbody.findall('body'):
                torso.set('pos', f"0 0 {-target_base_height}")
                for base in torso.findall('body'):
                    for geom in base.findall('geom'):
                        if torso_mass:
                            geom.set('mass', str(torso_mass))
                    for thigh in base.findall('body'):
                        if thigh.get('name') == "link1":
                            thigh.set('pos', "0 0 0")
                            thigh.set('euler', f"0 {(np.pi*0.5)-theta1} 0")
                            for geom in thigh.findall('geom'):
                                if geom.get('name') == "thigh":                            
                                    geom.set('fromto', f"0 0 0 {l1} 0 0")
                                    if thigh_link_mass:
                                        geom.set('mass', str(thigh_link_mass))
                            for joint in thigh.findall('joint'):                        
                                joint.set('pos', "0 0 0")
                        for calf in thigh.findall('body'):
                            if calf.get('name') == "link2":                        
                                calf.set('pos', f"{l1} 0 0")
                                calf.set('euler', f"0 {-theta2} 0")                        
                                for geom in calf.findall('geom'):
                                    if geom.get('name') == "shank":                            
                                        geom.set('fromto', f"0 0 0 {l2} 0 0")                            
                                        if calf_link_mass:
                                            geom.set('mass', str(calf_link_mass))                            
                                for joint in calf.findall('joint'):                            
                                    joint.set('pos', "0 0 0")
        # Update ctrlrange if needed
        for motor in root.findall(".//motor"):
            if motor.get("name") == "torque1" and hip_peak_torque:
                motor.set("ctrlrange", f"-{hip_peak_torque} {hip_peak_torque}")
            elif motor.get("name") == "torque2" and knee_peak_torque:
                motor.set("ctrlrange", f"-{knee_peak_torque} {knee_peak_torque}")

        tree.write(output_file)


l1n = 0.3371880322006635
l2n = 0.30000119897805666
gt = 4.000000040018973

gk = 8.699999987222025
print(f"l1n: {l1n}, l2n: {l2n}")   
modify_link_length_and_angle(
    xml_file="/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/main xmls/modified_robot_planar.xml", output_file= "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/main xmls/test_planar.xml", thigh_length = l1n, calf_length = l2n, thigh_gear_ratio = gt, knee_gear_ratio=gk)

