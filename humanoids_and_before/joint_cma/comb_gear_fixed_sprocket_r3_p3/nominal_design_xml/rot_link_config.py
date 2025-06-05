# import xml.etree.ElementTree as ET
# import numpy as np
# import len_rad_id as lri
# import mujoco as mj

# def inverse_kinematics(x, y, l1, l2):
#     dist = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
#     if abs(dist) > 1:
#         raise ValueError("No solution: Target is out of reach")
#     theta2 = np.arctan2(np.sqrt(1 - dist**2), dist)
#     theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
#     return theta1, theta2

# def compute_position(length, angle):
#     x = length * np.sin(angle)
#     y = 0
#     z = -length * np.cos(angle)
#     return f"{x:.6f} {y:.6f} {z:.6f}"

# def axis_angle_y(theta):
#     return f"0 1 0 {theta:.6f}"

# def modify_robot_with_angles(xml_path, l1, l2, mass1, mass2, target_base_height=0.5, output_path="modified_robot_configured.xml"):
#     tree = ET.parse(xml_path)
#     root = tree.getroot()

#     # Compute joint angles
#     theta1, theta2 = inverse_kinematics(0, target_base_height, l1, l2)

#     # Compute positions
#     link2_pos = compute_position(l1, theta1)

#     for body in root.findall(".//body"):
#         name = body.get("name")
        
#         if name == "link1":
#             body.set("pos", "0 0 0")
#             for geom in body.findall("geom"):
#                 geom.set("fromto", f"0 0 0 0 0 {-l1:.6f}")
#                 geom.set("mass", f"{mass1:.6f}")
#             for joint in body.findall("joint"):
#                 joint.set("pos", "0 0 0")
#                 joint.set("axis", "0 1 0")
        
#         elif name == "link2":
#             body.set("pos", link2_pos)
#             for geom in body.findall("geom"):
#                 geom.set("fromto", f"0 0 0 0 0 {l2:.6f}")
#                 geom.set("mass", f"{mass2:.6f}")
#             for joint in body.findall("joint"):
#                 joint.set("pos", "0 0 0")
#                 joint.set("axis", "0 1 0")

#     tree.write(output_path)
#     print(f"Modified XML saved to {output_path}")
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
import len_rad_id as lri


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

def modify_link_length_and_angle(xml_file, output_file, 
                                 thigh_length, calf_length,
                                 thigh_mass=None, calf_mass=None,
                                 hip_peak_torque=None, knee_peak_torque=None,
                                 target_base_height=-0.55):
    
    # Compute joint angles
    l1, l2 = thigh_length, calf_length
    theta1, theta2 = inverse_kinematics(0, target_base_height, l1, l2)
    #print(f"Computed angles: theta1={theta1}, theta2={theta2}")
    print(theta2-theta1)

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
    print(f"theta1d: {theta1d}")
    theta2d = theta2 * 180 / np.pi

    for worldbody in root.findall('worldbody'):
        for torso in worldbody.findall('body'):
            torso.set('pos', f"0 0 {-target_base_height}")
            #.set('pos', f"0 0 0.5")
            for thigh in torso.findall('body'):
                if thigh.get('name') == "link1":
                    # update body pos
                    #thigh.set('pos', f"{knee_pos[0]} {knee_pos[1]} {knee_pos[2]}")
                    thigh.set('pos', "0 0 0")
                    thigh.set('euler', f"0 {(np.pi*0.5)-theta1} 0")
                    #thigh.set('euler', "0 0 0")
                    # update fromto
                    for geom in thigh.findall('geom'):
                        if geom.get('name') == "thigh":                            
                            geom.set('fromto', f"0 0 0 {l1} 0 0")
                            if thigh_mass:
                                geom.set('mass', str(thigh_mass))
                    for joint in thigh.findall('joint'):                        
                        joint.set('pos', "0 0 0")
                        #joint.set('pos', f"{-(thigh_vec[0])} 0 {-(thigh_vec[2])}")
                        #geom.set('fromto', f"0 0 0 {thigh_vec[0]} 0 {thigh_vec[2]}")
                for calf in thigh.findall('body'):
                    if calf.get('name') == "link2":                        
                        calf.set('pos', f"{l1} 0 0")
                        calf.set('euler', f"0 {-theta2} 0")
                        # update body pos
                        #calf.set('pos', f"{shank_vec[0]} {shank_vec[1]} {shank_vec[2]}")
                        #calf.set('pos', f"{knee_pos[0]} {knee_pos[1]} {knee_pos[2]}")
                        # update fromto
                        for geom in calf.findall('geom'):
                            if geom.get('name') == "shank":                            
                                geom.set('fromto', f"0 0 0 {l2} 0 0")                            
                                if calf_mass:
                                        geom.set('mass', str(calf_mass))
                            #geom.set('fromto', f"0 0 {calf_offset} {-(shank_vec[0])} {-(shank_vec[1])} {-(shank_vec[2])}")
                            #geom.set('fromto', f"{calf_x_offset} 0 {calf_offset} {(shank_vec[0])} 0 {(shank_vec[2])}")
                            #geom.set('fromto', f"0 0 0 {(shank_vec[0])} 0 {(shank_vec[2])}")
                        for joint in calf.findall('joint'):
                            #joint.set('pos', f"{-(shank_vec[0])} 0 {-(shank_vec[2])}")
                            joint.set('pos', "0 0 0")
    # Update ctrlrange if needed
    for motor in root.findall(".//motor"):
        if motor.get("name") == "torque1" and hip_peak_torque:
            motor.set("ctrlrange", f"-{hip_peak_torque} {hip_peak_torque}")
        elif motor.get("name") == "torque2" and knee_peak_torque:
            motor.set("ctrlrange", f"-{knee_peak_torque} {knee_peak_torque}")

    tree.write(output_file)
    print(f"Modified XML saved to: {output_file}")

# xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/nominal_design_xml/modified_robot_400_450.xml'
# m = mj.MjModel.from_xml_path(xml_path)
# d = mj.MjData(m)
# lp = lri.len_rad_id(xml_path, m, d)
# l1n = lp.thigh_total_length()
# l2n = lp.calf_total_length()
l1n = 0.3
l2n = 0.5
print(f"l1n: {l1n}, l2n: {l2n}")   
modify_link_length_and_angle(
    xml_file="modified_robot_400_400.xml", output_file= "modified_robot_configured_300_500.xml", thigh_length = l1n, calf_length = l2n, hip_peak_torque=100, knee_peak_torque=100,
    target_base_height=-0.5
)

# xml_new_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/nominal_design_xml/modified_robot_400_400.xml'
#     m_n = mj.MjModel.from_xml_path(xml_new_path)
#     d_n = mj.MjData(m_n)
#     lp = lri.len_rad_id(xml_new_path, m_n, d_n)
#     # Reset simulation state for consistency
#     mj.mj_resetData(m_n, d_n)
#     mj.mj_resetData(m, d)
#     # Increase solver precision for more stable results
#     l1n = lp.thigh_total_length()
#     l2n = lp.calf_total_length()
#     theta1, theta2 = ik.inverse_kinematics(0, ik_value, l1n, l2n)

