import csv
import cma
import numpy as np
import xml.etree.ElementTree as ET
from collections import Counter
#import rcp
import os
import sys
import multiprocessing
import uuid
from datetime import datetime
import final_rcp as rcp

#sys.path.append(os.path.abspath('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/rcp.py'))


# Constants
xml_template = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/design_xmls/modified_robot_400_400.xml"
mass_ik_values_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/final_links_with_mass.csv"  
date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
best_results_file = f"best_comb_065_035_{date_str}.csv"
all_samples_file = f"all_comb_065_035_{date_str}.csv"
all_runs_file = f"all_runs_{date_str}.csv"
original_bounds = np.array([
    [0.300, 0.500],  # Thigh length
    [0.300, 0.500],  # Calf length
    [4, 8.7], 
    [4, 8.7],
    [5, 200],        # Controller Param 1
    [0, 5], 
    [0, 20]     # Controller Param            # Controller Param 6
])

# Helper Functions
def normalize(params):
    return (params - original_bounds[:, 0]) / (original_bounds[:, 1] - original_bounds[:, 0])

def denormalize(norm_params):
    norm_params = np.clip(norm_params, 0, 1)
    return norm_params * (original_bounds[:, 1] - original_bounds[:, 0]) + original_bounds[:, 0]

import xml.etree.ElementTree as ET

# def modify_link_length(xml_file, link_name, new_length, axis='z', 
#                        output_file="modified_robot.xml", 
#                        hip_peak_torque=None, knee_peak_torque=None):
#     tree = ET.parse(xml_file)
#     root = tree.getroot()
#     axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]

#     for link in root.findall('worldbody'):
#         for slot in link.findall('body'):
#             for new in slot.findall('body'):
#                 if new.get('name') == link_name:
#                     pos = new.get('pos').split()
#                     pos[axis_index] = str(-new_length)
#                     new.set('pos', ' '.join(pos))
#                     for element in new.findall('geom'):
#                         fromto = element.get('fromto').split()
#                         fromto[5] = str(new_length)
#                         element.set('fromto', ' '.join(fromto))
#                     for joint in new.findall('joint'):
#                         pos = joint.get('pos').split()
#                         pos[axis_index] = str(new_length)
#                         joint.set('pos', ' '.join(pos))
#                 else:
#                     for link in new.findall('body'):
#                         if link.get('name') == link_name:
#                             pos = link.get('pos').split()
#                             pos[axis_index] = str(-new_length)
#                             link.set('pos', ' '.join(pos))
#                             for element in link.findall('geom'):
#                                 fromto = element.get('fromto').split()
#                                 fromto[5] = str(new_length)
#                                 element.set('fromto', ' '.join(fromto))
#                             for joint in link.findall('joint'):
#                                 pos = joint.get('pos').split()
#                                 pos[axis_index] = str(new_length)
#                                 joint.set('pos', ' '.join(pos))

#     # Modify peak torque through ctrlrange
#     for motor in root.findall(".//motor"):
#         motor_name = motor.get("name")
#         if motor_name == "torque1" and hip_peak_torque is not None:
#             motor.set("ctrlrange", f"-{hip_peak_torque} {hip_peak_torque}")
#         elif motor_name == "torque2" and knee_peak_torque is not None:
#             motor.set("ctrlrange", f"-{knee_peak_torque} {knee_peak_torque}")

#     tree.write(output_file)

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
def modify_link_length_and_angle(xml_file, output_file, 
                                 thigh_length, calf_length,
                                 thigh_mass=None, calf_mass=None,
                                 hip_peak_torque=None, knee_peak_torque=None,
                                 target_base_height=-0.5):
    
    # Compute joint angles
    l1, l2 = thigh_length, calf_length
    theta1, theta2 = inverse_kinematics(0, target_base_height, l1, l2)
    #print(f"Computed angles: theta1={theta1}, theta2={theta2}")
    #print(theta2-theta1)

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
    #print(f"theta1d: {theta1d}")
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
    #print(f"Modified XML saved to: {output_file}")
import pandas as pd

def get_mass_and_torque(gear_ratio, csv_path):
    """
    Returns mass and torque corresponding to the given gear ratio
    by rounding it to 1 decimal and looking it up in the CSV.
    
    Parameters:
    - gear_ratio (float): The gear ratio for which to get values.
    - csv_path (str): Path to the CSV with columns: gear_ratio, mass, peak_torque

    Returns:
    - mass (float), peak_torque (float)
    """
    # Round to 1 decimal place
    rounded_ratio = np.round(gear_ratio, 1)
    
    # Load the CSV
    df = pd.read_csv(csv_path)

    # Check if the rounded_ratio exists
    row = df[df['gear_ratio'] == rounded_ratio]

    if row.empty:
        raise ValueError(f"Gear ratio {rounded_ratio} not found in CSV.")
    
    mass = float(row['mass'].values[0])
    torque = float(row['peak_torque'].values[0])

    return mass, torque




def process_action(action):
    action = action.copy()
    for i in range(len(action)):
        action[i] = np.round(action[i], 1)
    return action

def load_mass_ik_values(filename):
    mass_ik_dict = {}
    with open(filename, newline='') as file:
        reader = csv.DictReader(file)
        for row in reader:
            thigh = float(row["thigh_length"])
            calf = float(row["calf_length"])
            mass = float(row["mass"])
            ik_value = float(row["ik_value"])
            mass_ik_dict[(thigh, calf)] = (mass, ik_value)
    return mass_ik_dict

mass_ik_dict = load_mass_ik_values(mass_ik_values_file)
def round_to_nearest(x, base=0.005):
    return round(base * round(x / base), 3)

def get_cost(params):
    params = denormalize(params)
    #thigh_length = round(params[0], 2)
    #calf_length = round(params[1], 2)
    thigh_length = round(float(params[0]),3)
    calf_length = round(float(params[1]),3)
    mass_wo_g, ik_value = mass_ik_dict[(thigh_length, calf_length)]
    other_params = process_action(params[4:])
    hip_gear_ratio = np.round(params[2],1)
    knee_gear_ratio = np.round(params[3],1)
    controller_params = other_params
    hip_mass, hip_peak_torque = get_mass_and_torque(hip_gear_ratio, "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/comb_mass_torque.csv")
    knee_mass, knee_peak_torque = get_mass_and_torque(knee_gear_ratio, "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/comb_mass_torque.csv")
    mass = mass_wo_g + hip_mass + knee_mass
    unique_id = uuid.uuid4().hex[:8]
    #modified_xml = f"modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}_{int(hip_gear_ratio*10)}_{int(knee_gear_ratio*10)}_{unique_id}.xml"
    modified_xml = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/design_xmls/modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}_{int(hip_gear_ratio*10)}_{int(knee_gear_ratio*10)}_{unique_id}.xml"
    modify_link_length_and_angle(xml_template, modified_xml, thigh_length,calf_length, hip_peak_torque=hip_peak_torque, knee_peak_torque=knee_peak_torque*1.3636)
    #modify_link_length(modified_xml, "link2", calf_length, output_file=modified_xml, hip_peak_torque=hip_peak_torque, knee_peak_torque=knee_peak_torque*1.3636)
    #mass, ik_value = mass_ik_dict[(thigh_length, calf_length)]    
    height_values = []
    energy_values = []
    run_entries = []
    for i in range(1):
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results = rcp.run(modified_xml, controller_params, thigh_length, calf_length, mass, ik_value, hip_peak_torque, knee_peak_torque)
        base_max =max_height_after_control
        te = max_jump_energy
        height_values.append(base_max)
        energy_values.append(te)
        run_entries.append([i + 1, thigh_length, calf_length, base_max, te] + list(controller_params))      
    
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
    jump_results = [result for height, result in zip(height_values, jump_results) if height == mode_height]
    height_cost = 100 * np.exp(-(0.5+mode_height))
    #final_cost = 0.85*(np.round(height_cost,3)) + 0.15*(np.round(avg_energy, 5))
    final_cost = (0.65*(height_cost) + 0.35*(avg_energy))
    # with open(all_runs_file, "a", newline="") as file:
    #     writer = csv.writer(file)
    #         #writer.writerow([i+1, thigh_length, calf_length, final_cost, mode_height, avg_energy] + list(controller_params))
    #     writer.writerows(run_entries)
    with open(all_samples_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, hip_gear_ratio, knee_gear_ratio, mass, hip_peak_torque, knee_peak_torque*1.3636, final_cost, mode_height, avg_energy, jump_results] + list(controller_params))
    return final_cost 

# CMA-ES Optimization
x0 = normalize(np.array([0.4,0.4,6.0,6.0,50,2.5,10]))
sigma0 = 0.1
opts = cma.CMAOptions()
# opts.set({
#     'tolfun': 0.000005, 'tolx': 0.000001, 'maxiter': 500, 'popsize': 10, 'seed': 42,
#     'bounds': [np.zeros(7), np.ones(7)], 'verb_disp': 1000
# })
opts.set({
    'maxiter': 500, 'popsize': 8, 'seed': 42,
    'bounds': [np.zeros(7), np.ones(7)], 'verb_disp': 1000
})

es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

with open(best_results_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Best index", "Best Cost"] + [f"ac{i+1}" for i in range(3)])

with open(all_runs_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["run count", "Thigh", "Calf", "Max Height", "Max Energy"] + [f"ac{i+1}" for i in range(3)])

with open(all_samples_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "mass", "hip peak torque", "knee peak torque", "Cost", "Max Height", "Max Energy", "Jump Results"] + [f"ac{i+1}" for i in range(3)])

# while not es.stop():
#     solutions = es.ask()
#     costs = [get_cost(sol) for sol in solutions]
#     es.tell(solutions, costs)
#     best_index = np.argmin(costs)
#     best_params = denormalize(solutions[best_index])
#     best_cost = costs[best_index]
#     #thigh, calf = round(best_params[0], 2), round(best_params[1], 2)
#     thigh,calf = round_to_nearest(best_params[0]), round_to_nearest(best_params[1])
#     controller_params = process_action(best_params[2:])
#     with open(best_results_file, "a", newline="") as file:
#         writer = csv.writer(file)
#         writer.writerow([thigh, calf, best_cost] + list(controller_params))
while not es.stop():
    solutions = es.ask()
    with multiprocessing.Pool(processes=8) as pool:
        costs = pool.map(get_cost, solutions)
    best_index = np.argmin(costs)
    best_params = denormalize(solutions[best_index])
    best_cost = costs[best_index]
    thigh, calf = round(float(best_params[0]),3), round(float(best_params[1]),3)
    hip_gear_ratio = np.round(best_params[2],1)
    knee_gear_ratio = np.round(best_params[3],1)
    controller_params = process_action(best_params[4:])
    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh, calf, hip_gear_ratio, knee_gear_ratio, best_index, best_cost] + list(controller_params))
    es.tell(solutions, costs)
    


print("Joint optimization complete.")
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
#03:58:40 2025
#seed=42, Thu May 15 03:45:46 2025

