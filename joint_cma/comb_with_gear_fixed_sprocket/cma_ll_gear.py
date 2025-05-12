import csv
import cma
import numpy as np
import xml.etree.ElementTree as ET
from collections import Counter
import rcp
import os
import sys
import multiprocessing
import uuid
from datetime import datetime

sys.path.append(os.path.abspath('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/rcp.py'))


# Constants
xml_template = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/modified_robot.xml"
mass_ik_values_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/final_links_with_mass.csv"  
best_results_file = "best_joint_results.csv"
all_samples_file = "all_samples.csv"
all_runs_file = "all_runs.csv"
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

def modify_link_length(xml_file, link_name, new_length, axis='z', 
                       output_file="modified_robot.xml", 
                       hip_peak_torque=None, knee_peak_torque=None):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]

    for link in root.findall('worldbody'):
        for slot in link.findall('body'):
            for new in slot.findall('body'):
                if new.get('name') == link_name:
                    pos = new.get('pos').split()
                    pos[axis_index] = str(-new_length)
                    new.set('pos', ' '.join(pos))
                    for element in new.findall('geom'):
                        fromto = element.get('fromto').split()
                        fromto[5] = str(new_length)
                        element.set('fromto', ' '.join(fromto))
                    for joint in new.findall('joint'):
                        pos = joint.get('pos').split()
                        pos[axis_index] = str(new_length)
                        joint.set('pos', ' '.join(pos))
                else:
                    for link in new.findall('body'):
                        if link.get('name') == link_name:
                            pos = link.get('pos').split()
                            pos[axis_index] = str(-new_length)
                            link.set('pos', ' '.join(pos))
                            for element in link.findall('geom'):
                                fromto = element.get('fromto').split()
                                fromto[5] = str(new_length)
                                element.set('fromto', ' '.join(fromto))
                            for joint in link.findall('joint'):
                                pos = joint.get('pos').split()
                                pos[axis_index] = str(new_length)
                                joint.set('pos', ' '.join(pos))

    # Modify peak torque through ctrlrange
    for motor in root.findall(".//motor"):
        motor_name = motor.get("name")
        if motor_name == "torque1" and hip_peak_torque is not None:
            motor.set("ctrlrange", f"-{hip_peak_torque} {hip_peak_torque}")
        elif motor_name == "torque2" and knee_peak_torque is not None:
            motor.set("ctrlrange", f"-{knee_peak_torque} {knee_peak_torque}")

    tree.write(output_file)
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
    thigh_length = round_to_nearest(params[0])
    calf_length = round_to_nearest(params[1])
    mass_wo_g, ik_value = mass_ik_dict[(thigh_length, calf_length)]
    other_params = process_action(params[4:])
    hip_gear_ratio = np.round(params[2],1)
    knee_gear_ratio = np.round(params[3],1)
    controller_params = other_params
    hip_mass, hip_peak_torque = get_mass_and_torque(hip_gear_ratio, "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_with_gear/comb_mass_torque.csv")
    knee_mass, knee_peak_torque = get_mass_and_torque(knee_gear_ratio, "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_with_gear/comb_mass_torque.csv")
    mass = mass_wo_g + hip_mass + knee_mass
    unique_id = uuid.uuid4().hex[:8]
    modified_xml = f"modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}_{int(hip_gear_ratio*10)}_{int(knee_gear_ratio*10)}_{unique_id}.xml"
    modify_link_length(xml_template, "link1", thigh_length, output_file=modified_xml, hip_peak_torque=hip_peak_torque, knee_peak_torque=knee_peak_torque*1.3636)
    modify_link_length(modified_xml, "link2", calf_length, output_file=modified_xml, hip_peak_torque=hip_peak_torque, knee_peak_torque=knee_peak_torque*1.3636)
    #mass, ik_value = mass_ik_dict[(thigh_length, calf_length)]    
    height_values = []
    energy_values = []
    run_entries = []
    for i in range(1):
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy = rcp.run(modified_xml, controller_params, mass, ik_value)
        base_max = np.round(max_height_after_control, 2)
        te = np.round(max_jump_energy, 2)
        height_values.append(base_max)
        energy_values.append(te)
        run_entries.append([i + 1, thigh_length, calf_length, base_max, te] + list(controller_params))      
    
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
    height_cost = 3 * np.exp(-mode_height)
    final_cost = 0.5*(np.round(height_cost,1)) + 0.5*(np.round(avg_energy, 1))
    # with open(all_runs_file, "a", newline="") as file:
    #     writer = csv.writer(file)
    #         #writer.writerow([i+1, thigh_length, calf_length, final_cost, mode_height, avg_energy] + list(controller_params))
    #     writer.writerows(run_entries)
    with open(all_samples_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, hip_gear_ratio, knee_gear_ratio, final_cost, mode_height, avg_energy] + list(controller_params))
    return final_cost 

# CMA-ES Optimization
x0 = normalize(np.array([0.4,0.4,6.35,6.35,30,2.5,10]))
sigma0 = 0.1
opts = cma.CMAOptions()
opts.set({
    'tolfun': 0.000005, 'tolx': 0.000001, 'maxiter': 100, 'popsize': 10, 'seed': 42,
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
    writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Cost", "Max Height", "Max Energy"] + [f"ac{i+1}" for i in range(3)])

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
    es.tell(solutions, costs)
    best_index = np.argmin(costs)
    best_params = denormalize(solutions[best_index])
    best_cost = costs[best_index]
    thigh, calf = round_to_nearest(best_params[0]), round_to_nearest(best_params[1])
    hip_gear_ratio = np.round(best_params[2],1)
    knee_gear_ratio = np.round(best_params[3],1)
    controller_params = process_action(best_params[4:])
    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh, calf, hip_gear_ratio, knee_gear_ratio, best_index, best_cost] + list(controller_params))


print("Joint optimization complete.")
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

