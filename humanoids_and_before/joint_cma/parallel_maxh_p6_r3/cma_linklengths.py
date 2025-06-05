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
    [5, 200],        # Controller Param 1
    [0, 1], 
    [0, 5],
    [1, 20],        # Controller Param 2
    [1, 10],
    [0, 1]     # Controller Param            # Controller Param 6
])

# Helper Functions
def normalize(params):
    return (params - original_bounds[:, 0]) / (original_bounds[:, 1] - original_bounds[:, 0])

def denormalize(norm_params):
    norm_params = np.clip(norm_params, 0, 1)
    return norm_params * (original_bounds[:, 1] - original_bounds[:, 0]) + original_bounds[:, 0]

def modify_link_length(xml_file, link_name, new_length, axis='z', output_file="modified_robot.xml"):
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

    tree.write(output_file)


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
    controller_params = process_action(params[2:])
    
    unique_id = uuid.uuid4().hex[:8]
    modified_xml = f"modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}_{unique_id}.xml"
    modify_link_length(xml_template, "link1", thigh_length, output_file=modified_xml)
    modify_link_length(modified_xml, "link2", calf_length, output_file=modified_xml)
    mass, ik_value = mass_ik_dict[(thigh_length, calf_length)]    
    height_values = []
    energy_values = []
    run_entries = []
    for i in range(3):
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
    height_cost = 10 * np.exp(-mode_height)
    final_cost = np.round(height_cost, 2)
    # with open(all_runs_file, "a", newline="") as file:
    #     writer = csv.writer(file)
    #         #writer.writerow([i+1, thigh_length, calf_length, final_cost, mode_height, avg_energy] + list(controller_params))
    #     writer.writerows(run_entries)
    with open(all_samples_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, final_cost, mode_height, avg_energy] + list(controller_params))
    return final_cost 

# CMA-ES Optimization
x0 = normalize(np.array([0.4,0.4,31.9,0.4,2.2,12.8,2.9,0.4]))
sigma0 = 0.2
opts = cma.CMAOptions()
opts.set({
    'tolfun': 0.005, 'tolx': 0.001, 'maxiter': 100, 'popsize': 10, 'seed': 42,
    'bounds': [np.zeros(5), np.ones(5)], 'verb_disp': 1000
})

es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

with open(best_results_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Thigh", "Calf", "Best Cost"] + [f"ac{i+1}" for i in range(6)])

with open(all_runs_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["run count", "Thigh", "Calf", "Max Height", "Max Energy"] + [f"ac{i+1}" for i in range(6)])

with open(all_samples_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Thigh", "Calf", "Cost", "Max Height", "Max Energy"] + [f"ac{i+1}" for i in range(6)])

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
    with multiprocessing.Pool(processes=7) as pool:
        costs = pool.map(get_cost, solutions)
    es.tell(solutions, costs)
    best_index = np.argmin(costs)
    best_params = denormalize(solutions[best_index])
    best_cost = costs[best_index]
    thigh, calf = round_to_nearest(best_params[0]), round_to_nearest(best_params[1])
    controller_params = process_action(best_params[2:])
    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh, calf, best_cost] + list(controller_params))


print("Joint optimization complete.")

