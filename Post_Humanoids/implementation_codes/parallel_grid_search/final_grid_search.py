import csv
import cma
import numpy as np
import xml.etree.ElementTree as ET
import rcp  # Your script for running simulations
import csv
from collections import Counter
import multiprocessing
import uuid
from datetime import datetime
#from Post_Humanoids.controller_codes import final_rcp as rcp
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import sys, os

# Define grid search range
# thigh_lengths = np.linspace(0.250, 0.450, 9)  #both inclusive
# calf_lengths = np.linspace(0.250, 0.450, 9)
start = 0.25
stop = 0.45
step = 0.025

# Calculate the exact values and round
grid_thigh_lengths = [round(start + i * step, 3) for i in range(int((stop - start) / step) + 1)]
grid_calf_lengths = grid_thigh_lengths.copy()  # Since they have the same range

# Convert to numpy array if needed
grid_thigh_lengths = np.array(grid_thigh_lengths)
grid_calf_lengths = np.array(grid_calf_lengths)

print(grid_thigh_lengths)
print(grid_calf_lengths)

date_st = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
xml_template = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/modified_robot_400_400_60_60.xml"
# File for storing the best results for each link length combination
best_results_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/1D/ll/gs_150/best_results_{date_st}.csv"
# Load Mass & IK Values from CSV
def load_mass_ik_values(csv_filename):
    mass_ik_dict = {}
    with open(csv_filename, newline='') as file:
        reader = csv.DictReader(file)
        for row in reader:
            thigh = float(row["thigh_length"])
            calf = float(row["calf_length"])
            mass = float(row["mass"])
            ik_value = float(row["ik_value"])
            mass_ik_dict[(thigh, calf)] = (mass, ik_value)
    return mass_ik_dict

# Initialize CSV with headers if empty
with open(best_results_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["thigh_length", "calf_length", "best_cost"] + [f"ac{i+1}" for i in range(6)])

np.random.seed(42)
original_bounds = np.array([
    [5, 600],    # Param 1
    [0, 5],      # Param 2
    [0, 5],      # Param 3
    [5, 20],     # Param 4
    [1, 20],     # Param 5
    [0, 1]       # Param 6
])
#print(original_bounds[:,0])
#print(original_bounds[:,1])
# 🔹 Define the normalized range [0,1]
norm_range = np.array([0, 1])


def normalize(params):
    """Scales parameters from their original range to [0,1]."""
    return (params - original_bounds[:, 0]) / (original_bounds[:, 1] - original_bounds[:, 0])

def denormalize(norm_params):
    """Scales parameters from [0,1] back to their original range."""
    norm_params = np.clip(norm_params, 0, 1)
    return (norm_params * (original_bounds[:, 1] - original_bounds[:, 0])) + original_bounds[:, 0]

def inverse_kinematics(x, z, l1, l2, branch=1):
        c2 = (x**2 + z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(c2) > 1:
            raise ValueError("Target out of reach")
        if branch == 1:
            theta2 = np.arctan2(-np.sqrt(1 - c2**2), c2)
        else:   
            theta2 = np.arctan2(np.sqrt(1 - c2**2), c2)
        A = l1 + l2 * c2
        B = l2 * np.sin(theta2)
        theta1 = np.arctan2(A*x + B*z, x*B - A*z)
        return theta1, theta2

dft = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/thigh_025_045.csv')

    # Sort by gear ratio to avoid interpolation issues
dft = dft.sort_values(by='Link Length (mm)')

# Extract gear ratio and mass
thigh_lengths_interp = dft['Link Length (mm)'].values/1000
thigh_masses_interp = dft['Calculated Mass (kg)'].values

# Create a linear interpolation function without extrapolation
thigh_interp_func = interp1d(
    thigh_lengths_interp,
    thigh_masses_interp,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)


dfc = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/calf_025_045.csv')

# Sort by gear ratio to avoid interpolation issues
dfc = dfc.sort_values(by='Link Length (mm)')

# Extract gear ratio and mass
calf_lengths_interp = dfc['Link Length (mm)'].values/1000
calf_masses_interp = dfc['Calculated Mass (kg)'].values

# Create a linear interpolation function without extrapolation
calf_interp_func = interp1d(
    calf_lengths_interp,
    calf_masses_interp,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)

af = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/data/comb_mass_torque.csv')

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

# XML Modification Function
def modify_link_length_and_angle(xml_file, output_file, 
                                    thigh_length, calf_length,
                                    thigh_gear_ratio, knee_gear_ratio,
                                    target_base_height=-0.35):
        
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
                for geom in torso.findall('geom'):
                    if torso_mass:
                        geom.set('mass', str(torso_mass))
                for thigh in torso.findall('body'):
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

# Function to compute cost (Runs controller 5 times per action)
def get_reward(action, log_file, mode_data_file, thigh_length, calf_length):     
    num_runs = 1
    height_values = []
    energy_values = []
    run_entries = []
    unique_id = uuid.uuid4().hex[:8]  
    hip_gear_ratio = 6.0
    knee_gear_ratio = 6.0
    modified_xml = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/ll_xmls/{int(thigh_length * 100000)}_{int(calf_length * 100000)}_{int(hip_gear_ratio*100)}_{int(knee_gear_ratio*100)}_{unique_id}.xml"
    modify_link_length_and_angle(xml_template, modified_xml, thigh_length,calf_length, hip_gear_ratio, knee_gear_ratio)
    action = denormalize(action)
    action = np.round(action, 1)
    ik_value = -0.35
    hip_peak_torque = hip_gear_ratio * 5.5
    knee_peak_torque = knee_gear_ratio * 5.5 * 1.3636
    for run_id in range(num_runs):  
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, max_jump_joule_energy, max_jump_mech_energy,jump_results = rcp.run(modified_xml, action, thigh_length, calf_length, ik_value, hip_peak_torque, knee_peak_torque)
        base_max =max_height_after_control
        te = max_jump_energy
        height_values.append(base_max)
        energy_values.append(te)
        run_entries.append([run_id + 1, thigh_length, calf_length, base_max, te] + list(action))

    # Compute mode height & average energy
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
    jump_results = [result for height, result in zip(height_values, jump_results) if height == mode_height]
    height_cost = 150 * np.exp(-(0.35+mode_height))
    final_cost = (0.65*(height_cost) + 0.35*(avg_energy))
    # Log individual runs in separate file
    # with open(log_file, "a", newline="") as file:
    #     writer = csv.writer(file)
    #     writer.writerows(run_entries)

    # Log mode height, avg energy & cost
    with open(mode_data_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, final_cost, mode_height, avg_energy, max_jump_joule_energy, max_jump_mech_energy,jump_results, unique_id] + list(action))

    return final_cost

# Function to optimize controller parameters for a given design
def optimize_for_design(thigh_length, calf_length):
    seed = 0
    date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    #modified_xml = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/main xmls/modified_robot_400_400_60_60.xml"
    # Update thigh length first
    # modify_link_length(xml_template, "link1", thigh_length, output_file=modified_xml)
    # # Now update calf length using the updated file
    # modify_link_length(modified_xml, "link2", calf_length, output_file=modified_xml)
    log_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/1D/ll/gs_065_035_runs_{date_str}_{seed}_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"
    mode_data_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/1D/ll/gs_150/all_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"
    progress_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/1D/ll/gs_150/best_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"

    # Initialize CMA-ES
    x0 = normalize(np.array([100,2.5,2.5,12.5,10,0.5]))  # Initial guess in normalized space
    sigma0 = 0.1
    opts = cma.CMAOptions()
    opts.set({
        'maxiter': 100, 'popsize': 8, 'seed': int(seed),
        'bounds': [np.zeros(6), np.ones(6)], 'verb_disp': 1000
    })

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

    # Add headers for logs
    with open(progress_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Thigh", "Calf", "Best index", "Best Cost"] + [f"ac{i+1}" for i in range(len(x0))])

    with open(mode_data_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Thigh", "Calf", "Cost", "Max Height", "Max Energy", "Max Joule", "Max Mech", "Jump Results", "unique id"] + [f"ac{i+1}" for i in range(len(x0))])

    while not es.stop():
        solutions = es.ask()
        tasks = [(sol, log_file, mode_data_file, thigh_length, calf_length) for sol in solutions]  
        with multiprocessing.Pool(processes=8) as pool:
            costs = pool.starmap(get_reward, tasks)
        best_index = np.argmin(costs)
        best_params = denormalize(solutions[best_index])
        best_cost = costs[best_index]

        with open(progress_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([thigh_length, calf_length, best_index, best_cost] + list(best_params))

        #print("Iteration", es.countiter, "Best Cost =", best_cost)
        es.tell(solutions, costs)

    best_params = es.result.xbest
    best_cost = get_reward(best_params, log_file, mode_data_file, thigh_length, calf_length)
    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, best_cost] + list(best_params))
    return best_params, best_cost

# Run Grid Search
for thigh_length in grid_thigh_lengths:
    for calf_length in grid_calf_lengths:
        print("Optimizing for Thigh:", thigh_length, "Calf:", calf_length)
        best_params, best_cost = optimize_for_design(thigh_length, calf_length)
        

print("Grid Search and Optimization Completed")
