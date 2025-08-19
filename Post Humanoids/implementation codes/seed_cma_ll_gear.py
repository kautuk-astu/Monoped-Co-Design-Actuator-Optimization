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
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

seed_list = np.linspace(1, 1, num = 1)  # Modify this list for desired seeds
num_seeds = len(seed_list)



for seed in seed_list:
    print(f"\n\n========== Running optimization for SEED = {seed} ==========\n")

    xml_template = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/old_std.xml"
    mass_ik_values_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/final_links_with_mass.csv"  
    date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    best_results_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/post_camera_ready/best_comb_065_035_{date_str}_{seed}.csv"
    all_samples_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/post_camera_ready/all_comb_065_035_{date_str}_{seed}.csv"
    all_runs_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/post_camera_ready/all_runs_{date_str}_{seed}.csv"
    original_bounds = np.array([
        [0.3, 0.5],  # Thigh length
        [0.3, 0.5],  # Calf length
        [4, 8.7], 
        [4, 8.7],
        [0, 200],    # Controller Param 1
        [0, 10], 
        [0, 20]     # Controller Param        # Controller Param 6
    ])

    # Helper Functions
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

    af = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/comb_mass_torque.csv')

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

    def normalize(params):
        return (params - original_bounds[:, 0]) / (original_bounds[:, 1] - original_bounds[:, 0])

    def denormalize(norm_params):
        norm_params = np.clip(norm_params, 0, 1)
        return norm_params * (original_bounds[:, 1] - original_bounds[:, 0]) + original_bounds[:, 0]

    import xml.etree.ElementTree as ET
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
    
    def modify_link_length_and_angle(xml_file, output_file, 
                                    thigh_length, calf_length,
                                    thigh_gear_ratio, knee_gear_ratio,
                                    target_base_height=-0.5):
        
        # Compute joint angles
        #l1, l2 = thigh_length, calf_length
        l1, l2 = 0.4, 0.4  # Default values for testing
        thigh_gear_ratio = 6
        knee_gear_ratio = 6
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

    def round_to_nearest(x, base=0.005):
        return round(base * round(x / base), 3)

    def get_cost(params):
        params = denormalize(params)
        thigh_length, calf_length = 0.4, 0.4  # Default values for testing
        # thigh_length = params[0]
        # calf_length = params[1]    
        other_params = [20,2.5,10]
        #other_params = process_action(params[4:])
        # hip_gear_ratio = params[2]
        # knee_gear_ratio = params[3]
        hip_gear_ratio, knee_gear_ratio = 6,6  # Default values for testing
        controller_params = other_params   
        unique_id = uuid.uuid4().hex[:8]
        
        modified_xml = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/design_xmls/{int(thigh_length * 100000)}_{int(calf_length * 100000)}_{int(hip_gear_ratio*100)}_{int(knee_gear_ratio*100)}_{unique_id}.xml"
        modify_link_length_and_angle(xml_template, modified_xml, thigh_length,calf_length, hip_gear_ratio, knee_gear_ratio)        
        height_values = []
        energy_values = []
        run_entries = []
        ik_value = -0.5
        hip_peak_torque = hip_gear_ratio * 5.5
        knee_peak_torque = knee_gear_ratio * 5.5 * 1.3636
        for i in range(1):
            knee_peak_torque = knee_gear_ratio * 5.5 * 1.3636
            max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy, jump_results = rcp.run(modified_xml, controller_params, thigh_length, calf_length, ik_value, hip_peak_torque, knee_peak_torque)
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
        final_cost = (0.65*(height_cost) + 0.35*(avg_energy))
        with open(all_samples_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([thigh_length, calf_length, hip_gear_ratio, knee_gear_ratio, final_cost, mode_height, avg_energy, jump_results, unique_id] + list(controller_params))
        return final_cost 

    # CMA-ES Optimization
    x0 = normalize(np.array([0.4,0.4,6.0,6.0,100,5,10]))
    sigma0 = 0.1
    opts = cma.CMAOptions()
    opts.set({
        'maxiter': 2, 'popsize': 8, 'seed': int(seed),
        'bounds': [np.zeros(7), np.ones(7)], 'verb_disp': 1000
    })

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Best index", "Best Cost"] + [f"ac{i+1}" for i in range(3)])


    with open(all_samples_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Cost", "Max Height", "Max Energy", "Jump Results", "unique id"] + [f"ac{i+1}" for i in range(3)])


    while not es.stop():
        solutions = es.ask()
        with multiprocessing.Pool(processes=8) as pool:
            costs = pool.map(get_cost, solutions)
        best_index = np.argmin(costs)
        best_params = denormalize(solutions[best_index])
        best_cost = costs[best_index]
        thigh, calf = best_params[0],best_params[1]
        hip_gear_ratio = best_params[2]
        knee_gear_ratio = best_params[3]
        controller_params = process_action(best_params[4:])
        with open(best_results_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([thigh, calf, hip_gear_ratio, knee_gear_ratio, best_index, best_cost] + list(controller_params))
        es.tell(solutions, costs)
        


    print(f"Joint optimization completed for seed {seed}.")
    print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    #store time taken in the end of best results file
    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        #subtract start time from end time
        end_time = datetime.now()
        # start_time = datetime.strptime(es.start_time, "%Y-%m-%d %H:%M:%S")
        # start_time = 
        # time_taken = (end_time - start_time).total_seconds()
        writer.writerow(["Time taken (seconds)"])
        writer.writerow([end_time.strftime("%Y-%m-%d %H:%M:%S")])


