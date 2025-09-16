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
#from Post_Humanoids.controller_codes import final_rcp as rcp
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import sys, os
sys.path.append("/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization")
from Post_Humanoids.controller_codes import final_rcp_planar as rcp


#sys.path.append(os.path.abspath('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/rcp.py'))

coefficient_sets = []
for first_coeff in np.arange(0.4, 0.81, 0.05):  # 0.4 to 0.8 with step 0.05
    second_coeff = 1.0 - first_coeff
    coefficient_sets.append((first_coeff, second_coeff))

seed_list = np.linspace(5, 5, num=1)  # Modify this list for desired seeds
num_seeds = len(seed_list)
for coeff_set in coefficient_sets:
    coeff1, coeff2 = coeff_set
    coeff_str = f"{int(coeff1*100):03d}_{int(coeff2*100):03d}"
    
    print(f"\n\n========== Running optimization for COEFFICIENTS = {coeff1:.2f}, {coeff2:.2f} ==========\n")


    for seed in seed_list:
        print(f"\n\n========== Running optimization for SEED = {seed} ==========\n")


    # Constants
        xml_template = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/main xmls/modified_robot_planar.xml"
          
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Update filenames to include coefficient values
        best_results_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/planar/gear/best_gear_{coeff_str}_{date_str}_{seed}.csv"
        all_samples_file = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/result_csvs/planar/gear/all_gear_{coeff_str}_{date_str}_{seed}.csv"
        
        # Ensure directories exist
        os.makedirs(os.path.dirname(best_results_file), exist_ok=True)
        os.makedirs(os.path.dirname(all_samples_file), exist_ok=True)
        # original_bounds = np.array([
        #     [0.300, 0.500],  # Thigh length
        #     [0.300, 0.500],  # Calf length
        #     [4, 8.7], 
        #     [4, 8.7],
        #     [5, 200],        # Controller Param 1
        #     [0, 10], 
        #     [0, 50]     # Controller Param            # Controller Param 6
        # ])
        original_bounds = np.array([
        # Calf length
            [4, 8.7], 
            [4, 8.7],
            [0, 200],        # Controller Param 1
            [0, 10], 
            [0, 20] # Controller Param            # Controller Param 6
        ])

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

        # Helper Functions
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
            #theta1 = np.arctan2(z, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
            A = l1 + l2 * c2
            B = l2 * np.sin(theta2)
            theta1 = np.arctan2(A*x + B*z, x*B - A*z)

            #print(f"theta1: {theta1}, theta2: {theta2}, theta1n: {theta1}")
            return theta1, theta2
        def modify_link_length_and_angle(xml_file, output_file, 
                                        thigh_length, calf_length,
                                        thigh_gear_ratio, knee_gear_ratio,
                                        target_base_height=-0.5):
            
            # Compute joint angles
            l1, l2 = thigh_length, calf_length

            hip_peak_torque = thigh_gear_ratio*5.5
            knee_peak_torque = knee_gear_ratio*5.5*1.3636
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
            hip_actuator_mass = act_interp_func(thigh_gear_ratio)
            knee_actuator_mass = act_interp_func(knee_gear_ratio)
            torso_mass = hip_actuator_mass + knee_actuator_mass
            # torso_mass = 0.09
            thigh_link_mass = thigh_interp_func(thigh_length) 
            calf_link_mass = calf_interp_func(calf_length)
            # torso_mass = 0.0
            # thigh_link_mass = 0
            # calf_link_mass = 0
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

        
        def round_to_nearest(x, base=0.005):
            return round(base * round(x / base), 3)

        def get_cost(params):
            params = denormalize(params)
            thigh_length= 0.4
            calf_length= 0.4
            other_params = process_action(params[2:])
            hip_gear_ratio = params[0]
            knee_gear_ratio = params[1]
            controller_params = other_params
            

            unique_id = uuid.uuid4().hex[:8]
            #modified_xml = f"modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}_{int(hip_gear_ratio*10)}_{int(knee_gear_ratio*10)}_{unique_id}.xml"
            modified_xml = f"/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/Post_Humanoids/xmls/gear_xmls/{int(thigh_length * 100)}_{int(calf_length * 100)}_{int(hip_gear_ratio*100000)}_{int(knee_gear_ratio*100000)}_{unique_id}.xml"
            modify_link_length_and_angle(xml_template, modified_xml, thigh_length,calf_length, hip_gear_ratio, knee_gear_ratio)
                
            height_values = []
            energy_values = []
            run_entries = []
            distance_values = []
            x_values = []
            ik_value = -0.5
            hip_peak_torque = hip_gear_ratio * 5.5
            knee_peak_torque = knee_gear_ratio * 5.5 * 1.3636
            for i in range(1):
                (max_height_after_control, jump_count, total_energy, hip_energy, knee_energy,
                max_jump_height, max_jump_mech_energy, max_jump_joule_energy, max_jump_total_energy,
                max_jump_distance, max_jump_x_end, max_jump_duration,
                max_euclidean_height, max_euclidean_mech_energy, max_euclidean_joule_energy, max_euclidean_total_energy,
                max_euclidean_distance, max_euclidean_x_end, max_euclidean_duration, max_euclidean_value,
                jump_results, current_x_max) = rcp.run(modified_xml, controller_params, ik_value, hip_peak_torque, knee_peak_torque, thigh_length, calf_length)
                base_z_max = max_euclidean_height
                base_x_max = max_euclidean_distance
                base_total_max = max_euclidean_value
                mech_energy = max_euclidean_mech_energy
                joule_energy = max_euclidean_joule_energy
                te = max_euclidean_total_energy
                max_mech_energy = max_euclidean_mech_energy
                max_joule_energy = max_euclidean_joule_energy
                #te = max_jump_energy
                height_values.append(base_z_max)
                distance_values.append(base_total_max)
                x_values.append(base_x_max)
                energy_values.append(max_euclidean_total_energy)
            
            distance_counter = Counter(distance_values)
            mode_distance = max(distance_counter, key=lambda x: (distance_counter[x], x))
            mode_energy_values = [energy for distance, energy in zip(distance_values, energy_values) if distance == mode_distance]
            avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
            jump_results = [result for distance, result in zip(distance_values, jump_results) if distance == mode_distance]
            distance_cost = 50 * np.exp(-(mode_distance))
            
            # MODIFIED: Use the current coefficient set
            final_cost = (coeff1 * distance_cost + coeff2 * avg_energy)
            
            with open(all_samples_file, "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([thigh_length, calf_length, hip_gear_ratio, knee_gear_ratio, final_cost, mode_distance, base_z_max, base_x_max, avg_energy, max_joule_energy, max_mech_energy, jump_results, unique_id] + list(controller_params))
            return final_cost 

        # CMA-ES Optimization
        x0 = normalize(np.array([6.0,6.0,100,5,10]))
        sigma0 = 0.1
        opts = cma.CMAOptions()

        opts.set({
            'maxiter': 200, 'popsize': 8, 'seed': int(seed),
            'bounds': [np.zeros(5), np.ones(5)], 'verb_disp': 1000
        })

        es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

        with open(best_results_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Best index", "Best Cost"] + [f"ac{i+1}" for i in range(3)])


        with open(all_samples_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Thigh", "Calf", "Hip gear ratio", "Knee gear ratio", "Cost", "Max Distance", "Max Height","Max x", "Max Energy", "Max Joule", "Max Mech", "Jump Results", "unique id"] + [f"ac{i+1}" for i in range(3)])


        while not es.stop():
            solutions = es.ask()
            with multiprocessing.Pool(processes=8) as pool:
                costs = pool.map(get_cost, solutions)
            best_index = np.argmin(costs)
            best_params = denormalize(solutions[best_index])
            best_cost = costs[best_index]
            thigh,calf = 0.4,0.4
            hip_gear_ratio = best_params[0]
            knee_gear_ratio = best_params[1]
            controller_params = process_action(best_params[2:])
            
            with open(best_results_file, "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([thigh, calf, hip_gear_ratio, knee_gear_ratio, best_index, best_cost] + list(controller_params))
            es.tell(solutions, costs)
            


        print("Joint optimization complete.")
        print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        with open(best_results_file, "a", newline="") as file:
                writer = csv.writer(file)
                #subtract start time from end time
                end_time = datetime.now()
                writer.writerow(["Time taken (seconds)"])
                writer.writerow([end_time.strftime("%Y-%m-%d %H:%M:%S")])