import csv
import cma
import numpy as np
import xml.etree.ElementTree as ET
import rcp  
import csv
from collections import Counter
import multiprocessing
thigh_lengths = np.linspace(0.400, 0.600, 9)  #both inclusive
calf_lengths = np.linspace(0.400, 0.600, 9)
print(thigh_lengths)
print(calf_lengths)
# remaining_combinations = [
#     (0.55,0.5),
#     (0.525,0.45),
#     (0.525,0.5),
#     (0.525,0.525),
#     (0.525,0.55),
#     (0.525,0.575),
#     (0.5,0.45),
#     (0.5,0.525),
#     (0.5,0.575)
# ]
# remaining_combinations = list(remaining_combinations)

xml_template = "template_robot.xml"
best_results_file = "best_results.csv"
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

with open(best_results_file, "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["thigh_length", "calf_length", "best_cost"] + [f"ac{i+1}" for i in range(6)])
np.random.seed(42)
original_bounds = np.array([
    [5, 200],    # Param 1
    [0, 1],      # Param 2
    [0, 5],      # Param 3
    [1, 20],     # Param 4
    [1, 10],     # Param 5
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

# XML Modification Function
def modify_link_length(xml_file, link_name, new_length, axis='z', output_file="modified_robot.xml"):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]  

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
    print(f"Modified XML saved: {output_file}")

def process_action(action):
    """Applies selective rounding to an action before passing it to get_reward."""
    action = action.copy()  # Ensure the original action is not modified
    # Convert selected parameters to integers
    #action[0] = round(action[0])  # First parameter
    # action[2] = round(action[2])  # Third parameter
    # action[3] = round(action[3])  # Fourth parameter
    # action[4] = round(action[4])  # Fifth parameter

    # Round the rest to one decimal place
    for i in range(len(action)):  # Skip integer indices
            action[i] = np.round(action[i], 1)

    # Convert integer values explicitly to int type
    return action



# Function to compute cost (Runs controller 5 times per action)
def get_reward(action, log_file, mode_data_file,thigh_length,calf_length,modified_xml):     
    num_runs = 2
    height_values = []
    energy_values = []
    run_entries = []
    mass_ik_dict = load_mass_ik_values("mass_ik_values.csv")
    mass, ik_value = mass_ik_dict[(round(thigh_length, 3), round(calf_length, 3))]
    action = denormalize(action)
    #action = np.round(action, 2)
    action = process_action(action)  # Process the action for selective rounding
    for run_id in range(num_runs):  
        #base_height_store, height_reward, te, he, ke,max_height_after_control = rcp.run(modified_xml, action,mass,ik_value)
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, max_jump_height, max_jump_energy = rcp.run(modified_xml, action, mass, ik_value)
        #base_max = np.round(np.max(base_height_store), 1)
        base_max = np.round(max_height_after_control,1)
        te = np.round((max_jump_energy),1)
        height_values.append(base_max)
        energy_values.append(te)
        run_entries.append([run_id + 1, base_max, te] + list(action))

    # Compute mode height & average energy
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
    # Compute final cost
    height_cost = 10* np.exp(-mode_height)
    final_cost = np.round(height_cost,1)
    # print(f"Running for Thigh Length: {thigh_length}, Calf Length: {calf_length}")
    # print(f"Using XML: {modified_xml}, Mass: {mass}, IK Value: {ik_value}")

    # Log individual runs in separate file
    with open(log_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(run_entries)

    # Log mode height, avg energy & cost
    with open(mode_data_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([mode_height, avg_energy, final_cost] + list(action))

    return final_cost

# Function to optimize controller parameters for a given design
def optimize_for_design(thigh_length, calf_length):
    modified_xml = f"modified_robot_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.xml"
    # Update thigh length first
    modify_link_length(xml_template, "link1", thigh_length, output_file=modified_xml)
    # Now update calf length using the updated file
    modify_link_length(modified_xml, "link2", calf_length, output_file=modified_xml)
    log_file = f"run_logs_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"
    mode_data_file = f"mode_height_data_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"
    progress_file = f"cma_progress_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"
    # Initialize CMA-ES
    x0 = np.array([0.13, 0, 0.2, 0, 0, 0])
    sigma0 = 0.2
    opts = cma.CMAOptions()
    opts.set({
        'tolfun': 0.05, 'tolx': 0.001, 'maxiter': 20, 'popsize': 9, 'seed': 42,
        'bounds': [([0,0,0,0,0,0]), ([1,1,1,1,1,1])]
    })
    opts.set({'verb_disp': 1000})
    opts.set({'verb_log': 1})
    #print(f"cma_{int(thigh_length * 1000)}_{int(calf_length * 1000)}")
    opts.set({'verb_filenameprefix': f"cma_{int(thigh_length * 1000)}_{int(calf_length * 1000)}"}, force=True)

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)
    print("Optimizing for Thigh:", thigh_length, "Calf:", calf_length)

    # Add headers for logs
    with open(progress_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Iteration", "Best Cost"] + [f"ac{i+1}" for i in range(len(x0))])

    with open(mode_data_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Mode Height", "Avg Energy", "Final Cost"] + [f"ac{i+1}" for i in range(len(x0))])
    total_best_cost = float("inf")  # Initialize to a very high value
    total_best_params = None
    while not es.stop():
        solutions = es.ask()
        #rounded_solutions = [np.round(sol).astype(int) for sol in solutions]  # Convert to integers
        
        rounded_solutions = [process_action(sol) for sol in solutions]
        rewards=[]
        #rewards = [get_reward(action, log_file, mode_data_file, thigh_length, calf_length, modified_xml) for action in rounded_solutions]
        for action in solutions:
            cost = get_reward(action, log_file, mode_data_file,thigh_length,calf_length,modified_xml)
            rewards.append(cost)
        es.tell(rounded_solutions, rewards)
        print("Iteration", es.countiter)
        best_index = np.argmin(rewards)  
        best_cost = rewards[best_index]
        best_params = rounded_solutions[best_index]         
        with open(progress_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([es.countiter, best_cost] + list(best_params))
    # Store the best cost and parameters
        min_index = np.argmin(rewards)  # Index of the best reward in this iteration
        if rewards[min_index] < total_best_cost:
            total_best_cost = rewards[min_index]
            total_best_params = rounded_solutions[min_index]
        print("Iteration", es.countiter, "Best Cost =", best_cost)

    #best_params = es.result.xbest # Process the best parameters for selective rounding
    #best_cost = get_reward(best_params, log_file, mode_data_file, thigh_length, calf_length, modified_xml)

    with open(best_results_file, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([thigh_length, calf_length, total_best_cost] + list(total_best_params))
    return total_best_params, total_best_cost

# Run Grid Search
def parallel_grid_search():
    
    link_combinations = [(t, c) for t in thigh_lengths for c in calf_lengths]
    #link_combinations = remaining_combinations
    # Use multiprocessing to run optimizations in parallel
    with multiprocessing.Pool(processes=8) as pool:  # Adjust based on available cores
        results = pool.starmap(optimize_for_design, link_combinations)

    print("Parallel Grid Search and Optimization Completed")

# Run parallel grid search
if __name__ == "__main__":
    parallel_grid_search()

