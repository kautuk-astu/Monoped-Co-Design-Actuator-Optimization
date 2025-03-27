import cma
import numpy as np
import mujoco as mj
import rcp as rcp
import csv
from collections import Counter

xml_path = 'modified_robot.xml'

csv_filename = "data.csv"
run_log_filename = "run_logs.csv"
np.random.seed(42)


# 🔹 Define original min-max bounds for each controller parameter
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

def get_reward(norm_action):     
    num_runs = 5
    height_values = []
    energy_values = []
    run_entries = []
    # Convert from normalized range [0,1] to actual parameter range
    action = denormalize(norm_action)
    action = np.round(action, 2)  # Round to 2 decimal places

    for run_id in range(num_runs):  
        base_height_store, height_reward, te, he, ke = rcp.run(xml_path, action)
        base_max = np.round(np.max(base_height_store), 1)
        te = int(round(te))

        height_values.append(base_max)
        energy_values.append(te)

        run_entries.append([run_id + 1, base_max, te, he, ke] + list(action))

    # Compute mode height & average energy
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0
    # Compute final cost
    height_cost = 100 * np.exp(-mode_height)
    final_cost = height_cost + (1/3) * avg_energy

    # Log runs
    with open(run_log_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(run_entries)

    # Log final cost
    with open(csv_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([mode_height, final_cost, avg_energy] + list(action))

    return final_cost

# CMA-ES Optimization
fun = get_reward

# Normalize initial guess to [0,1]
x0 = np.array([0.13, 0, 0.2, 0, 0, 0])

sigma0 = 0.1  # Use a small sigma to match normalized range
opts = cma.CMAOptions()
opts.set({'tolfun': 2})
opts.set({'tolx': 0.001})
opts.set({'maxiter': 20})    
opts.set({'verb_disp': 1000})
opts.set({'popsize': 9})
opts.set({'verb_filenameprefix': '40'}, force=True)
opts.set({'seed': 42})

# Set CMA-ES bounds to [0,1] for all parameters
opts.set({'bounds': [([0,0,0,0,0,0]), ([1,1,1,1,1,1])]})

# Initialize CSV Files
headers = ["mode_height", "final_cost", "avg_energy"] + [f"ac{i+1}" for i in range(len(x0))]

with open(csv_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(headers)

with open(run_log_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["run_id", "max_height", "rounded_te", "he", "ke"] + [f"ac{i+1}" for i in range(len(x0))])

# Run CMA-ES optimization with normalization
x, es = cma.fmin2(fun , x0, sigma0, opts)

