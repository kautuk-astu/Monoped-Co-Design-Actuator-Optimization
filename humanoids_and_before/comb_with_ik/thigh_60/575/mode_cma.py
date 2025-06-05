import cma
import numpy as np
import mujoco as mj
import rcp as rcp
import csv
from collections import Counter

xml_path = 'modified_robot.xml'

csv_filename = "data.csv"
run_log_filename = "run_logs.csv"

def round_value(value, decimal_places=1):
    """Round a value to the given number of decimal places."""
    return round(value, decimal_places)

def get_reward(action):     
    num_runs = 5
    height_values = []
    energy_values = []
    run_entries = []

    for run_id in range(num_runs):
        base_height_store, height_reward, te, he, ke = rcp.run(xml_path, action)

        # Round height to 1 decimal place
        base_max = round_value(np.max(base_height_store), 1)
        # Round total energy to nearest integer
        te = round(te)

        height_values.append(base_max)
        energy_values.append(te)

        # Store each run’s details
        run_entries.append([run_id + 1, base_max, te, he, ke] + list(action))

    # Find the mode of the rounded heights
    height_counter = Counter(height_values)
    mode_height = max(height_counter, key=lambda x: (height_counter[x], x))  # Pick higher height in case of tie

    # Compute the average energy for this mode height
    mode_energy_values = [energy for height, energy in zip(height_values, energy_values) if height == mode_height]
    avg_energy = np.mean(mode_energy_values) if mode_energy_values else 0  # Safety check

    # Compute final cost using mode height and average energy
    height_cost = 100 * np.exp(-mode_height)
    final_cost = height_cost + (1/3) * avg_energy

    # Log all runs in `run_logs.csv`
    with open(run_log_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(run_entries)

    # Log final selected cost in `data.csv`
    with open(csv_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([mode_height, final_cost, avg_energy] + list(action))
    
    return final_cost

# CMA-ES Optimization
fun = get_reward
x0 = np.array([30,0,1,1,1,0])    
sigma0 = 5
opts = cma.CMAOptions()
opts.set({'tolfun': 2})
opts.set({'tolx': 1})
opts.set({'maxiter': 25})    
opts.set({'verb_disp': 1000})
opts.set({'verb_filenameprefix': '575'}, force=True)
opts.set({'bounds': [([5,0,0,1,1,0]), ([200,1,5,20,10,1])]})

# Initialize CSV Files
with open(csv_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["mode_height", "final_cost", "avg_energy"] + [f"ac{i+1}" for i in range(len(x0))])

with open(run_log_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["run_id", "max_height", "rounded_te", "he", "ke"] + [f"ac{i+1}" for i in range(len(x0))])

# Run CMA-ES optimization
x, es = cma.fmin2(fun, x0, sigma0, opts)
