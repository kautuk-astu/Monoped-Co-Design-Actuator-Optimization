import re
import pandas as pd
import numpy as np

# Read the file
with open("/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/lowest_best_cost_info_gear.txt") as f:
    text = f.read()

# Parameters we want to extract
params = [
    "Thigh", "Calf", "Hip gear ratio", "Knee gear ratio",
    "Max Height", "Max Energy", "ac1", "ac2", "ac3"
]

# Dictionary to hold lists of values
data = {p: [] for p in params}

# Regex patterns for each param
patterns = {p: re.compile(rf"{re.escape(p)}:\s+([-\d\.Ee]+)") for p in params}

# Go through each parameter and collect all matches
max_len = 0
for p in params:
    matches = patterns[p].findall(text)
    floats = [float(m) for m in matches]
    data[p] = floats
    max_len = max(max_len, len(floats))

# Pad lists with NaN so they all match length
for p in params:
    if len(data[p]) < max_len:
        data[p] += [np.nan] * (max_len - len(data[p]))

# Convert to DataFrame
df = pd.DataFrame(data)

# Compute mean for each parameter (ignoring NaN)
mean_values = df.mean()

print("Mean values across all seeds:")
print(mean_values)

# Optional: save to CSV
mean_values.to_csv("mean_values.csv", header=["Mean Value"])

