import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# Load your CSV
df = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Final_humanoids_results/comb_mass_torque.csv')

# Sort by gear ratio to avoid interpolation issues
df = df.sort_values(by='gear_ratio')

# Extract gear ratio and mass
gear_ratios = df['gear_ratio'].values
masses = df['mass'].values

# Create a linear interpolation function without extrapolation
mass_interp_func = interp1d(
    gear_ratios,
    masses,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)

# Example usage
gear_ratio_query = 4.65
try:
    interpolated_mass = mass_interp_func(gear_ratio_query)
    print(f"Interpolated mass at gear ratio {gear_ratio_query}: {interpolated_mass}")
except ValueError as e:
    print(f"Error: {e}")
