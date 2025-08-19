import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# Load your CSV
df = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Final_humanoids_results/thigh_03_05.csv')

# Sort by gear ratio to avoid interpolation issues
df = df.sort_values(by='Link Length (mm)')

# Extract gear ratio and mass
link_lengths = df['Link Length (mm)'].values
masses = df['Calculated Mass (kg)'].values

# Create a linear interpolation function without extrapolation
mass_interp_func = interp1d(
    link_lengths,
    masses,
    kind='linear',
    bounds_error=True  # This raises an error if input is out of bounds
)

# Example usage
link_lengths_query = 345.678
try:
    interpolated_mass = mass_interp_func(link_lengths_query)
    print(f"Interpolated mass at gear ratio {link_lengths_query}: {interpolated_mass}")
except ValueError as e:
    print(f"Error: {e}")