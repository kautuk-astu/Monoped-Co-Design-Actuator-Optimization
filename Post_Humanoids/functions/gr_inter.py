import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

def interpolate_gear_mass_torque(input_csv, output_csv, ratio_range=(4.0, 8.7), step=0.1, torque_multiplier=5.5):
    # Load gear ratio vs mass data
    data = pd.read_csv(input_csv)
    gear_ratios = data['gearRatio'].values
    masses = data['mass'].values

    # Linear interpolation for mass
    interp_fn = interp1d(gear_ratios, masses, kind='linear', fill_value='extrapolate')

    # Generate gear ratios from 4 to 8 with 0.1 spacing
    new_ratios = np.arange(ratio_range[0], ratio_range[1] + step, step)
    new_ratios = np.round(new_ratios, 1)
    new_masses = interp_fn(new_ratios)  
    # Calculate peak torque as gear_ratio × multiplier
    peak_torques = new_ratios * torque_multiplier

    # Save to CSV
    output_df = pd.DataFrame({
        'gear_ratio': new_ratios,
        'mass': new_masses,
        'peak_torque': peak_torques
    })
    output_df.to_csv(output_csv, index=False)
    print(f"Interpolated data with torque saved to: {output_csv}")

# Example usage
interpolate_gear_mass_torque("sspg_mass.csv", "sspg_interpolated_new.csv")
