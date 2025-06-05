import pandas as pd

# Load your full CSV
csv_full = pd.read_csv('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_03_05/final_links_with_mass_02_04.csv')

# Filter rows where thigh and calf are between 0.30 and 0.40 (inclusive)
subset = csv_full[
    (csv_full['thigh_length'] >= 0.30) & (csv_full['thigh_length'] <= 0.40) &
    (csv_full['calf_length'] >= 0.30) & (csv_full['calf_length'] <= 0.40)
]

# If you want to also clear ik_value
subset['ik_value'] = ''

# Save to a new CSV
subset.to_csv('mass_03_04.csv', index=False)
