import pandas as pd

# Load all four CSVs
thigh_02_04 = pd.read_csv('thigh_data_02_04.csv')
thigh_04_06 = pd.read_csv('thigh_data_04_06.csv')
calf_02_04 = pd.read_csv('shank_data_02_04.csv')
calf_04_06 = pd.read_csv('shank_data_04_06.csv')

# Combine thigh files
thigh_combined = pd.concat([thigh_02_04, thigh_04_06], ignore_index=True)
# Combine calf files
calf_combined = pd.concat([calf_02_04, calf_04_06], ignore_index=True)

# Make sure the 'length' column exists and is float
thigh_combined['Link Length (mm)'] = thigh_combined['Link Length (mm)'].astype(float)
calf_combined['Link Length (mm)'] = calf_combined['Link Length (mm)'].astype(float)

# Filter for length between 0.30 and 0.50 meters
thigh_filtered = thigh_combined[
    (thigh_combined['Link Length (mm)'] >= 300) & (thigh_combined['Link Length (mm)'] <= 500)
]
calf_filtered = calf_combined[
    (calf_combined['Link Length (mm)'] >= 300) & (calf_combined['Link Length (mm)'] <= 500)
]

# Drop 'ik_value' column if present
if 'ik_value' in thigh_filtered.columns:
    thigh_filtered = thigh_filtered.drop(columns=['ik_value'])
if 'ik_value' in calf_filtered.columns:
    calf_filtered = calf_filtered.drop(columns=['ik_value'])

# Save new filtered CSVs
thigh_filtered.to_csv('thigh_03_05.csv', index=False)
calf_filtered.to_csv('calf_03_05.csv', index=False)

print("Successfully created thigh_03_05.csv and calf_03_05.csv")
