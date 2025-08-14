import pandas as pd
import numpy as np

# File paths
primary_csv = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/best_ll_06_04_2025-08-13_21-10-17_90.0.csv'
secondary_csv = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/all_ll_06_04_2025-08-13_21-10-17_90.0.csv'
output_txt = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/lowest_best_cost_info_ll.txt'

# Load primary CSV
df1 = pd.read_csv(primary_csv)

# Ensure required columns
required_cols = ['Best Cost', 'Thigh', 'Calf', 'Hip gear ratio', 'Knee gear ratio']
missing_cols = [col for col in required_cols if col not in df1.columns]
if missing_cols:
    raise ValueError(f"Primary CSV missing columns: {missing_cols}")

for col in required_cols:
    df1[col] = pd.to_numeric(df1[col], errors='coerce')

# Get row with lowest Best Cost
min_row = df1.loc[df1['Best Cost'].idxmin()]
best_cost_value = min_row['Best Cost']

# Convert primary row to text
primary_output = ["Primary CSV - Entry with Lowest Best Cost:\n"]
primary_output += [f"{col}: {min_row[col]}" for col in df1.columns]
primary_text = "\n".join(primary_output)

# Load secondary CSV
df2 = pd.read_csv(secondary_csv)

# Ensure required columns exist in secondary CSV
# missing_secondary = [col for col in required_cols if col not in df2.columns]
# if missing_secondary:
#     raise ValueError(f"Secondary CSV missing columns: {missing_secondary}")

if 'Cost' in df2.columns and 'Best Cost' not in df2.columns:
    df2 = df2.rename(columns={'mass': 'Best Cost'})
# Match: First by Best Cost (float tolerance)

required_cols_secondary = ['Best Cost', 'Thigh', 'Calf', 'Hip gear ratio', 'Knee gear ratio']
missing_secondary = [col for col in required_cols_secondary if col not in df2.columns]
if missing_secondary:
    raise ValueError(f"Secondary CSV missing columns: {missing_secondary}")

for col in required_cols_secondary:
    df2[col] = pd.to_numeric(df2[col], errors='coerce')
tolerance = 1e-4
filtered_rows = df2[np.isclose(df2['Best Cost'], best_cost_value, atol=tolerance)]

# Match: Then by Thigh, Calf, Hip gear ratio, Knee gear ratio (also with tolerance)
for col in ['Thigh', 'Calf', 'Hip gear ratio', 'Knee gear ratio']:
    filtered_rows = filtered_rows[np.isclose(filtered_rows[col], min_row[col], atol=tolerance)]

print(f"Step 2: Found {len(filtered_rows)} rows after matching parameters.")
if not filtered_rows.empty:
    filtered_rows = filtered_rows.iloc[[0]]

# Convert matching rows to text
if filtered_rows.empty:
    secondary_text = (
        f"\n\nNo matching entries found in secondary CSV for:\n"
        f"Best Cost = {best_cost_value}, Thigh = {min_row['Thigh']}, "
        f"Calf = {min_row['Calf']}, Hip gear ratio = {min_row['Hip gear ratio']}, "
        f"Knee gear ratio = {min_row['Knee gear ratio']}"
    )
else:
    secondary_output = ["\n\nMatching Entry/Entries from Secondary CSV:\n"]
    for idx, row in filtered_rows.iterrows():
        secondary_output += [f"{col}: {row[col]}" for col in df2.columns]
        secondary_output.append("\n" + "-"*40 + "\n")
    secondary_text = "\n".join(secondary_output)

# Final output
final_text = primary_text + secondary_text

# Print and save
print(final_text)
with open(output_txt, 'a') as f:
    f.write(final_text)

