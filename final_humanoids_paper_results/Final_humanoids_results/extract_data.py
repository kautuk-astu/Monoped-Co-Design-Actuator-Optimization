import pandas as pd

# File paths
primary_csv ='/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/best_comb_065_035_2025-08-07_01-20-21.csv'        # CSV with 'Best Cost' column
secondary_csv = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/final_humanoids_paper_results/Final_humanoids_results/camera_ready/all_comb_065_035_2025-08-07_01-20-21.csv'    # CSV to match joint/gear values
output_txt = 'lowest_best_cost_info.txt'     # Output file

# Load the primary CSV
df1 = pd.read_csv(primary_csv)

# Check necessary columns
required_cols = ['Best Cost', 'Thigh', 'Calf', 'Hip gear ratio', 'Knee gear ratio']
missing_cols = [col for col in required_cols if col not in df1.columns]
if missing_cols:
    raise ValueError(f"Missing columns in primary CSV: {missing_cols}")

# Get the row with the lowest 'Best Cost'
min_row = df1.loc[df1['Best Cost'].idxmin()]

# Convert primary row to text
primary_output = ["Primary CSV - Entry with Lowest Best Cost:\n"]
primary_output += [f"{col}: {min_row[col]}" for col in df1.columns]
primary_text = "\n".join(primary_output)

# Load secondary CSV
df2 = pd.read_csv(secondary_csv)

# Filter based on matching Thigh, Calf, Hip gear ratio, and Knee gear ratio
match_criteria = {
    'Thigh': min_row['Thigh'],
    'Calf': min_row['Calf'],
    'Hip gear ratio': min_row['Hip gear ratio'],
    'Knee gear ratio': min_row['Knee gear ratio']
}

# Check required columns in secondary CSV
missing_secondary = [col for col in match_criteria if col not in df2.columns]
if missing_secondary:
    raise ValueError(f"Missing columns in secondary CSV: {missing_secondary}")

# Perform the match
matched_rows = df2
for col, val in match_criteria.items():
    matched_rows = matched_rows[matched_rows[col] == val]

# Convert matching rows to string
if matched_rows.empty:
    secondary_text = "\n\nNo matching entries found in secondary CSV for the given parameters."
else:
    secondary_output = ["\n\nMatching Entry/Entries from Secondary CSV:\n"]
    for idx, row in matched_rows.iterrows():
        secondary_output += [f"{col}: {row[col]}" for col in df2.columns]
        secondary_output.append("\n" + "-"*40 + "\n")
    secondary_text = "\n".join(secondary_output)

# Final output text
final_text = primary_text + secondary_text

# Print and write to file
print(final_text)
with open(output_txt, 'w') as f:
    f.write(final_text)

