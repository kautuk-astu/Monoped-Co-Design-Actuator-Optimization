import pandas as pd
import os

# File paths
best_results_file = "best_results.csv"
final_results_file = "final_results.csv"

# Load the best results file
best_results_df = pd.read_csv(best_results_file)

# Initialize list to store results
final_results = []

# Loop through all link length combinations
for index, row in best_results_df.iterrows():
    thigh_length, calf_length = row["thigh_length"], row["calf_length"]
    best_cost = row["best_cost"]  # No rounding applied

    # Define corresponding mode height file
    mode_height_file = f"mode_height_data_{int(thigh_length * 1000)}_{int(calf_length * 1000)}.csv"

    if not os.path.exists(mode_height_file):
        print(f"Warning: File {mode_height_file} not found. Skipping.")
        continue  # Skip if file is missing

    # Load mode height data
    mode_df = pd.read_csv(mode_height_file)

    # Find row where best_cost appears in mode_data_file
    matched_row = mode_df.loc[mode_df["Final Cost"] == best_cost]

    if matched_row.empty:
        print(f"Warning: No matching cost found in {mode_height_file} for {best_cost}. Skipping.")
        continue

    # Extract mode height & avg energy (take first match if multiple)
    mode_height = matched_row.iloc[0]["Mode Height"]
    avg_energy = matched_row.iloc[0]["Avg Energy"]

    # Store the results
    final_results.append([thigh_length, calf_length, mode_height, avg_energy])

# Convert to DataFrame and save to CSV
final_df = pd.DataFrame(final_results, columns=["thigh_length", "calf_length", "mode_height", "avg_energy"])
final_df.to_csv(final_results_file, index=False)

print(f"Final results saved to {final_results_file}")
