import pandas as pd

# Load the CSV file
csv_file = "best_results.csv"  # Replace with your actual file name
df = pd.read_csv(csv_file)

# Sort by 'thigh_length' first, then by 'calf_length'
df_sorted = df.sort_values(by=['thigh_length', 'calf_length'], ascending=[True, True])

# Save the sorted CSV
sorted_csv_file = "sorted_" + csv_file  # Creates a new sorted file
df_sorted.to_csv(sorted_csv_file, index=False)

print(f"Sorted CSV saved as: {sorted_csv_file}")
