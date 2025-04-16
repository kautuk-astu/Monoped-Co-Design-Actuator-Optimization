import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV
df = pd.read_csv("sorted_final_results.csv")  # Update if filename is different

# Ensure relevant columns exist
required_columns = ['thigh_length', 'calf_length', 'mode_height']
assert all(col in df.columns for col in required_columns), "Required columns missing in the CSV"

# Compute total leg length
df['total_leg_length'] = df['thigh_length'] + df['calf_length']

# Optional: drop rows with NaN in mode_height if any
df = df.dropna(subset=['mode_height'])

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(df['total_leg_length'], df['mode_height'], color='teal', alpha=0.7)
#plt.plot(df['total_leg_length'], df['mode_height'], linestyle='--', alpha=0.4)

plt.title("Mode Height vs Total Leg Length (Thigh + Calf)")
plt.xlabel("Total Leg Length")
plt.ylabel("Mode Height")
plt.grid(True)
plt.tight_layout()
plt.savefig("total_leg_length_vs_mode_height.png", dpi=300)
plt.show()

#make 3d plot with total leg length vs height and energy 
# dont comment it out - give code
