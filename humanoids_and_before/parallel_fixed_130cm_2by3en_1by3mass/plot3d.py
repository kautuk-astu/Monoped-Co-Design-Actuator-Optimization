import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load CSV file
csv_filename = "sorted_best_results.csv"  # Update this if your filename is different
df = pd.read_csv(csv_filename)

# Extract values
thigh_lengths = df["thigh_length"]
calf_lengths = df["calf_length"]
best_costs = np.round(df["best_cost"], 1)  # Round best cost to 1 decimal

# Create 3D Figure
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot data
ax.scatter(thigh_lengths, calf_lengths, best_costs, c=best_costs, cmap="viridis", marker='o')

# Labels and Title
ax.set_xlabel("Thigh Length")
ax.set_ylabel("Calf Length")
ax.set_zlabel("Best Cost")
ax.set_title("3D Plot of Best Cost vs Thigh & Calf Length")
plt.savefig("best_cost_3d_plot.png", dpi=300)


# Show Plot
plt.show()
