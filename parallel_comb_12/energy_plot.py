import pandas as pd
import matplotlib.pyplot as plt
from collections import Counter

# Load the final results
final_results_file = "final_results.csv"
df = pd.read_csv(final_results_file)

# Find the mode (most frequent) height
mode_height = Counter(df["mode_height"]).most_common(1)[0][0]

# Filter rows where mode height matches
filtered_df = df[df["mode_height"] == mode_height]

# Plot Energy Consumption for the Same Height
plt.figure(figsize=(8, 6))
scatter = plt.scatter(filtered_df["thigh_length"], filtered_df["calf_length"], c=filtered_df["avg_energy"], cmap="coolwarm", edgecolors="k", s=100)

# Add color bar
cbar = plt.colorbar(scatter)
cbar.set_label("Energy Consumption")

# Labels and title
plt.xlabel("Thigh Length")
plt.ylabel("Calf Length")
plt.title(f"Energy Consumption for Mode Height = {mode_height}")
plt.savefig("energy_plot.png",dpi=300)

# Show plot
plt.show()
