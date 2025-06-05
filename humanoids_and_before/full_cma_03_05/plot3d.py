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
#add a color bar
# Create a scatter plot with color mapping
sc = ax.scatter(thigh_lengths, calf_lengths, best_costs, c=best_costs, cmap="viridis", marker='o')
# Add a color bar to the plot
cbar = plt.colorbar(sc)
cbar.set_label('Best Cost')  # Label for the color bar
# Set color bar ticks
cbar.set_ticks(np.arange(min(best_costs), max(best_costs) + 0.1, 0.1))  # Adjust ticks as needed
# Set color bar tick labels
cbar.set_ticklabels(np.arange(min(best_costs), max(best_costs) + 0.1, 0.1))  # Adjust labels as needed
# Set color bar limits
#cbar.set_clim(min(best_costs), max(best_costs))  # Adjust limits as needed
# Set color bar format
cbar.ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: '{:.1f}'.format(x)))  # Format color bar ticks
# Set color bar ticks
cbar.ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))  # Set color bar ticks to be integers
# Set color bar ticks
cbar.ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=10))  # Set color bar ticks to be integers

# Plot data
#ax.scatter(thigh_lengths, calf_lengths, best_costs, c=best_costs, cmap="viridis", marker='o')

# Labels and Title
ax.set_xlabel("Thigh Length")
ax.set_ylabel("Calf Length")
ax.set_zlabel("Best Cost")
ax.set_title("3D Plot of Best Cost vs Thigh & Calf Length")
plt.savefig("best_cost_3d_plot.png", dpi=300)


# Show Plot
plt.show()
