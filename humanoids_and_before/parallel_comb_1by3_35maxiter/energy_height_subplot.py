import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import numpy as np
# Load the final results file
final_results_file = "sorted_final_results.csv"
df = pd.read_csv(final_results_file)

# Get all unique mode heights (sorted)
unique_heights = sorted(df["mode_height"].unique())

# Set up subplots (adjust number of rows and columns as needed)
n_cols = 3
n_rows = (len(unique_heights) + n_cols - 1) // n_cols

fig, axs = plt.subplots(n_rows, n_cols, figsize=(6 * n_cols, 5 * n_rows))
axs = axs.flatten()

# Define the same colormap and norm for all subplots to keep it comparable
cmap = cm.coolwarm
norm = mcolors.Normalize(vmin=df["avg_energy"].min(), vmax=df["avg_energy"].max())

for i, height in enumerate(unique_heights):
    ax = axs[i]
    filtered_df = df[df["mode_height"] == height]

    # Map color from normalized energy
    colors = cmap(norm(filtered_df["avg_energy"]))

    scatter = ax.scatter(
        filtered_df["thigh_length"],
        filtered_df["calf_length"],
        c=colors,
        s=100,
        edgecolor='black'
    )

    ax.set_title(f"Mode Height = {height}")
    ax.set_xlabel("Thigh Length")
    ax.set_ylabel("Calf Length")

    # Custom color legend (not colorbar)
    for val in np.linspace(df["avg_energy"].min(), df["avg_energy"].max(), num=5):
        ax.scatter([], [], c=[cmap(norm(val))], label=f"{val:.0f}", s=100, edgecolor='black')

    ax.legend(title="Avg Energy", loc="upper right", frameon=True)

# Hide any unused subplots
for j in range(i + 1, len(axs)):
    fig.delaxes(axs[j])

plt.tight_layout()
plt.savefig("subplot_energy_modeheight_individual_legends.png", dpi=300)
plt.show()
