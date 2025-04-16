import pandas as pd
import matplotlib.pyplot as plt

# Load the data
csv_filename = "sorted_final_results.csv"
df = pd.read_csv(csv_filename)

# All 9 thigh lengths grouped in sets of 3
grouped_thighs = [
    [0.4, 0.425, 0.45],
    [0.475, 0.5, 0.525],
    [0.55, 0.575, 0.6]
]

# Set up subplots
fig, axs = plt.subplots(3, 1, figsize=(12, 6), sharey=True)
fig.suptitle("Height vs. Calf Length (Grouped by Thigh Lengths)", fontsize=16)

# Loop through each thigh group and create subplot
for i, thigh_group in enumerate(grouped_thighs):
    ax = axs[i]
    for thigh in thigh_group:
        subset = df[df["thigh_length"] == thigh]
        if subset.empty:
            print(f"No data for thigh length {thigh}")
            continue
        ax.plot(subset["calf_length"], subset["mode_height"], marker='o', label=f"Thigh {thigh}")
    
    ax.set_title(f"Group {i+1}: Thighs {', '.join(str(t) for t in thigh_group)}")
    ax.set_xlabel("Calf Length")
    ax.grid(True)
    ax.legend()

axs[0].set_ylabel("Mode Height")

plt.tight_layout(rect=[0, 0, 1, 0.95])  # leave space for suptitle
plt.savefig("height_vs_calf_grouped_by_thighs.png")
plt.show()
