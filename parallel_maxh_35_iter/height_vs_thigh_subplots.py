import pandas as pd
import matplotlib.pyplot as plt

# Load the data
csv_filename = "sorted_final_results.csv"
df = pd.read_csv(csv_filename)

# All 9 thigh lengths grouped in sets of 3
grouped_calfs = [
    [0.4, 0.425, 0.45],
    [0.475, 0.5, 0.525],
    [0.55, 0.575, 0.6]
]

# Set up subplots
fig, axs = plt.subplots(3, 1, figsize=(12, 6), sharey=True)
fig.suptitle("Height vs. Thigh Length (Grouped by Calf Lengths)", fontsize=16)

# Loop through each thigh group and create subplot
for i, calf_group in enumerate(grouped_calfs):
    ax = axs[i]
    for calf in calf_group:
        subset = df[df["calf_length"] == calf]
        if subset.empty:
            print(f"No data for thigh length {calf}")
            continue
        ax.plot(subset["thigh_length"], subset["mode_height"], marker='o', label=f"Calf {calf}")
    
    ax.set_title(f"Group {i+1}: Calfs {', '.join(str(t) for t in calf_group)}")
    ax.set_xlabel("Thigh Length")
    ax.grid(True)
    ax.legend()

axs[0].set_ylabel("Mode Height")

plt.tight_layout(rect=[0, 0, 1, 0.95])  # leave space for suptitle
plt.savefig("height_vs_thigh_grouped_by_calfs.png")
plt.show()
