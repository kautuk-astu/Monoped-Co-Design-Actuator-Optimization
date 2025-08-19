import pandas as pd
import matplotlib.pyplot as plt

# File paths
csv1 = 'thigh_03_05.csv'
csv2 = 'calf_03_05.csv'

# Read and convert mm to meters
df1 = pd.read_csv(csv1)
df2 = pd.read_csv(csv2)
axis_label_fontsize = 10
tick_fontsize = 11
title_fontsize = 14
legend_fontsize = 12

df1['length_m'] = df1['Link Length (mm)'] / 1000  # Assuming column is named 'length_mm'
df2['length_m'] = df2['Link Length (mm)'] / 1000

# Set up 2x2 grid, using only top two cells
fig, axs = plt.subplots(2, 2, figsize=(12, 6), height_ratios=[1, 0.001])
for label in axs.get_xticklabels() + axs.get_yticklabels():
    label.set_fontweight("bold")

# Set spine color
for spine in axs.spines.values():
    spine.set_color("black")

# Plot in (0, 0)
axs[0, 0].plot(df1['length_m'], df1['Calculated Mass (kg)'], color='blue', label='Thigh Mass')
axs[0, 0].set_title('Mass vs Thigh Length', fontsize=title_fontsize)
axs[0, 0].set_xlabel('Link Length (m)', fontsize=axis_label_fontsize)
axs[0, 0].set_ylabel('Mass (kg)', fontsize=axis_label_fontsize)
axs[0, 0].grid(True)
axs[0, 0].legend(fontsize=legend_fontsize)

# Plot in (0, 1)
axs[0, 1].plot(df2['length_m'], df2['Calculated Mass (kg)'], color='green', label='Shank Mass')
axs[0, 1].set_title('Mass vs Shank Length', fontsize=title_fontsize)
axs[0, 1].set_xlabel('Link Length (m)', fontsize=axis_label_fontsize)
axs[0, 1].set_ylabel('Mass (kg)', fontsize=axis_label_fontsize)
axs[0, 1].grid(True)
axs[0, 1].legend(fontsize=legend_fontsize)

# Turn off unused subplots in row 2
axs[1, 0].axis('off')
axs[1, 1].axis('off')

plt.tight_layout()
plt.subplots_adjust(hspace=0.1)
plt.savefig('mass_vs_length.png', dpi=300)
plt.show()
