#give code for plotting actuator gear ratio vs mass from the csv file
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import matplotlib.ticker as ticker
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# File paths
csv_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/comb_mass_torque.csv"
csv_2 = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/isspg_interpolated.csv"
csv_3 = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/sspg_interpolated_new.csv"

# Font size options
axis_label_fontsize = 22
tick_fontsize = 14
title_fontsize = 22
legend_fontsize = 14

# Read data
df1 = pd.read_csv(csv_file)
df2 = pd.read_csv(csv_2)
df3 = pd.read_csv(csv_3)

# Extract columns
x1, y1 = df1["gear_ratio"], df1["mass"]
x2, y2 = df2["gear_ratio"], df2["mass"]
x3, y3 = df3["gear_ratio"], df3["mass"]

# Determine common x and y limits
x_min = min(x1.min(), x2.min(), x3.min())
x_max = max(x1.max(), x2.max(), x3.max())
y_min = min(y1.min(), y2.min(), y3.min())
y_max = max(y1.max(), y2.max(), y3.max())

x_lim = (x_min - 0.5, x_max + 0.5)
y_lim = (y_min - 0.05, y_max + 0.05)

# Create figure with GridSpec layout
fig = plt.figure(figsize=(14, 10))
gs = GridSpec(2, 2, figure=fig)

# Subplot 1: ISSPG
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(x2, y2, marker='x', linestyle='-', color='purple', label='ISSPG Mass')
ax1.axvline(x=6, color='red', linestyle='--', label='Gear Ratio = 6')
ax1.set_xlim(*x_lim)
ax1.set_ylim(*y_lim)
ax1.set_title('ISSPG Interpolated', fontsize=title_fontsize)
ax1.set_ylabel('Mass(kg)', fontsize=axis_label_fontsize)
ax1.tick_params(axis='both', labelsize=tick_fontsize)
ax1.legend(fontsize=legend_fontsize)
ax1.grid()

# Subplot 2: ESSPG (renamed from SSPG)
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(x3, y3, marker='s', linestyle='-', color='green', label='ESSPG Mass')
ax2.axvline(x=6, color='red', linestyle='--', label='Gear Ratio = 6')
ax2.set_xlim(*x_lim)
ax2.set_ylim(*y_lim)
ax2.set_title('ESSPG Interpolated', fontsize=title_fontsize)
ax2.tick_params(axis='both', labelsize=tick_fontsize)
ax2.legend(fontsize=legend_fontsize)
ax2.grid()

# Subplot 3: Comb Mass Torque (bottom row)
ax3 = fig.add_subplot(gs[1, :])
ax3.plot(x1, y1, marker='o', linestyle='-', color='blue', label='Actuator Mass')
ax3.axvline(x=6, color='red', linestyle='--', label='Gear Ratio = 6')
ax3.fill_betweenx(y=[y_lim[0], y_lim[1]], x1=6, x2=x_lim[1], color='orange', alpha=0.3)
ax3.fill_betweenx(y=[y_lim[0], y_lim[1]], x1=x_lim[0], x2=6, color='green', alpha=0.3)
ax3.set_xlim(*x_lim)
ax3.set_ylim(*y_lim)
ax3.set_xlabel('Actuator Gear Ratio', fontsize=axis_label_fontsize)
ax3.set_ylabel('Mass(kg)', fontsize=axis_label_fontsize)
ax3.set_title('Optimized Mass', fontsize=title_fontsize)
ax3.tick_params(axis='both', labelsize=tick_fontsize)
ax3.legend(fontsize=legend_fontsize)
ax3.grid()

# Final layout
plt.tight_layout()
plt.savefig('gear_ratio_mass_2x2_comparison.png', dpi=300)
plt.show()


