import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

# Load data
df = pd.read_csv("final_results.csv")
df = df.dropna(subset=['thigh_length', 'calf_length', 'mode_height', 'avg_energy'])

# Compute total leg length
df['total_leg_length'] = df['thigh_length'] + df['calf_length']

# # Round energy to nearest integer (optional, for fewer legend entries)
df['energy_rounded']  = int(df['avg_energy'])

# Sort to make legend cleaner
df = df.sort_values('avg_energy')
df = df.sort_values('energy_rounded')

# Unique sizes
unique_energies = sorted(df['energy_rounded'].unique())

# Plot with custom sizes
plt.figure(figsize=(30, 7))
scatter = sns.scatterplot(
    data=df,
    x='total_leg_length',
    y='mode_height',
    size='avg_energy',
    hue='avg_energy',
    sizes=(50, 500),
    palette='viridis',
    alpha=0.8,
    legend='full'  # This keeps all sizes in the legend
)

# Title and axis labels
plt.title("Mode Height vs Total Leg Length (Point Size = Energy)")
plt.xlabel("Total Leg Length")
plt.ylabel("Mode Height")
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', title='Energy')

plt.tight_layout()
plt.show()
