import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Load and clean data
df = pd.read_csv("final_results.csv")
df = df.dropna(subset=['thigh_length', 'calf_length', 'mode_height', 'avg_energy'])

# Compute total leg length
df['total_leg_length'] = df['thigh_length'] + df['calf_length']

# Plot
plt.figure(figsize=(10, 7))
scatter = sns.scatterplot(
    data=df,
    x='total_leg_length',
    y='mode_height',
    size='avg_energy',
    hue='avg_energy',
    palette='viridis',
    sizes=(50, 500),
    alpha=0.8,
    legend='brief'
)

# Labeling
plt.title("2D Scatter: Mode Height vs Total Leg Length (Size = Energy)")
plt.xlabel("Total Leg Length")
plt.ylabel("Mode Height")
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', title='Energy')

plt.tight_layout()
plt.show()
