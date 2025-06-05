import pandas as pd
import matplotlib.pyplot as plt

# Load CMA progress CSV
csv_file = 'cma_progress_575_500.csv'  # replace with your actual filename
df = pd.read_csv(csv_file)

# Extract relevant columns
iterations = df['Iteration']
best_costs = df['Best Cost']

# Compute best-ever cost up to each iteration
best_ever_costs = best_costs.cummin()

# Plotting
plt.figure(figsize=(10, 6))
plt.scatter(iterations, best_costs, color='blue', label='Best Cost (per iteration)', alpha=0.6)
plt.plot(iterations, best_ever_costs, color='red', label='Best-Ever Cost (cumulative)', linewidth=2)

plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('CMA-ES Optimization Progress')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('cma_progress_plot.png', dpi=300)  # Save the figure
plt.show()

