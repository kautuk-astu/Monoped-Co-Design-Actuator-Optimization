import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Replace with your CSV file path
csv_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Final_humanoids_results/results/best_comb_065_035_2025-06-05_09-53-12.csv"

# Load the CSV (assuming it has a column like 'cost')
df = pd.read_csv(csv_file)

# Generate iteration numbers automatically
iterations = range(len(df))
costs = df["Best Cost"]

ln_costs = np.log(costs)
ln_best_ever_costs = np.log(costs.cummin())

# Compute best-ever (minimum so far) cost over iterations
best_ever_costs = costs.cummin()
#give me the iteration number of the best ever cost after rounding off cost to 6 decimal places
best_ever_costs = np.round(best_ever_costs, 6)
ln_best_ever_costs = np.log(best_ever_costs)
best_ever_iteration = np.argmin(np.round(costs,6))
print(f"Best ever cost: {best_ever_costs[best_ever_iteration]} at iteration {best_ever_iteration}")

# Plotting
plt.figure(figsize=(10, 6))

plt.scatter(iterations, ln_costs, label="ln(Cost)", color="blue", alpha=0.2)
plt.plot(iterations, ln_best_ever_costs, label="ln(Best-Ever Cost)", color="red", linewidth=2)

plt.xlabel("Iteration")
plt.ylabel("ln(Cost)")
plt.title("Log Cost per Iteration and Best-Ever Log Cost")
#plt.ylim(-1.1,-0.8)
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("cma_log_cost.png", dpi=300)
plt.show()
plt.figure(figsize=(10, 6))
plt.scatter(iterations, costs, label="Iteration Cost", color="blue", alpha=0.2)
plt.plot(iterations, best_ever_costs, label="Best-Ever Cost", color="red", linewidth=2)


plt.xlabel("Iteration")
plt.ylabel("Cost")
plt.title("Cost per Iteration and Best-Ever Cost")
#plt.ylim(0.335, 0.43)
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("cma_best_cost.png", dpi=300)
plt.show()
