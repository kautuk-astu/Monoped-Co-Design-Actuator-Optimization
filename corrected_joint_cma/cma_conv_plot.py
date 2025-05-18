import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Replace with your CSV file path
csv_file = "/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/best_joint_results_065_035_2025-05-15_15-25-34.csv"

# Load the CSV (assuming it has a column like 'cost')
df = pd.read_csv(csv_file)

# Generate iteration numbers automatically
iterations = range(len(df))
costs = df["Best Cost"]

ln_costs = np.log(costs)
ln_best_ever_costs = np.log(costs.cummin())

# Compute best-ever (minimum so far) cost over iterations
best_ever_costs = costs.cummin()

# Plotting
plt.figure(figsize=(10, 6))

plt.scatter(iterations, ln_costs, label="ln(Cost)", color="blue", alpha=0.6)
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
plt.scatter(iterations, costs, label="Iteration Cost", color="blue", alpha=0.6)
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
