import numpy as np
import matplotlib.pyplot as plt

# Load the .dat file (assuming space-separated values)
filename = "/home/stochlab/Aastha_pybullet/test_pb/indi_maxh_exp/kp_exp/maxh_std_kp_maxiter20_testfit.dat"  # Replace with your actual file
data = np.loadtxt(filename, skiprows=1)  # Skip the first row (header)

# Extract columns (assuming first column is X and second is Y)
x = data[:, 0]  # First column
y = data[:, 4]  # Second column

# Create a scatter plot
plt.figure(figsize=(10, 6))
#plt.scatter(x, y, color='b', marker='o', alpha=0.7)
plt.plot(x, y, color='r', label='Bestever cost value')

# Labels and title
plt.xlabel("iterations")
plt.ylabel("bestever cost value")
plt.title("CMA-ES Optimization")

plt.legend()
plt.savefig('maxh_std_kp_maxiter20_testfit.png')

# Show the plot
plt.show()
