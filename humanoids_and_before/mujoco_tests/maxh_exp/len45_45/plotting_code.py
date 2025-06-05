import numpy as np
import matplotlib.pyplot as plt

# Load the .dat file (assuming space-separated values)
filename = "/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/len45_45/maxh_4545_maxiter20xrecentbest.dat"  # Replace with your actual file
data = np.loadtxt(filename, skiprows=1)  # Skip the first row (header)

# Extract columns (assuming first column is X and second is Y)
x = data[:, 5]  # First column
y = data[:, 4]  # Second column

# Create a scatter plot
plt.figure(figsize=(10, 6))
plt.scatter(x, y, color='b', marker='o', alpha=0.7)
#plt.plot(x, y, color='r', label='Bestever cost value')

# Labels and title
plt.xlabel("kprise")
plt.ylabel("cost")
plt.title("cost vs kprise")

#plt.legend()
plt.savefig('maxh_4545_kpvsrise.png')

# Show the plot
plt.show()
