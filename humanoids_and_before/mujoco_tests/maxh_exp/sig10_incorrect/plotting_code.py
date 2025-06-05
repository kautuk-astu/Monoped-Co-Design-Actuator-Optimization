import numpy as np
import matplotlib.pyplot as plt

# Load the .dat file (assuming space-separated values)
filename = "/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/sig10_incorrect/maxh_ini_sig10_stdxrecentbest.dat"  # Replace with your actual file

data = np.loadtxt(filename, skiprows=1) 
data = data[:-30, :]   # Skip the first row (header)

# Extract columns (assuming first column is X and second is Y)
#x = data[:, 0]  # First column
#y = data[:, 4]  # Second column

xs = data[:, 7]
ys = data[:, 4]

# Create a scatter plot
plt.figure(figsize=(10, 2))
#plt.scatter(x, y, color='b', marker='o', alpha=0.7)
#plt.plot(x, y, color='r', label='Bestever cost ')
plt.scatter(xs, ys, color='b', label=' cost through iterations')

# Labels and title
plt.xlabel("kpfall")
plt.ylabel("cost")
plt.title("cost vs kpfall")


#plt.legend()
plt.savefig('costvskpfall.png')

# Show the plot
plt.show()
