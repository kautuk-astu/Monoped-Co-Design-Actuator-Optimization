import numpy as np
import matplotlib.pyplot as plt

# Load the .dat file (assuming space-separated values)
#filename = "/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/thigh_45/45/45fit.dat"  # Replace with your actual file
filename = "/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/thigh_525/525/525fit.dat"
data = np.loadtxt(filename, skiprows=1) 
#data = data[:-4, :]   # Skip the first row (header)

# Extract columns (assuming first column is X and second is Y)
x = data[:, 0]  # First column
y = data[:, 4]  # Second column

xs = data[:, 0]
ys = data[:, 5]

# Create a scatter plot
plt.figure(figsize=(8, 5))
#plt.scatter(x, y, color='b', marker='o', alpha=0.7)
plt.plot(x, y, color='r', label='Bestever cost ')
plt.scatter(xs, ys, color='b', label=' cost through iterations')
plt.grid()

# Labels and title
plt.xlabel("iteration")
plt.ylabel("cost")
plt.title("Cost evolution during optimization")


plt.legend()
plt.savefig('cma_conv.png')

# Show the plot
plt.show()
