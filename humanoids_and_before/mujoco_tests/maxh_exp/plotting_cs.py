import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = "/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/height.csv"  # Change this to your CSV file path
df = pd.read_csv(file_path)

# Plot the data
plt.figure(figsize=(8, 5))
plt.plot(df['shank'], df['height'], marker='o', linestyle='-', color='b', label="Shank vs Jump Height")

# Customize the plot
plt.xlabel("Shank")
plt.ylabel("Jump Height")
plt.title("Shank vs Jump Height Plot")
plt.legend()
#plt.grid(True)
plt.savefig('shank_vs_max_height.png')

# Show the plot
plt.show()
