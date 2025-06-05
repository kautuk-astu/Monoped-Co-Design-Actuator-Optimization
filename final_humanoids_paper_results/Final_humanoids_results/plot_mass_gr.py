import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

#code to plot mass of actuator vs gear ratio from csv file
# Read the CSV file
df = pd.read_csv("/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_with_gear/sspg_interpolated.csv")
# Extract the columns
gear_ratio = df['gear_ratio']
mass = df['mass']
# Create a scatter plot
#plt.scatter(gear_ratio, mass, color='blue', label='Mass vs Gear Ratio')
#create a line plot
plt.plot(gear_ratio, mass, color='blue', alpha=0.5)
# Add labels and title
plt.xlabel('Gear Ratio')
plt.ylabel('Mass (kg)')
plt.title('Mass of Actuator vs Gear Ratio')
plt.show()