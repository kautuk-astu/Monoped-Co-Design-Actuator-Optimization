import csv
import numpy as np

# Define the range
lengths = np.round(np.arange(0.2, 0.41, 0.01), 2)  # 0.2 to 0.4 inclusive

# Create CSV
with open('lengths_grid.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['thigh_length', 'calf_length', 'mass', 'ik_value'])  # headers
    
    for thigh in lengths:
        for calf in lengths:
            writer.writerow([thigh, calf, '', ''])  # empty mass and ik_value
