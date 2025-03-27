import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = "/home/stochlab/Aastha_Coopt_Monoped/mujoco_tests/setup_for_opt/data.csv"  # Change this to your CSV file path
df = pd.read_csv(file_path)

df_subset = df.iloc[1368:1900]
#df_subset["time"] -= 5.73435115814209 
df_subset["time"] -= 5.745


# Plot the data
plt.figure(figsize=(8, 5))
#plt.plot(df_subset['knee'], df_subset['hip'], marker='o', linestyle='-', color='b', label="Shank vs Jump Height")
plt.plot( df_subset['time'], df_subset['knee'])
plt.plot( df_subset['time'], df_subset['hip'])
plt.plot( df_subset['time'], df_subset['base'])
plt.grid()

# Customize the plot
plt.ylabel("[rad]")
plt.xlabel("Time [s]")
plt.title("Joint positions vs Time")
plt.legend(["Knee", "Hip", 'Base'])
#plt.grid(True)
plt.savefig('angle_time.png')

# Show the plot
plt.show()
