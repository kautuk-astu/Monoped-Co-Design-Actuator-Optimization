import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = "/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/thigh_45/45/45fit.dat"  # Change this to your CSV file path
df = pd.read_csv(file_path, delimiter="\t")

#df_subset = df.iloc[1368:1900]
#df_subset["time"] -= 5.73435115814209 
#df_subset["time"] -= 5.745


# Plot the data
plt.figure(figsize=(8, 5))
#plt.plot(df_subset['knee'], df_subset['hip'], marker='o', linestyle='-', color='b', label="Shank vs Jump Height")
plt.plot( df['iteration'], df['bestever'])
plt.plot( df['iteration'], df['best'], marker='o')
plt.grid()

# Customize the plot
plt.ylabel("cost")
plt.xlabel("iterations")
plt.title("cost vs iterations")
plt.legend(["bestever cost", "best"])
#plt.grid(True)
plt.savefig('conv_plot.png')

# Show the plot
plt.show()
