import pandas as pd
import matplotlib.pyplot as plt

# Load data
csv_filename = "best_results.csv"
df = pd.read_csv(csv_filename)

# Define thigh length groups
group_1 = [0.4, 0.425, 0.475, 0.45]
group_2 = [0.5, 0.525, 0.55]

# Function to plot scatter for a group of thigh lengths
def plot_scatter(df, thigh_lengths, title, filename):
    plt.figure(figsize=(10, 7))
    
    for thigh_length in thigh_lengths:
        subset = df[df["thigh_length"] == thigh_length]  # Filter data
        
        if subset.empty:
            print(f"No data found for thigh length {thigh_length}")
            continue  # Skip if no data for this thigh length

        plt.scatter(subset["calf_length"], subset["best_cost"], label=f"Thigh {thigh_length}", s=50)

    # Labels and title
    plt.xlabel("Calf Length")
    plt.ylabel("Best Cost")
    plt.title(title)
    plt.legend(title="Thigh Length")
    plt.grid(True)

    # Save the plot
    plt.savefig(filename)

    # Show the plot
    plt.show()

# Plot for Group 1
plot_scatter(df, group_1, "Scatter Plot: Cost vs. Calf Length (Thighs: 0.4, 0.425, 0.475, 0.45)", "scatter_group1.png")

# Plot for Group 2
plot_scatter(df, group_2, "Scatter Plot: Cost vs. Calf Length (Thighs: 0.5, 0.525, 0.55)", "scatter_group2.png")
