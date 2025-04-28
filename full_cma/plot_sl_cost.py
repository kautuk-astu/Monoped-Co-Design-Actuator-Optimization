import pandas as pd
import matplotlib.pyplot as plt

# Load data
csv_filename = "sorted_best_results.csv"
df = pd.read_csv(csv_filename)

# Define thigh length groups
group_1 = [0.4, 0.425, 0.45]
group_2 = [0.475, 0.5, 0.525]
group_3 = [0.55,0.575,0.6]


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
plot_scatter(df, group_1, "Scatter Plot: Best Cost vs. Calf Length (Thighs: 0.4, 0.425, 0.45)", "scatter_group1_bestcost.png")

# Plot for Group 2
plot_scatter(df, group_2, "Scatter Plot: Best Cost vs. Calf Length (Thighs: 0.475, 0.5, 0.525)", "scatter_group2_bestcost.png")

# Plot for Group 3
plot_scatter(df, group_3, "Scatter Plot: Best Cost vs. Calf Length (Thighs: 0.55, 0.575, 0.6)", "scatter_group3_bestcost.png")

# Function to plot line graph for a group of thigh lengths
def plot_line(df, thigh_lengths, title, filename):
    plt.figure(figsize=(10, 7))
    
    for thigh_length in thigh_lengths:
        subset = df[df["thigh_length"] == thigh_length]  # Filter data
        
        if subset.empty:
            print(f"No data found for thigh length {thigh_length}")
            continue  # Skip if no data for this thigh length

        plt.plot(subset["calf_length"], subset["best_cost"], marker='o', label=f"Thigh {thigh_length}")

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
# Line plot for Group 1
plot_line(df, group_1, "Line Plot: Cost vs. Calf Length (Thighs: 0.4, 0.425, 0.45)", "line_group1_bestcost.png")
# Line plot for Group 2
plot_line(df, group_2, "Line Plot: Cost vs. Calf Length (Thighs: 0.475, 0.5, 0.525)", "line_group2_bestcost.png")
# Line plot for Group 3
plot_line(df, group_3, "Line Plot: Cost vs. Calf Length (Thighs: 0.55, 0.575, 0.6)", "line_group3_bestcost.png")