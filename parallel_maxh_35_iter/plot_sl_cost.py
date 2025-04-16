import pandas as pd
import matplotlib.pyplot as plt

# Load data
csv_filename = "sorted_final_results.csv"
df = pd.read_csv(csv_filename)

# Define thigh length groups
group_1 = [0.4]
group_2 = [0.425]
group_3 = [0.45]
# group_1 = [0.4, 0.425, 0.45]
# group_2 = [0.475, 0.5, 0.525]
# group_3 = [0.55,0.575,0.6]
group_4 = [0.475]
group_5 = [0.5]
group_6 = [0.525]
group_7 = [0.55]
group_8 = [0.575]
group_9 = [0.6]


# Function to plot scatter for a group of thigh lengths
def plot_scatter(df, thigh_lengths, title, filename):
    plt.figure(figsize=(10, 7))
    
    for thigh_length in thigh_lengths:
        subset = df[df["calf_length"] == thigh_length]  # Filter data
        
        if subset.empty:
            print(f"No data found for calf length {thigh_length}")
            continue  # Skip if no data for this thigh length

        plt.scatter(subset["thigh_length"], subset["mode_height"], label=f"Calf {thigh_length}", s=50)

    # Labels and title
    plt.xlabel("thigh Length")
    plt.ylabel("Mode Height")
    plt.title(title)
    plt.legend(title="calf Length")
    plt.grid(True)

    # Save the plot
    plt.savefig(filename)

    # Show the plot
    plt.show()
#write code to add calf and thigh lengths, to get a measure of total length and then obtain the height being achieved by this sum of link lengths
 


# Plot for Group 1
#plot_scatter(df, group_1, "Scatter Plot: Height vs. thigh Length (calfs: 0.4, 0.425, 0.45)", "scatter_group1_height_calfs.png")

# Plot for Group 2
#plot_scatter(df, group_2, "Scatter Plot: Height vs. thigh Length (calfs: 0.475, 0.5, 0.525)", "scatter_group2_height_calfs.png")

# Plot for Group 3
#plot_scatter(df, group_3, "Scatter Plot: Height vs. thigh Length (calfs: 0.55, 0.575, 0.6)", "scatter_group3_height_calfs.png")
# Plot for Group 1
plot_scatter(df, group_1, "Scatter Plot: Height vs. thigh Length (calfs: 0.4)", "scatter_group1_height_calfs.png")
# Plot for Group 2
plot_scatter(df, group_2, "Scatter Plot: Height vs. thigh Length (calfs: 0.425)", "scatter_group2_height_calfs.png")
# Plot for Group 3
plot_scatter(df, group_3, "Scatter Plot: Height vs. thigh Length (calfs: 0.45)", "scatter_group3_height_calfs.png")
# Plot for Group 4
plot_scatter(df, group_4, "Scatter Plot: Height vs. thigh Length (calfs: 0.475)", "scatter_group4_height_calfs.png")
# Plot for Group 5
plot_scatter(df, group_5, "Scatter Plot: Height vs. thigh Length (calfs: 0.5)", "scatter_group5_height_calfs.png")
# Plot for Group 6
plot_scatter(df, group_6, "Scatter Plot: Height vs. thigh Length (calfs: 0.525)", "scatter_group6_height_calfs.png")
# Plot for Group 7
plot_scatter(df, group_7, "Scatter Plot: Height vs. thigh Length (calfs: 0.55)", "scatter_group7_height_calfs.png")
# Plot for Group 8
plot_scatter(df, group_8, "Scatter Plot: Height vs. thigh Length (calfs: 0.575)", "scatter_group8_height_calfs.png")
# Plot for Group 9
plot_scatter(df, group_9, "Scatter Plot: Height vs. thigh Length (calfs: 0.6)", "scatter_group9_height_calfs.png")

# Function to plot line graph for a group of thigh lengths
def plot_line(df, thigh_lengths, title, filename):
    plt.figure(figsize=(10, 7))
    
    for thigh_length in thigh_lengths:
        subset = df[df["calf_length"] == thigh_length]  # Filter data
        
        if subset.empty:
            print(f"No data found for calf length {thigh_length}")
            continue  # Skip if no data for this thigh length

        plt.plot(subset["thigh_length"], subset["mode_height"], marker='o', label=f"calf {thigh_length}")

    # Labels and title
    plt.xlabel("thigh Length")
    plt.ylabel("Mode Height")
    plt.title(title)
    plt.legend(title="calf Length")
    plt.grid(True)

    # Save the plot
    plt.savefig(filename)

    # Show the plot
    plt.show()
# # Line plot for Group 1
# plot_line(df, group_1, "Line Plot: Cost vs. thigh Length (calfs: 0.4, 0.425, 0.45)", "line_group1_height_calfs.png")
# # Line plot for Group 2
# plot_line(df, group_2, "Line Plot: Cost vs. thigh Length (calfs: 0.475, 0.5, 0.525)", "line_group2_height_calfs.png")
# # Line plot for Group 3
# plot_line(df, group_3, "Line Plot: Cost vs. thigh Length (calfs: 0.55, 0.575, 0.6)", "line_group3_height_calfs.png")
# Line plot for Group 1
plot_line(df, group_1, "Line Plot: Height vs. thigh Length (calfs: 0.4)", "line_group1_height_calfs.png")
# Line plot for Group 2
plot_line(df, group_2, "Line Plot: Height vs. thigh Length (calfs: 0.425)", "line_group2_height_calfs.png")
# Line plot for Group 3
plot_line(df, group_3, "Line Plot: Height vs. thigh Length (calfs: 0.45)", "line_group3_height_calfs.png")
# Line plot for Group 4
plot_line(df, group_4, "Line Plot: Height vs. thigh Length (calfs: 0.475)", "line_group4_height_calfs.png")
# Line plot for Group 5
plot_line(df, group_5, "Line Plot: Height vs. thigh Length (calfs: 0.5)", "line_group5_height_calfs.png")
# Line plot for Group 6
plot_line(df, group_6, "Line Plot: Height vs. thigh Length (calfs: 0.525)", "line_group6_height_calfs.png")
# Line plot for Group 7
plot_line(df, group_7, "Line Plot: Height vs. thigh Length (calfs: 0.55)", "line_group7_height_calfs.png")
# Line plot for Group 8
plot_line(df, group_8, "Line Plot: Height vs. thigh Length (calfs: 0.575)", "line_group8_height_calfs.png")
# Line plot for Group 9
plot_line(df, group_9, "Line Plot: Height vs. thigh Length (calfs: 0.6)", "line_group9_height_calfs.png")
