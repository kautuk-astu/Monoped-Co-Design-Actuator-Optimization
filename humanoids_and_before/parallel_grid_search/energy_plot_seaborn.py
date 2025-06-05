import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load the final results file
final_results_file = "final_results.csv"
df = pd.read_csv(final_results_file)

# Get all unique heights (convert to integers if they are rounded values like 0.4, 0.5, etc.)
unique_heights = sorted(df["mode_height"].unique())

# Set Seaborn style
sns.set_style("whitegrid")

# Create separate scatter plots for each unique mode height
for height in unique_heights:
    filtered_df = df[df["mode_height"] == height]

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(8, 6))

    # Scatter plot
    scatter = sns.scatterplot(
        data=filtered_df, x="thigh_length", y="calf_length", 
        hue="avg_energy", palette="coolwarm", size="avg_energy", 
        sizes=(50, 200), edgecolor="k", ax=ax
    )

    # Add color bar
    norm = plt.Normalize(filtered_df["avg_energy"].min(), filtered_df["avg_energy"].max())
    sm = plt.cm.ScalarMappable(cmap="coolwarm", norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label("Energy Consumption")

    # Labels and title
    ax.set_xlabel("Thigh Length")
    ax.set_ylabel("Calf Length")
    ax.set_title(f"Energy Consumption for Mode Height = {height}")

    # Save plot (convert height to integer for filename)
    filename = f"energy_plot_height_{int(height*100)}.png"
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    print(f"Saved: {filename}")

    # Close figure to free memory
    plt.close(fig)
