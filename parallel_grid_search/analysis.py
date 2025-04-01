import pandas as pd

# Read the CSV files
df1 = pd.read_csv('mode_height_data_425_400.csv')
df2 = pd.read_csv('mode_height_data_400_425.csv')

def mode_and_mean(df, col1, col2):
    # Get the mode(s) of the first column
    modes = df[col1].mode()
    
    mode_results = {}
    count_results = {}

    for mode_value in modes:
        # Filter rows where the first column matches the mode value
        filtered_df = df[df[col1] == mode_value]
        # Compute the mean of the second column for these rows
        mean_value = filtered_df[col2].mean()
        # Count occurrences of this mode value
        count_value = filtered_df.shape[0]
        # Store results
        mode_results[mode_value] = mean_value
        count_results[mode_value] = count_value

    # Compute overall mean of second column
    overall_mean = df[col2].mean()
    
    return mode_results, count_results, overall_mean


results_df1, counts_df1, overall_mean_df1 = mode_and_mean(df1, 'Mode Height', 'Avg Energy')
results_df2, counts_df2, overall_mean_df2 = mode_and_mean(df2, 'Mode Height', 'Avg Energy')

# Print results
# print("First CSV - Mode(s) and Mean(s):", results_df1)
# print("First CSV - Count of Each Mode:", counts_df1)
# print("First CSV - Overall Mean of Second Column:", overall_mean_df1)
# print("Second CSV - Mode(s) and Mean(s):", results_df2)
# print("Second CSV - Count of Each Mode:", counts_df2)
# print("Second CSV - Overall Mean of Second Column:", overall_mean_df2)

#obtain the mean value for specific HEIGHTS 
filtered_df1 = df1[df1['Mode Height'] == -0.3]
mean_value1 = filtered_df1['Avg Energy'].mean()
count_value1 = filtered_df1.shape[0]
filtered_df2 = df2[df2['Mode Height'] == -0.3]
mean_value2 = filtered_df2['Avg Energy'].mean()
count_value2 = filtered_df2.shape[0]
print("Mean value for Mode Height 0.1 in first CSV:", mean_value1)
print("Count for Mode Height 0.1 in first CSV:", count_value1)
print("Mean value for Mode Height 0.1 in second CSV:", mean_value2)
print("Count for Mode Height 0.1 in second CSV:", count_value2)

#0.575,600 better than 06,0.600 for all values except -0.1
#0.550,600 better than 06,0.600 for all values except -0.1, 0.4
#0.550,575 better than 06,0.600 for all values except -0.1
#0.550,550 better than 06,0.600 for all values except -0.1
#0.550,550 better than 0.6,0.575
#0.550,550 better than 0.6,0.550
#550,525 better than 0.6,0.525
#500,500 better than 0.550,500 only at -0.1
#525,550 better than 0.550,525 only at -0.1
#525,550 better than 575,525 in some and not in some
#425,550 almost same as 475,550 in some 
#425,550 better than 450,550 in some and not in some
#400,425 better than 425,400 in some and not in some








