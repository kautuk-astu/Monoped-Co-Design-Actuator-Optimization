import pandas as pd

# Load the existing link combinations
link_df = pd.read_csv("lengths_grid.csv")

# Load mass data for thigh and calf
thigh_mass = pd.read_csv("thigh_03_05.csv")  # columns: length_mm, mass
calf_mass = pd.read_csv("calf_03_05.csv")    # columns: length_mm, mass

# Filter only values ending in 0 (400, 410, ..., 600)
#thigh_mass = thigh_mass[thigh_mass["Link Length (mm)"]]
# First, ensure length is numeric
thigh_mass["Link Length (mm)"] = thigh_mass["Link Length (mm)"].astype(float)

# Filter between 300mm and 500mm (i.e., 0.3m to 0.5m)
thigh_mass = thigh_mass[
    (thigh_mass["Link Length (mm)"] >= 300.0) & (thigh_mass["Link Length (mm)"] <= 500.0)
]

#calf_mass = calf_mass[calf_mass["Link Length (mm)"]]
calf_mass["Link Length (mm)"] = calf_mass["Link Length (mm)"].astype(float)

calf_mass = calf_mass[
    (calf_mass["Link Length (mm)"] >= 300.0) & (calf_mass["Link Length (mm)"] <= 500.0)
]



# Convert mm to meters for matching
thigh_mass["length_m"] = thigh_mass["Link Length (mm)"] / 1000
calf_mass["length_m"] = calf_mass["Link Length (mm)"] / 1000

# Create lookup dictionaries
thigh_lookup = dict(zip(thigh_mass["length_m"], thigh_mass["Calculated Mass (kg)"]))
calf_lookup = dict(zip(calf_mass["length_m"], calf_mass["Calculated Mass (kg)"]))

# Replace the mass column with total mass from lookup (thigh + calf)
def compute_mass(row):
    t = round(row["thigh_length"], 3)
    c = round(row["calf_length"], 3)
    thigh_m = thigh_lookup.get(t, None)
    calf_m = calf_lookup.get(c, None)
    return thigh_m + calf_m + 1.6 if thigh_m is not None and calf_m is not None else ""

def ik_value(row):
   return 0.6

link_df["mass"] = link_df.apply(compute_mass, axis=1)

# Clear ik_value column
link_df["ik_value"] = link_df.apply(ik_value, axis=1)

# Save updated CSV
link_df.to_csv("final_links_with_mass.csv", index=False)
print("Updated CSV with total mass (including 1.6 kg) saved as final_links_with_mass.csv")
