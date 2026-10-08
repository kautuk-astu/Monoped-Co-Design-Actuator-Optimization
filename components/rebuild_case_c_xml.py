from pathlib import Path
import json
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
from scipy.interpolate import interp1d

ROOT = Path(__file__).resolve().parent.parent

# --------------------------------------------------
# Load the exact Case C selected by play_codesign_5bar.py
# --------------------------------------------------
with open(ROOT / "results/Opt_design_control_parameters/summary_CaseC.json") as f:
    summary = json.load(f)

p = summary["all_min_row"]

uid = p["Unique id"]

thigh = float(p["Thigh"])
calf = float(p["Calf"])
torso_width = float(p["Torso distance"])
ik_height = float(p["ik_height"])

motor_left = p["Hip left motor"]
motor_right = p["Hip right motor"]

ratio_left = float(p["Hip left ratio"])
ratio_right = float(p["Hip right ratio"])

eff_left = float(p["Efficiency left"])
eff_right = float(p["Efficiency right"])

# --------------------------------------------------
# Actuator masses: same lookup logic as cmaes.py
# --------------------------------------------------
lookup = pd.read_csv(ROOT / "results/optimal_gearbox_selection.csv")

def gearbox_properties(motor, ratio):
    ratio = round(ratio, 1)
    rows = lookup[lookup["motor"] == motor]

    if rows.empty:
        raise RuntimeError(f"Motor not found: {motor}")

    idx = (rows["target_ratio"] - ratio).abs().idxmin()
    row = rows.loc[idx]

    return (
        float(row["mass"]),
        float(row["efficiency"]),
        row["gearbox"],
    )

mass_left, lookup_eff_left, gearbox_left = gearbox_properties(
    motor_left, ratio_left
)

mass_right, lookup_eff_right, gearbox_right = gearbox_properties(
    motor_right, ratio_right
)

# --------------------------------------------------
# Continuous motor torque
# --------------------------------------------------
with open(
    ROOT / "actuator_optimization/config_files/config.json"
) as f:
    cfg = json.load(f)

def continuous_torque(motor):
    m = cfg["Motors"][f"Motor{motor}_framed"]
    Kv = float(m["Kv"])
    Icont = float(m["maxContinuousCurrent"])

    Kt = 60.0 / (2.0 * np.pi * Kv)
    return Kt * Icont

tau_left = continuous_torque(motor_left) * ratio_left
tau_right = continuous_torque(motor_right) * ratio_right

# --------------------------------------------------
# Link masses: same interpolation used by cmaes.py
# --------------------------------------------------
df = pd.read_csv(ROOT / "components/calf_15_35.csv")
df = df.sort_values("Link Length (mm)")

lengths = df["Link Length (mm)"].values / 1000.0
masses = df["Calculated Mass (kg)"].values

mass_interp = interp1d(
    lengths,
    masses,
    kind="linear",
    bounds_error=False,
    fill_value="extrapolate",
)

thigh_mass = float(mass_interp(thigh))
calf_mass = float(mass_interp(calf))

# --------------------------------------------------
# Modify base XML exactly like Case C optimizer
# --------------------------------------------------
source = ROOT / "xmls/5bar_base.xml"

outdir = ROOT / "xmls/design_xmls"
outdir.mkdir(parents=True, exist_ok=True)

output = outdir / f"{uid}.xml"

tree = ET.parse(source)
root = tree.getroot()

half_width = torso_width / 2.0

# Base height
for body in root.findall(".//body[@name='root']"):
    pos = body.get("pos").split()
    pos[2] = str(ik_height)
    body.set("pos", " ".join(pos))

# Torso geometry and actuator mass
for geom in root.findall(".//geom[@name='torso']"):
    size = geom.get("size").split()
    size[0] = str(torso_width)
    geom.set("size", " ".join(size))
    geom.set("mass", str(mass_left + mass_right))

# Hip locations
for body in root.findall(".//body[@name='l1_left']"):
    body.set("pos", f"{-half_width} 0 0")

for body in root.findall(".//body[@name='l1_right']"):
    body.set("pos", f"{half_width} 0 0")

# Thigh geometry
for geom in root.findall(".//geom[@name='thigh_left']"):
    geom.set("fromto", f"0 0 0 {thigh} 0 0")
    geom.set("mass", str(thigh_mass))

for geom in root.findall(".//geom[@name='thigh_right']"):
    geom.set("fromto", f"0 0 0 {thigh} 0 0")
    geom.set("mass", str(thigh_mass))

# Knee locations
for body in root.findall(".//body[@name='l2_left']"):
    body.set("pos", f"{thigh} 0 0")

for body in root.findall(".//body[@name='l2_right']"):
    body.set("pos", f"{thigh} 0 0")

# Calf geometry
for geom in root.findall(".//geom[@name='shank_left']"):
    geom.set("fromto", f"0 0 0 {calf} 0 0")
    geom.set("mass", str(calf_mass))

for geom in root.findall(".//geom[@name='shank_right']"):
    geom.set("fromto", f"0 0 0 {calf} 0 0")
    geom.set("mass", str(calf_mass))

# Feet and sites
for name in ["foot_left", "foot_right"]:
    for geom in root.findall(f".//geom[@name='{name}']"):
        geom.set("pos", f"{calf} 0 0")

for name in ["left_tip", "right_tip"]:
    for site in root.findall(f".//site[@name='{name}']"):
        site.set("pos", f"{calf} 0 0")

# Actuator limits
for motor in root.findall(".//motor[@name='motor_left']"):
    limit = eff_left * tau_left
    motor.set("ctrlrange", f"-{limit} {limit}")

for motor in root.findall(".//motor[@name='motor_right']"):
    limit = eff_right * tau_right
    motor.set("ctrlrange", f"-{limit} {limit}")

tree.write(output)

print("\nCASE C XML GENERATED")
print("--------------------")
print("XML:", output)
print("UID:", uid)
print()
print("Thigh:", thigh)
print("Calf:", calf)
print("Torso distance:", torso_width)
print("IK height:", ik_height)
print()
print("Left :", motor_left, ratio_left, gearbox_left,
      "mass =", mass_left, "eff =", eff_left,
      "torque =", tau_left)
print("Right:", motor_right, ratio_right, gearbox_right,
      "mass =", mass_right, "eff =", eff_right,
      "torque =", tau_right)
print()
print("Expected distance:", p["Max distance"])
print("Expected energy:", p["Average energy"])
