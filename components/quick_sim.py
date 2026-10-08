from pathlib import Path
import json
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET

import opt_codesign_5bar as sim


ROOT = Path(__file__).resolve().parent.parent

# --------------------------------------------------
# Nominal configuration used by cmaes_baseline.py
# --------------------------------------------------
thigh_length = 0.297
calf_length = 0.302
torso_width = 0.1
ik_height = 0.3002049217058575

motor = "U10"
gear_ratio = 6.0

controller = np.array([
    670.0,
    5.7,
    10.1
])

ori_l = 0.2200611217066836
ori_theta = 1.075783873110416
# --------------------------------------------------
# Read the actuator properties used by the repo
# --------------------------------------------------
lookup = pd.read_csv(ROOT / "results" / "optimal_gearbox_selection.csv")
rows = lookup[lookup["motor"] == motor]

idx = (rows["target_ratio"] - gear_ratio).abs().idxmin()
row = rows.loc[idx]

actuator_mass = float(row["mass"])
efficiency = float(row["efficiency"])
gearbox = row["gearbox"]

with open(
    ROOT / "actuator_optimization" / "config_files" / "config.json"
) as f:
    config = json.load(f)

motor_cfg = config["Motors"][f"Motor{motor}_framed"]

Kv = motor_cfg["Kv"]
I_cont = motor_cfg["maxContinuousCurrent"]

Kt = 60.0 / (2.0 * np.pi * Kv)
motor_torque = Kt * I_cont
joint_torque = motor_torque * gear_ratio

print("Motor:", motor)
print("Gearbox:", gearbox)
print("Gear ratio:", gear_ratio)
print("Actuator mass:", actuator_mass)
print("Efficiency:", efficiency)
print("Joint torque limit:", joint_torque)

# --------------------------------------------------
# Generate nominal XML
# --------------------------------------------------
source_xml = ROOT / "xmls" / "5bar_base.xml"
test_xml = ROOT / "xmls" / "quick_initial.xml"

tree = ET.parse(source_xml)
xml_root = tree.getroot()

# Put body at the nominal IK height
body = xml_root.find(".//body[@name='root']")
pos = body.get("pos").split()
pos[2] = str(ik_height)
body.set("pos", " ".join(pos))

# Account for the two actuators
torso = xml_root.find(".//geom[@name='torso']")
torso.set("mass", str(2.0 * actuator_mass))

# Match actuator limits
ctrl_limit = efficiency * joint_torque

for name in ["motor_left", "motor_right"]:
    actuator = xml_root.find(f".//motor[@name='{name}']")
    actuator.set(
        "ctrlrange",
        f"{-ctrl_limit} {ctrl_limit}"
    )

tree.write(test_xml)

# --------------------------------------------------
# Run exactly ONE simulation
# --------------------------------------------------
result = sim.run(
    str(test_xml),
    controller,
    -ik_height,
    joint_torque,
    joint_torque,
    thigh_length,
    calf_length,
    torso_width / 2.0,
    efficiency,
    efficiency,
    ori_l,
    ori_theta,
)

height, velocity, distance, energy, duration, jumps = result

print("\n==========================")
print("SIMULATION COMPLETE")
print("==========================")
print("Jump height   :", height)
print("Jump distance :", distance)
print("Velocity      :", velocity)
print("Energy        :", energy)
print("Duration      :", duration)
print("Detected jumps:", len(jumps))
