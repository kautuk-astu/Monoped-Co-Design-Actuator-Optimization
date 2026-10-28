import json
from pathlib import Path

from jump_engine_visual import evaluate

ROOT = Path(__file__).resolve().parent

# Load the final optimised design
with open(ROOT / "results_stage4/B_best.json") as f:
    result = json.load(f)

# Load the motor configuration
with open(ROOT / "motor_config.json") as f:
    cfg = json.load(f)

hip = result["gearboxes"]["hip"]
knee = result["gearboxes"]["knee"]
gains = result["pd_gains"]

# Apply optimised gearbox properties
cfg["gearbox_efficiencies"] = [
    hip["efficiency"],
    knee["efficiency"]
]

cfg["actuator_masses_kg"] = [
    hip["mass"],
    knee["mass"]
]

# Optimisation vector:
# [hip_ratio, knee_ratio, hip_Kp, hip_Kd, knee_Kp, knee_Kd]
x = [
    hip["ratio"],
    knee["ratio"],
    gains["hip_Kp"],
    gains["hip_Kd"],
    gains["knee_Kp"],
    gains["knee_Kd"]
]

print("\nFINAL OPTIMISED MONOPED")
print("=" * 45)
print("Configuration:", result["configuration"])
print("Hip ratio:", x[0])
print("Knee ratio:", x[1])
print("Hip Kp:", x[2])
print("Hip Kd:", x[3])
print("Knee Kp:", x[4])
print("Knee Kd:", x[5])
print("=" * 45)

print("\nLaunching MuJoCo animation...")

score, metrics, trajectory = evaluate(
    x,
    cfg,
    duration=1.4,
    record=True
)

print("\nSIMULATION RESULTS")
print(json.dumps(metrics, indent=2))
