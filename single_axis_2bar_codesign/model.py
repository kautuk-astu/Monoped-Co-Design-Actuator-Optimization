"""Model generation and actuator lookup for the vertical two-bar jumper."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import pandas as pd

HERE = Path(__file__).resolve().parent
ACTUATOR_TABLE = HERE / "data" / "sspg_actuator_selection.csv"
MOTOR_CONFIG = HERE / "data" / "motor_config.json"
MOTORS = ("U8", "U10", "U12", "MN8014", "VT8020", "MAD_M6C12", "MAD_M6C12_KV170")
CARRIAGE_BASE_MASS_KG = 0.25
KNEE_PIVOT_MASS_KG = 0.01


@dataclass(frozen=True)
class Actuator:
    motor: str
    ratio: float
    gearbox: str
    mass: float
    efficiency: float
    continuous_output_torque: float
    kv_rpm_per_volt: float = float("nan")
    rated_voltage: float = float("nan")
    resistance_ohm: float = float("nan")
    max_continuous_current: float = float("nan")


@dataclass(frozen=True)
class Design:
    thigh: float
    shank: float
    payload_mass: float
    hip: Actuator
    knee: Actuator
    theta1: float
    theta2: float
    ori_l: float
    ori_theta: float
    K: float
    C: float
    T_gain: float
    thrust_time: float
    # Aligned-controller parameters.  Defaults preserve compatibility with
    # saved legacy studies that predate horizontal Cartesian stabilization.
    Kx: float = 250.0
    Dx: float = 8.0
    prepare_time: float = 0.05
    flight_leg_length: float = 0.24
    landing_leg_length: float = 0.31
    joint_kp: float = 100.0
    joint_kd: float = 10.0


def _motor_spec(motor: str, motor_config: Path = MOTOR_CONFIG) -> dict[str, float]:
    """Normalize either the legacy motor file or COMPAct's motors.json."""
    with Path(motor_config).open() as stream:
        data = json.load(stream)
    if "motors" in data:
        if motor not in data["motors"]:
            raise ValueError(f"No motor configuration for {motor!r} in {motor_config}")
        entry = data["motors"][motor]
        electrical = entry["electrical"]
        kv = float(electrical["kv_rpm_per_V"])
        rated_current = float(electrical["rated_current_A"])
        rated_voltage = float(data.get("system_voltage_V", 48.0))
        rated_speed = float(electrical["rated_speed_rpm"])
        resistance = max(0.0, (rated_voltage - rated_speed / kv) / rated_current)
        return {
            "Kv": kv,
            "ratedVoltage": rated_voltage,
            "Resistance": resistance,
            "maxContinuousCurrent": rated_current,
            # Keep the simulation's established Kt=60/(2*pi*Kv) convention
            # consistent with its motor-current diagnostic.
            "continuousTorque": 60.0 / (2.0 * math.pi * kv) * rated_current,
        }
    key = f"Motor{motor}_framed"
    if key not in data.get("Motors", {}):
        raise ValueError(f"No motor configuration for {motor!r} in {motor_config}")
    spec = data["Motors"][key]
    return {
        **spec,
        "continuousTorque": (
            60.0 / (2.0 * math.pi * float(spec["Kv"]))
            * float(spec["maxContinuousCurrent"])
        ),
    }


def actuator_for(
    motor: str,
    ratio: float,
    gearbox: str | None = None,
    actuator_table: Path = ACTUATOR_TABLE,
    motor_config: Path = MOTOR_CONFIG,
) -> Actuator:
    """Choose the nearest feasible catalogue actuator for a motor/topology."""
    if motor not in MOTORS:
        raise ValueError(f"Unknown motor {motor!r}; choose one of {', '.join(MOTORS)}")
    actuator_table = Path(actuator_table)
    if not actuator_table.is_file():
        raise FileNotFoundError(f"Actuator lookup missing: {actuator_table}")

    table = pd.read_csv(actuator_table)
    compact_catalogue = "gearbox_type" in table.columns
    if compact_catalogue:
        required = {
            "gearbox_type", "requested_ratio", "actual_ratio", "mass_kg",
            "efficiency", "feasibility_status",
        }
        missing = required.difference(table.columns)
        if missing:
            raise ValueError(f"Catalogue {actuator_table} is missing columns: {sorted(missing)}")
        if gearbox is None:
            raise ValueError("gearbox must be specified for a multi-topology COMPAct catalogue")
        options = table[
            (table["gearbox_type"].str.upper() == gearbox.upper())
            & (table["feasibility_status"].str.lower() == "feasible")
        ].copy()
        target_column, mass_column, gearbox_column = "requested_ratio", "mass_kg", "gearbox_type"
    else:
        options = table[table["motor"] == motor].copy()
        if gearbox is not None:
            options = options[options["gearbox"].str.upper() == gearbox.upper()]
        target_column, mass_column, gearbox_column = "target_ratio", "mass", "gearbox"
    if options.empty:
        raise ValueError(f"No feasible {gearbox or ''} catalogue entries for motor {motor}")
    for column in (target_column, "actual_ratio", mass_column, "efficiency"):
        options[column] = pd.to_numeric(options[column], errors="coerce")
    options = options.dropna(subset=[target_column, "actual_ratio", mass_column, "efficiency"])
    if options.empty:
        raise ValueError(f"No numeric feasible rows for {gearbox or motor} in {actuator_table}")
    requested = round(float(ratio), 1)
    row = options.loc[(options[target_column] - requested).abs().idxmin()]
    spec = _motor_spec(motor, Path(motor_config))
    actual_ratio = float(row["actual_ratio"])
    return Actuator(
        motor=motor,
        ratio=actual_ratio,
        gearbox=str(row[gearbox_column]).upper(),
        mass=float(row[mass_column]),
        efficiency=float(row["efficiency"]),
        continuous_output_torque=float(spec["continuousTorque"]) * actual_ratio,
        kv_rpm_per_volt=float(spec["Kv"]), rated_voltage=float(spec["ratedVoltage"]),
        resistance_ohm=float(spec["Resistance"]),
        max_continuous_current=float(spec["maxContinuousCurrent"]),
    )


def link_mass(length: float, radius: float = 0.018, density: float = 700.0) -> float:
    """A simple tube-equivalent link mass; replace with a CAD-derived model later."""
    volume = 3.141592653589793 * radius**2 * length
    return density * volume


def write_xml(design: Design, output: Path) -> None:
    """Write a complete MuJoCo model for a one-axis body and two-bar leg."""
    if design.thigh <= 0 or design.shank <= 0:
        raise ValueError("Link lengths must be positive")
    if design.payload_mass <= 0 or design.thrust_time <= 0:
        raise ValueError("payload_mass and thrust_time must be positive")

    foot_radius = 0.025
    # Joint coordinates use MuJoCo's +y hinge axis.  With q=(theta1, theta2),
    # the toe relative to the hip is
    # p = [-l1 sin(q1) - l2 sin(q1+q2), 0,
    #      -l1 cos(q1) - l2 cos(q1+q2)].  Set the carriage height from p_z so
    # the foot starts tangent to the ground without constraining p_x.
    toe_z_relative = (
        -design.thigh * math.cos(design.theta1)
        - design.shank * math.cos(design.theta1 + design.theta2)
    )
    initial_height = foot_radius - toe_z_relative
    thigh_mass = link_mass(design.thigh)
    shank_mass = link_mass(design.shank)
    hip_limit = design.hip.efficiency * design.hip.continuous_output_torque
    knee_limit = design.knee.efficiency * design.knee.continuous_output_torque

    xml = f'''<mujoco model="single_axis_two_bar">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <!-- Accommodates the default 640x640 MP4 renderer. -->
    <global offwidth="1280" offheight="1280"/>
  </visual>
  <default>
    <joint damping="0.08"/>
    <geom friction="1.2 0.01 0.0001" condim="3"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.18 0.22 0.25 1"/>
    <!-- A visual linear guide.  It deliberately has no collision geometry:
         the pole_slide joint below, rather than contact with this pole, is
         what strictly constrains the carriage to vertical translation. -->
    <geom name="vertical_pole" type="capsule" fromto="-0.14 0 0 -0.14 0 2.5" size="0.03"
          mass="0" contype="0" conaffinity="0" rgba="0.28 0.30 0.34 1"/>
    <body name="carriage" pos="0 0 {initial_height}">
      <joint name="pole_slide" type="slide" axis="0 0 1"/>
      <!-- This massless slider block visibly wraps the guide pole. -->
      <geom name="guide_bracket" type="box" pos="-0.14 0 0" size="0.05 0.05 0.065"
            mass="0" contype="0" conaffinity="0" rgba="0.42 0.45 0.50 1"/>
      <!-- The slider link connects the pole carriage to the hip housing. -->
      <geom name="slider_link" type="capsule" fromto="-0.10 0 0 -0.055 0 0" size="0.018"
            mass="0" contype="0" conaffinity="0" rgba="0.42 0.45 0.50 1"/>
      <!-- All actuator mass rides on the vertically translating carriage. -->
      <geom name="carriage_body" type="box" pos="-0.055 0 0" size="0.07 0.055 0.055"
            mass="{CARRIAGE_BASE_MASS_KG}" rgba="0.32 0.35 0.40 1"/>
      <!-- Hip motor + gearbox: directly drives the hip. -->
      <geom name="hip_motor_cover" type="sphere" pos="0 0 0" size="0.06" mass="{design.hip.mass}"
            rgba="0.16 0.43 0.82 1"/>
      <!-- Knee motor + gearbox: mounted at the hip/carriage and remotely
           applies torque to the knee through an ideal massless 1:1 drive. -->
      <geom name="knee_motor_cover" type="sphere" pos="0.105 0 0" size="0.05" mass="{design.knee.mass}"
            rgba="0.18 0.45 0.84 1"/>
      <geom name="payload" type="box" pos="0.075 0 0" size="0.035 0.045 0.045"
            mass="{design.payload_mass}" rgba="0.2 0.6 1 1"/>
      <body name="thigh" pos="0 0 0">
        <joint name="hip" type="hinge" axis="0 1 0"/>
        <geom name="thigh_link" type="capsule" fromto="0 0 0 0 0 -{design.thigh}" size="0.018" mass="{thigh_mass}" rgba="0.9 0.25 0.2 1"/>
        <body name="shank" pos="0 0 -{design.thigh}">
          <joint name="knee" type="hinge" axis="0 1 0"/>
          <!-- Passive knee pivot only: the knee actuator is carried at hip. -->
          <geom name="knee_pivot" type="sphere" size="0.018" mass="{KNEE_PIVOT_MASS_KG}"
                rgba="0.30 0.30 0.32 1"/>
          <geom name="shank_link" type="capsule" fromto="0 0 0 0 0 -{design.shank}" size="0.016" mass="{shank_mass}" rgba="0.95 0.6 0.15 1"/>
          <geom name="foot" type="sphere" pos="0 0 -{design.shank}" size="{foot_radius}" mass="0.03" rgba="0.15 0.15 0.15 1"/>
          <site name="toe" pos="0 0 -{design.shank}" size="0.008"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="hip_motor" joint="hip" ctrlrange="-{hip_limit} {hip_limit}"/>
    <motor name="knee_motor" joint="knee" ctrlrange="-{knee_limit} {knee_limit}"/>
  </actuator>
</mujoco>
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml)
    # Parse once here so malformed generated XML fails before a long optimization.
    ET.parse(output)
