"""Model generation and actuator lookup for the vertical two-bar jumper."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
ACTUATOR_TABLE = ROOT / "results" / "optimal_gearbox_selection.csv"
MOTOR_CONFIG = ROOT / "actuator_optimization" / "config_files" / "config.json"
MOTORS = ("U8", "U10", "U12", "MN8014", "VT8020", "MAD_M6C12")


@dataclass(frozen=True)
class Actuator:
    motor: str
    ratio: float
    gearbox: str
    mass: float
    efficiency: float
    continuous_output_torque: float


@dataclass(frozen=True)
class Design:
    thigh: float
    shank: float
    payload_mass: float
    hip: Actuator
    knee: Actuator
    crouch_fraction: float
    thrust_time: float
    hip_kp: float
    knee_kp: float


def _continuous_motor_torque(motor: str) -> float:
    with MOTOR_CONFIG.open() as stream:
        spec = json.load(stream)["Motors"][f"Motor{motor}_framed"]
    return 60.0 / (2.0 * 3.141592653589793 * float(spec["Kv"])) * float(spec["maxContinuousCurrent"])


def actuator_for(motor: str, ratio: float) -> Actuator:
    """Choose the closest actuator-table entry, matching the main experiments."""
    if motor not in MOTORS:
        raise ValueError(f"Unknown motor {motor!r}; choose one of {', '.join(MOTORS)}")
    if not ACTUATOR_TABLE.is_file():
        raise FileNotFoundError(f"Actuator lookup missing: {ACTUATOR_TABLE}")

    table = pd.read_csv(ACTUATOR_TABLE)
    options = table[table["motor"] == motor]
    if options.empty:
        raise ValueError(f"No actuator-table entries for motor {motor}")
    requested = round(float(ratio), 1)
    row = options.loc[(options["target_ratio"] - requested).abs().idxmin()]
    return Actuator(
        motor=motor,
        ratio=float(ratio),
        gearbox=str(row["gearbox"]),
        mass=float(row["mass"]),
        efficiency=float(row["efficiency"]),
        continuous_output_torque=_continuous_motor_torque(motor) * float(ratio),
    )


def link_mass(length: float, radius: float = 0.018, density: float = 700.0) -> float:
    """A simple tube-equivalent link mass; replace with a CAD-derived model later."""
    volume = 3.141592653589793 * radius**2 * length
    return density * volume


def write_xml(design: Design, output: Path) -> None:
    """Write a complete MuJoCo model for a one-axis body and two-bar leg."""
    if design.thigh <= 0 or design.shank <= 0:
        raise ValueError("Link lengths must be positive")
    if not 0.0 < design.crouch_fraction < 1.0:
        raise ValueError("crouch_fraction must lie between zero and one")

    foot_radius = 0.025
    standing_length = 0.97 * (design.thigh + design.shank)
    initial_height = standing_length + foot_radius
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
      <!-- This sphere is the hip motor cover.  Its mass is the selected hip
           motor/gearbox mass, and the hip hinge sits at its centre. -->
      <geom name="hip_motor_cover" type="sphere" pos="0 0 0" size="0.06" mass="{design.hip.mass}"
            rgba="0.16 0.43 0.82 1"/>
      <geom name="payload" type="box" pos="0.075 0 0" size="0.035 0.045 0.045"
            mass="{design.payload_mass}" rgba="0.2 0.6 1 1"/>
      <body name="thigh" pos="0 0 0">
        <joint name="hip" type="hinge" axis="0 1 0"/>
        <geom name="thigh_link" type="capsule" fromto="0 0 0 0 0 -{design.thigh}" size="0.018" mass="{thigh_mass}" rgba="0.9 0.25 0.2 1"/>
        <body name="shank" pos="0 0 -{design.thigh}">
          <joint name="knee" type="hinge" axis="0 1 0"/>
          <!-- The selected knee actuator mass is placed at the knee pivot. -->
          <geom name="knee_motor_cover" type="sphere" size="0.035" mass="{design.knee.mass}"
                rgba="0.18 0.45 0.84 1"/>
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
