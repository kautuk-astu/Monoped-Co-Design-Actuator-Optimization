"""Command-line generator for one vertical two-bar jumper XML."""

from __future__ import annotations

import argparse
from pathlib import Path

from model import Design, MOTORS, actuator_for, write_xml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--thigh", type=float, default=0.25)
    parser.add_argument("--shank", type=float, default=0.25)
    parser.add_argument("--payload", type=float, default=1.0)
    parser.add_argument("--hip-motor", choices=MOTORS, default="U8")
    parser.add_argument("--knee-motor", choices=MOTORS, default="U8")
    parser.add_argument("--hip-ratio", type=float, default=10.0)
    parser.add_argument("--knee-ratio", type=float, default=10.0)
    parser.add_argument("--theta1", type=float, default=-0.25)
    parser.add_argument("--theta2", type=float, default=0.50)
    parser.add_argument("--ori-l", type=float, default=0.45)
    parser.add_argument("--ori-theta", type=float, default=0.0)
    parser.add_argument("--K", type=float, default=400.0)
    parser.add_argument("--C", type=float, default=4.0)
    parser.add_argument("--T-gain", type=float, default=20.0)
    parser.add_argument("--thrust-time", type=float, default=0.12)
    args = parser.parse_args()
    design = Design(
        thigh=args.thigh, shank=args.shank, payload_mass=args.payload,
        hip=actuator_for(args.hip_motor, args.hip_ratio), knee=actuator_for(args.knee_motor, args.knee_ratio),
        theta1=args.theta1, theta2=args.theta2, ori_l=args.ori_l, ori_theta=args.ori_theta,
        K=args.K, C=args.C, T_gain=args.T_gain, thrust_time=args.thrust_time,
    )
    write_xml(design, args.output)
    print(f"Wrote {args.output}")
    print(f"Hip: {design.hip.motor} {design.hip.gearbox}, ratio {design.hip.ratio:.2f}")
    print(f"Knee: {design.knee.motor} {design.knee.gearbox}, ratio {design.knee.ratio:.2f}")


if __name__ == "__main__":
    main()
