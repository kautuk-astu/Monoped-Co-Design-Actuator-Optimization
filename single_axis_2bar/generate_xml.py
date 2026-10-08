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
    parser.add_argument("--crouch-fraction", type=float, default=0.65)
    parser.add_argument("--thrust-time", type=float, default=0.12)
    parser.add_argument("--hip-kp", type=float, default=300.0)
    parser.add_argument("--knee-kp", type=float, default=300.0)
    args = parser.parse_args()
    design = Design(
        args.thigh, args.shank, args.payload,
        actuator_for(args.hip_motor, args.hip_ratio),
        actuator_for(args.knee_motor, args.knee_ratio),
        args.crouch_fraction, args.thrust_time, args.hip_kp, args.knee_kp,
    )
    write_xml(design, args.output)
    print(f"Wrote {args.output}")
    print(f"Hip: {design.hip.motor} {design.hip.gearbox}, ratio {design.hip.ratio:.2f}")
    print(f"Knee: {design.knee.motor} {design.knee.gearbox}, ratio {design.knee.ratio:.2f}")


if __name__ == "__main__":
    main()
