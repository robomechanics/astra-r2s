"""Reproduce the contact-driven pre-engaged nut experiment and its failures."""
from __future__ import annotations

import argparse
import json

from .gripper import threaded_grip_benchmark
from .model import ThreadConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="outputs/m8/demo")
    parser.add_argument("--single-turn", action="store_true",
                        help="Turn once, then release and reset the open hand; omit the second grip/turn")
    parser.add_argument("--angular-speed", type=float, default=4., help="Hand turn rate in radians/second")
    parser.add_argument("--resume", action="store_true", help="Resume an identical demo at its saved full-state phase checkpoint")
    parser.add_argument("--contact-margin", type=float, default=None,
                        help="Explicit diagnostic contact margin in meters; default uses ThreadConfig")
    parser.add_argument("--contact-impratio", type=float, default=None,
                        help="Explicit diagnostic solver friction/normal impedance ratio")
    args = parser.parse_args()
    options = {}
    if args.contact_margin is not None:
        options["contact_margin"] = args.contact_margin
    if args.contact_impratio is not None:
        options["contact_impratio"] = args.contact_impratio
    report = threaded_grip_benchmark(args.output, regrasp=not args.single_turn,
                                    release_reset=True, angular_speed=args.angular_speed,
                                    config=ThreadConfig(**options), resume=args.resume)
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
