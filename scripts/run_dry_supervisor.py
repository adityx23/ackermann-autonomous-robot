#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


def main() -> int:
    from ackermann_robot.main import RobotSupervisor
    from ackermann_robot.utils.logging_utils import setup_logging

    parser = argparse.ArgumentParser(description="Run the mock-only dry-run robot supervisor.")
    parser.add_argument("--config-dir", default="config", help="Directory containing config YAML files.")
    parser.add_argument("--cycles", type=int, default=30, help="Number of supervisor cycles to run.")
    parser.add_argument("--no-sleep", action="store_true", help="Run cycles without fixed-rate sleeping.")
    args = parser.parse_args()

    setup_logging(console=True, level=logging.INFO)
    supervisor = RobotSupervisor(config_dir=args.config_dir, dry_run=True)
    supervisor.run(cycles=args.cycles, sleep=not args.no_sleep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
