"""Command-line entry point for the synthetic chart adapter skeleton."""
from __future__ import annotations
import argparse
from pathlib import Path
from .chart import Chart


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert a chart-v1 JSON into ENU/WorldModel/SDF skeleton")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    chart = Chart.from_path(args.input)
    chart.write(args.output)
    print(f"chart={chart.name} obstacles={len(chart.obstacles)} depth={chart.depth.shape} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
