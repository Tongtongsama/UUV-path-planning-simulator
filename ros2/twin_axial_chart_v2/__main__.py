"""CLI for chart-v2."""
from __future__ import annotations
import argparse
from pathlib import Path
from .adapter import ChartV2


def main() -> int:
    p = argparse.ArgumentParser(description="Build a chart-v2 WorldModel/SDF skeleton")
    p.add_argument("--metadata", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    chart = ChartV2.from_path(args.metadata)
    chart.write(args.output)
    print(f"chart={chart.metadata['name']} obstacles={len(chart.obstacles)} depth={chart.depth.shape} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
