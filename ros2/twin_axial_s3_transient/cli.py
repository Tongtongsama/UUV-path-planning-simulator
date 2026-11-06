"""CLI for the independent S3 transient-feasibility study (Python only)."""
import argparse
from pathlib import Path

from .study import audit, screen, analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)

    audit_parser = sub.add_parser("audit", help="read-only audit of existing S3 records")
    audit_parser.add_argument("--source", type=Path, required=True)
    audit_parser.add_argument("--output", type=Path, required=True)

    screen_parser = sub.add_parser("screen", help="run the 12-group Python screen")
    screen_parser.add_argument("--output", type=Path, required=True)

    analyze_parser = sub.add_parser("analyze", help="read-only check of a screen directory")
    analyze_parser.add_argument("directory", type=Path)

    args = parser.parse_args()
    if args.mode in ("audit", "screen") and args.output.exists():
        parser.error("Output exists; choose a new directory")
    if args.mode == "audit":
        result = audit(args.source, args.output)
    elif args.mode == "screen":
        result = screen(args.output)
    else:
        result = analyze(args.directory)
    raise SystemExit(0 if result.get("passed", True) else 2)


if __name__ == "__main__":
    main()
