# src/mpdptw/common/test_cli.py
from pathlib import Path
import argparse
from .instance_tools import merge_instances

def main():
    parser = argparse.ArgumentParser(description="Test CLI for merging MPDPTW instances")
    parser.add_argument("--a", required=True, help="Path to first instance (keeps this depot)")
    parser.add_argument("--b", required=True, help="Path to second instance")
    parser.add_argument("--out", required=True, help="Output file path")
    parser.add_argument("--vehicles", type=int, default=None, help="Override number of vehicles")
    parser.add_argument("--capacity", type=int, default=None, help="Override vehicle capacity")
    args = parser.parse_args()

    result_path = merge_instances(
        path_a=args.a,
        path_b=args.b,
        out_path=args.out,
        vehicles=args.vehicles,
        capacity=args.capacity,
    )
    print(f"✅ Successfully wrote merged instance: {result_path}")

if __name__ == "__main__":
    main()
