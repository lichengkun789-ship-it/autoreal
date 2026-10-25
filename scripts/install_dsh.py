#!/usr/bin/env python3
"""Install the pinned DeepSeek Harness outside committed source files."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.deepseek_cli.installation import install  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--node', type=Path, help='Node.js 24+ executable')
    parser.add_argument('--npm', type=Path, help='npm executable paired with --node')
    args = parser.parse_args()
    print(json.dumps(install(args.prefix, args.cache, node=args.node, npm=args.npm), indent=2))


if __name__ == '__main__':
    main()
