"""Dispatch a configured session to its operator-selected agent harness."""
import argparse
from pathlib import Path

from harness.backends import get
from services.controller.config import Configuration


def selected(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', type=Path, required=True)
    args, _ = parser.parse_known_args(argv)
    return get(Configuration.load(args.config).agent_cli)


def main():
    selected().main()


if __name__ == '__main__':
    main()
