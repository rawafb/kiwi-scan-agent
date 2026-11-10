# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

"""Command-line entry point for creating a complete scan config."""

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from kiwi_scan.tools.config_template import default_scan_config_yaml


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="scanconfig",
        description=(
            "Create a complete YAML scan config with active defaults and "
            "commented examples of nested configuration blocks."
        ),
        epilog=(
            "Examples:\n"
            "  scanconfig\n"
            "  scanconfig scan.yaml\n"
            "  scanconfig --force scan.yaml"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "output",
        nargs="?",
        default="-",
        metavar="FILE",
        help="Output file. Omit it or use '-' to write to stdout.",
    )
    parser.add_argument(
        "--force",
        "-f",
        action="store_true",
        help="Overwrite FILE if it already exists.",
    )
    args = parser.parse_args(argv)

    rendered = default_scan_config_yaml()
    if args.output == "-":
        sys.stdout.write(rendered)
        return 0

    path = Path(args.output).expanduser()
    mode = "w" if args.force else "x"
    try:
        with path.open(mode, encoding="utf-8") as stream:
            stream.write(rendered)
    except FileExistsError:
        parser.error(
            f"output file already exists: {path} (use --force to overwrite)"
        )
    except OSError as exc:
        parser.error(f"cannot write {path}: {exc}")
    return 0


if __name__ == "__main__":
    main()
