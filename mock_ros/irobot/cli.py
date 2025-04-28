#!/usr/bin/env python3
import argparse
import importlib
import sys
import logging

def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(prog="iRobot")
    subparsers = parser.add_subparsers(dest="command")

    run_p = subparsers.add_parser("run", help="Run a node")
    run_p.add_argument("module_path", help="Module path under irobot (e.g. nodes_py)")
    run_p.add_argument("node_name",   help="Node file name (e.g. talker)")
    run_p.add_argument("node_args", nargs=argparse.REMAINDER, help="Arguments passed to the node")

    args = parser.parse_args()
    if args.command != "run":
        parser.print_help()
        sys.exit(1)

    module_name = f"irobot.{args.module_path}.{args.node_name}"
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        logging.error(f"Could not import {module_name}: {e}")
        print(f"Could not import {module_name}: {e}", file=sys.stderr)
        sys.exit(1)

    if not hasattr(module, "main"):
        logging.error(f"{module_name} has no main()")
        sys.exit(1)

    try:
        module.main(args.node_args)
    except Exception:
        logging.exception(f"Error while running {module_name}.main()")
        sys.exit(1)
