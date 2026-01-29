#!/usr/bin/env python3
"""Serial JSON recipe client for the syringe filler."""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency
    serial = None


RECIPE_MAX_STEPS = 32
COLOR_HEX_RE = re.compile(r"^#[0-9a-fA-F]{6}$")


@dataclass
class ValidationError:
    index: Optional[int]
    message: str


def _is_positive_int(value: Any) -> bool:
    return isinstance(value, int) and value > 0


def validate_recipe_steps(steps: Iterable[Dict[str, Any]]) -> List[ValidationError]:
    errors: List[ValidationError] = []
    steps_list = list(steps)

    if not steps_list:
        errors.append(ValidationError(None, "recipe must have at least one step"))
        return errors

    if len(steps_list) > RECIPE_MAX_STEPS:
        errors.append(
            ValidationError(None, f"recipe has {len(steps_list)} steps (max {RECIPE_MAX_STEPS})")
        )

    for idx, step in enumerate(steps_list):
        if not isinstance(step, dict):
            errors.append(ValidationError(idx, "step must be an object"))
            continue

        volume = step.get("volume_ml")
        if not isinstance(volume, (int, float)) or volume <= 0:
            errors.append(ValidationError(idx, "volume_ml must be a positive number"))

        base_slot = step.get("base_slot")
        base_rfid = step.get("base_rfid")
        color_hex = step.get("color_hex")
        paint_id = step.get("paint_id")

        identifiers = [
            _is_positive_int(base_slot),
            _is_positive_int(base_rfid),
            isinstance(color_hex, str) and bool(color_hex.strip()),
            _is_positive_int(paint_id),
        ]
        if not any(identifiers):
            errors.append(
                ValidationError(
                    idx,
                    "step requires an identifier (base_slot, base_rfid, color_hex, or paint_id)",
                )
            )

        if base_slot is not None and not _is_positive_int(base_slot):
            errors.append(ValidationError(idx, "base_slot must be a positive integer"))
        if base_rfid is not None and not _is_positive_int(base_rfid):
            errors.append(ValidationError(idx, "base_rfid must be a positive integer"))
        if paint_id is not None and not _is_positive_int(paint_id):
            errors.append(ValidationError(idx, "paint_id must be a positive integer"))

        if color_hex is not None:
            if not isinstance(color_hex, str) or not COLOR_HEX_RE.match(color_hex):
                errors.append(ValidationError(idx, "color_hex must look like #RRGGBB"))

    return errors


def parse_recipe_payload(payload: Any) -> List[Dict[str, Any]]:
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict) and "steps" in payload:
        if isinstance(payload["steps"], list):
            return payload["steps"]
    raise ValueError("recipe JSON must be an array or an object with a 'steps' array")


def parse_recipe_id(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "Recipe ID must be an integer (decimal or 0xHEX)"
        ) from exc


def ensure_serial_available() -> None:
    if serial is None:
        raise SystemExit("pyserial is required: pip install pyserial")


def open_serial(port: str, baud: int, timeout: float):
    ensure_serial_available()
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def send_command(ser, payload: Dict[str, Any]) -> None:
    line = json.dumps(payload, separators=(",", ":"))
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()


def read_response(ser, timeout: float) -> Dict[str, Any]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            continue
    raise TimeoutError("timed out waiting for JSON response")


def print_response(response: Dict[str, Any], pretty: bool) -> None:
    if pretty:
        print(json.dumps(response, indent=2))
    else:
        print(json.dumps(response))


def handle_list(args: argparse.Namespace) -> Dict[str, Any]:
    return {"cmd": "sfc.recipe.list"}


def handle_show(args: argparse.Namespace) -> Dict[str, Any]:
    return {"cmd": "sfc.recipe.show", "data": {"recipe_id": args.recipe_id}}


def handle_delete(args: argparse.Namespace) -> Dict[str, Any]:
    return {"cmd": "sfc.recipe.delete", "data": {"recipe_id": args.recipe_id}}


def handle_save(args: argparse.Namespace) -> Dict[str, Any]:
    with open(args.recipe, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    steps = parse_recipe_payload(payload)
    errors = validate_recipe_steps(steps)
    if errors:
        for err in errors:
            prefix = f"step {err.index + 1}: " if err.index is not None else ""
            print(f"validation error: {prefix}{err.message}", file=sys.stderr)
        raise SystemExit(2)
    return {
        "cmd": "sfc.recipe.save",
        "data": {
            "recipe_id": args.recipe_id,
            "recipe": steps,
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Syringe filler recipe serial client")
    parser.add_argument("--port", required=True, help="Serial device path, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for JSON")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON response")

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List stored recipes")

    show_parser = subparsers.add_parser("show", help="Show a specific recipe")
    show_parser.add_argument("recipe_id", type=parse_recipe_id, help="Recipe ID")

    save_parser = subparsers.add_parser("save", help="Save a recipe")
    save_parser.add_argument("recipe_id", type=parse_recipe_id, help="Recipe ID")
    save_parser.add_argument("recipe", help="Path to recipe JSON file")

    delete_parser = subparsers.add_parser("delete", help="Delete a stored recipe")
    delete_parser.add_argument("recipe_id", type=parse_recipe_id, help="Recipe ID")

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "list":
        payload = handle_list(args)
    elif args.command == "show":
        payload = handle_show(args)
    elif args.command == "save":
        payload = handle_save(args)
    elif args.command == "delete":
        payload = handle_delete(args)
    else:
        parser.error("unknown command")
        return

    try:
        with open_serial(args.port, args.baud, args.timeout) as ser:
            send_command(ser, payload)
            response = read_response(ser, args.timeout)
    except TimeoutError as exc:
        raise SystemExit(str(exc)) from exc

    print_response(response, args.pretty)


if __name__ == "__main__":
    main()
