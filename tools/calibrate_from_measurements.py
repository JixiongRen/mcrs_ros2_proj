#!/usr/bin/env python3
"""Fit encoder scale from independent signed travel measurements, offline only.

Input CSV columns: count_start,count_end,measured_displacement_mm.
Positive measured displacement denotes insertion; withdrawal is negative.
The measurement must come from an independent instrument, never a commanded
target or servo theoretical position. This tool never imports a device driver.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def calibrate(path: Path) -> tuple[list[dict], dict]:
    rows = []
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"count_start", "count_end", "measured_displacement_mm"}
        if not required <= set(reader.fieldnames or []):
            raise ValueError(f"input must contain {sorted(required)}")
        for number, row in enumerate(reader, 2):
            try:
                start, end = int(row["count_start"]), int(row["count_end"])
                measured = float(row["measured_displacement_mm"])
            except (ValueError, TypeError) as exc:
                raise ValueError(f"invalid numeric measurement on CSV line {number}") from exc
            if not math.isfinite(measured) or measured == 0 or end == start:
                raise ValueError(
                    f"finite nonzero independent travel/count required on line {number}"
                )
            if not all(-(2**63) <= count <= 2**64 - 1 for count in (start, end)):
                raise ValueError(
                    f"count exceeds supported signed/unsigned 64-bit range on line {number}"
                )
            delta = end - start
            rows.append(
                {
                    "line": number,
                    "count_start": start,
                    "count_end": end,
                    "delta_count": delta,
                    "measured_displacement_mm": measured,
                    "signed_mm_per_count": measured / delta,
                    "direction": "insertion" if measured > 0 else "withdrawal",
                }
            )
    if not rows:
        raise ValueError("input contains no measurements")
    signs = {1 if row["signed_mm_per_count"] > 0 else -1 for row in rows}
    if len(signs) != 1:
        raise ValueError("count/physical direction is inconsistent between measurements")
    signed_scale = sum(row["delta_count"] * row["measured_displacement_mm"] for row in rows) / sum(
        row["delta_count"] ** 2 for row in rows
    )
    if not math.isfinite(signed_scale) or signed_scale == 0:
        raise ValueError("fitted scale must be finite and nonzero")
    residuals = [
        row["measured_displacement_mm"] - signed_scale * row["delta_count"] for row in rows
    ]
    groups = {}
    for direction in ("insertion", "withdrawal"):
        scales = [abs(row["signed_mm_per_count"]) for row in rows if row["direction"] == direction]
        groups[direction] = {
            "samples": len(scales),
            "mean_mm_per_count": statistics.mean(scales) if scales else None,
            "std_mm_per_count": statistics.stdev(scales) if len(scales) > 1 else None,
        }
    warnings = []
    if any(group["samples"] < 2 for group in groups.values()):
        warnings.append(
            "At least two independent trips per direction are needed to assess repeatability."
        )
    if signed_scale < 0:
        warnings.append(
            "Raw encoder direction opposes insertion: verify the direction mapping; do not silently apply a positive scale."
        )
    result = {
        "method": "least_squares_displacement_equals_signed_scale_times_delta_count",
        "samples": len(rows),
        "signed_mm_per_count": signed_scale,
        "mm_per_count_magnitude": abs(signed_scale),
        "direction_sign": next(iter(signs)),
        "residual_rmse_mm": math.sqrt(sum(value**2 for value in residuals) / len(residuals)),
        "directions": groups,
        "warnings": warnings,
        "hardware_verified_by_this_tool": False,
        "operator_must_confirm": [
            "independent measurement provenance",
            "device/count format and epoch continuity",
            "direction",
            "load and travel range",
            "repeatability",
            "calibration version",
        ],
    }
    return rows, result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="independent measured travel CSV")
    parser.add_argument(
        "--output", type=Path, required=True, help="output directory, never a device path"
    )
    args = parser.parse_args(argv)
    try:
        rows, result = calibrate(args.input)
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / "samples.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (args.output / "calibration.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "samples": len(rows),
                    "output": str(args.output / "calibration.json"),
                    "hardware_verified": False,
                }
            )
        )
        return 0
    except (ValueError, OSError, OverflowError) as exc:
        print(f"calibration failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
