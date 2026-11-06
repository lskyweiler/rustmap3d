"""Summarize pytest-benchmark JSON output as a markdown table.

Usage:
    pytest --benchmark-json=out.json bench/bench_single.py
    python bench/summarize.py out.json [more.json ...] > summary.md
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

UNITS = ((1.0, "s"), (1e-3, "ms"), (1e-6, "µs"), (1e-9, "ns"))


def fmt_time(seconds: float) -> str:
    """format a timestring to nano seconds"""
    for scale, unit in UNITS:
        if seconds >= scale:
            return f"{seconds / scale:.2f} {unit}"
    return f"{seconds / 1e-9:.2f} ns"


def table(benchmarks: list[dict]) -> str:
    """Render one table, sorted fastest first, with speedup relative to the slowest."""
    rows = sorted(benchmarks, key=lambda b: b["stats"]["mean"])
    slowest = rows[-1]["stats"]["mean"]
    lines = [
        "| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for b in rows:
        s = b["stats"]
        lines.append(
            f"| `{b['name']}` | {fmt_time(s['mean'])} | {fmt_time(s['median'])} "
            f"| {fmt_time(s['min'])} | {fmt_time(s['stddev'])} | {s['rounds']} "
            f"| {slowest / s['mean']:.1f}x |"
        )
    return "\n".join(lines)


def summarize(paths: list[Path]) -> str:
    """Summarize all benchmark files to a markdown"""
    sections = []
    for path in paths:
        data = json.loads(path.read_text())
        benchmarks = data.get("benchmarks", [])
        if not benchmarks:
            continue
        # one table per file, or per group when the benchmarks define groups
        groups: dict[str, list[dict]] = {}
        for b in benchmarks:
            groups.setdefault(b.get("group") or path.stem, []).append(b)
        for name, group in groups.items():
            sections.append(f"### {name}\n\n{table(group)}")
    return "\n\n".join(sections) + "\n"


def main():
    """main function"""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])  # type: ignore
    parser.add_argument(
        "files", nargs="+", type=Path, help="pytest-benchmark JSON files"
    )
    args = parser.parse_args()
    print(summarize(args.files), end="")


if __name__ == "__main__":
    main()
