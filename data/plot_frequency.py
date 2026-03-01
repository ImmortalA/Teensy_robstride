#!/usr/bin/env python3
"""
Graph frequency (Hz) from response_log CSV and print statistics.
Requires: pip install pandas matplotlib
Usage: python plot_frequency.py
"""

import sys
from pathlib import Path
from typing import Optional

import pandas as pd
import matplotlib.pyplot as plt

CSV_PATH = Path(__file__).resolve().parent / "response_log_3_motor.csv"
COLUMNS = ["bus", "node", "hz", "us"]


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, header=None, names=COLUMNS)
    df["motor"] = df["bus"].astype(str) + "-" + df["node"].astype(str)
    return df


def get_stats_lines(df: pd.DataFrame) -> list[str]:
    lines = ["## Per motor (bus, node)\n"]
    for (bus, node), g in df.groupby(["bus", "node"]):
        h = g["hz"]
        lines.append(f"- **Bus {bus} node {node}**: n={len(h)}, min={h.min():.2f} Hz, max={h.max():.2f} Hz, mean={h.mean():.2f} Hz, std={h.std():.2f} Hz, median={h.median():.2f} Hz\n")
    return lines


def print_stats(df: pd.DataFrame) -> None:
    lines = get_stats_lines(df)
    print("=== Per motor (bus, node) ===")
    for line in lines[1:]:  # skip markdown header
        print(" ", line.strip().replace("**", "").replace("- ", ""))


def save_stats(df: pd.DataFrame, stats_path: Path, source_name: str) -> None:
    lines = ["# Frequency statistics\n", f"Source: {source_name}\n", f"Total samples: {len(df)}\n\n"]
    lines.extend(get_stats_lines(df))
    stats_path.write_text("".join(lines), encoding="utf-8")
    print(f"Stats saved to {stats_path}")


def plot_frequency(df: pd.DataFrame, out_path: Optional[Path] = None) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Top: Hz vs sample index, one series per motor
    ax1 = axes[0]
    for motor in df["motor"].unique():
        sub = df[df["motor"] == motor]
        ax1.plot(sub.index, sub["hz"], label=f"Bus {sub['bus'].iloc[0]} node {sub['node'].iloc[0]}", alpha=0.7)
    ax1.set_ylabel("Frequency (Hz)")
    ax1.set_title("Send→response frequency by motor")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Bottom: Latency (us) vs sample index
    ax2 = axes[1]
    for motor in df["motor"].unique():
        sub = df[df["motor"] == motor]
        ax2.plot(sub.index, sub["us"], label=f"Bus {sub['bus'].iloc[0]} node {sub['node'].iloc[0]}", alpha=0.7)
    ax2.set_xlabel("Sample index")
    ax2.set_ylabel("Latency (µs)")
    ax2.set_title("Send→response latency by motor")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"\nPlot saved to {out_path}")
    plt.show()


def main() -> None:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else CSV_PATH
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(path)
    print(f"Loaded {len(df)} rows from {path}\n")
    print_stats(df)

    stats_path = path.with_suffix(".md")
    save_stats(df, stats_path, path.name)

    out_path = path.with_suffix(".png")
    plot_frequency(df, out_path)


if __name__ == "__main__":
    main()
