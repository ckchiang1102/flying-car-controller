#!/usr/bin/env python3
"""Compute the standard deviation of the sensor logs from scenario 06_SensorNoise.

Usage:
    python3 calc_noise_std.py                 # reads config/log/Graph1.txt and Graph2.txt
    python3 calc_noise_std.py some/other.txt  # or any logged graph

Standard library only -- no numpy needed, so it runs on any Python 3.
"""
import csv
import os
import statistics
import sys

DEFAULT_LOGS = ["config/log/Graph1_06.csv", "config/log/Graph2_06.csv"]


def read_log(path):
    """Return (signal name, list of values) from a simulator graph log."""
    with open(path, newline="") as f:
        rows = csv.reader(f)
        header = next(rows)
        # logs are "time, <signal name>"
        signal = header[-1].strip()
        values = [float(r[1]) for r in rows if len(r) >= 2 and r[1].strip()]
    return signal, values


def analyze(path):
    signal, values = read_log(path)
    if len(values) < 2:
        print(f"{os.path.basename(path)}: only {len(values)} sample(s), nothing to do\n")
        return

    mean = statistics.fmean(values)
    # sample standard deviation (divides by n-1), which is what we want when
    # estimating the sensor's sigma from a finite number of samples
    std = statistics.stdev(values)
    # fraction of samples inside +/- 1 sigma; a Gaussian gives ~68.3%
    within = sum(abs(v - mean) < std for v in values) / len(values) * 100

    print(f"{os.path.basename(path)}  [{signal}]")
    print(f"  samples       : {len(values)}")
    print(f"  mean          : {mean:+.4f}")
    print(f"  std dev       : {std:.4f}   <-- put this in config/06_SensorNoise.txt")
    print(f"  within 1 sigma: {within:.1f}%  (Gaussian expects ~68.3%)")
    print()


if __name__ == "__main__":
    # run from anywhere: resolve the default logs relative to this file
    here = os.path.dirname(os.path.abspath(__file__))
    paths = sys.argv[1:] or [os.path.join(here, p) for p in DEFAULT_LOGS]

    print(f"(python {sys.version.split()[0]} at {sys.executable})\n")
    for p in paths:
        if os.path.exists(p):
            analyze(p)
        else:
            print(f"{p}: not found -- run scenario 06_SensorNoise first\n")
