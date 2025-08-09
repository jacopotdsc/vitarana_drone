#!/usr/bin/env python3
import argparse
import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def rmse(a, d):
    m = ~np.isnan(a) & ~np.isnan(d)
    return np.sqrt(np.mean((a[m]-d[m])**2)) if m.any() else np.nan

def main():
    ap = argparse.ArgumentParser(description="Plot X/Y/Z actual vs desired over time from a 6-column CSV (no header).")
    ap.add_argument("csv", help="CSV with columns: x_act, x_des, y_act, y_des, z_act, z_des (no header).")
    ap.add_argument("--hz", type=float, default=None, help="Sampling rate in Hz (e.g., 50).")
    ap.add_argument("--dt", type=float, default=None, help="Sampling period in seconds (e.g., 0.02).")
    ap.add_argument("--downsample", type=int, default=1, help="Keep every Nth row (default 1).")
    ap.add_argument("--out", default=None, help="Output PNG (default: <csv>_tracking.png).")
    ap.add_argument("--show", action="store_true", help="Show plot window.")
    ap.add_argument("--errors", action="store_true", help="Also plot error (actual - desired) under each axis.")
    args = ap.parse_args()

    if not os.path.isfile(args.csv):
        sys.exit(f"File not found: {args.csv}")

    df = pd.read_csv(args.csv, header=None)
    if df.shape[1] != 6:
        sys.exit(f"Expected exactly 6 columns, found {df.shape[1]}.")

    df.columns = ["x_act","x_des","y_act","y_des","z_act","z_des"]

    # Build time vector
    n = len(df)
    if args.hz and args.dt:
        sys.exit("Provide either --hz or --dt, not both.")
    if args.hz:
        t = np.arange(n, dtype=float) / float(args.hz)
        t_label = "time (s)"
    elif args.dt:
        t = np.arange(n, dtype=float) * float(args.dt)
        t_label = "time (s)"
    else:
        t = np.arange(n, dtype=float)
        t_label = "sample"

    # Downsample
    if args.downsample > 1:
        df = df.iloc[::args.downsample].reset_index(drop=True)
        t = t[::args.downsample]
        n = len(df)

    # Convert to numeric
    xa = pd.to_numeric(df["x_act"], errors="coerce").values
    xd = pd.to_numeric(df["x_des"], errors="coerce").values
    ya = pd.to_numeric(df["y_act"], errors="coerce").values
    yd = pd.to_numeric(df["y_des"], errors="coerce").values
    za = pd.to_numeric(df["z_act"], errors="coerce").values
    zd = pd.to_numeric(df["z_des"], errors="coerce").values

    # Errors & RMSE
    ex, ey, ez = xa - xd, ya - yd, za - zd
    rmse_x, rmse_y, rmse_z = rmse(xa, xd), rmse(ya, yd), rmse(za, zd)

    # Figure layout
    rows = 6 if args.errors else 3
    fig, axes = plt.subplots(rows, 1, figsize=(12, 10 if args.errors else 9), sharex=True)
    axes = np.atleast_1d(axes)

    # X
    axes[0].plot(t, xa, label="X actual")
    axes[0].plot(t, xd, linestyle="--", label="X desired")
    axes[0].set_ylabel("X (m)")
    axes[0].set_title(f"X tracking  (RMSE: {rmse_x:.3f} m)")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend()
    if args.errors:
        axes[1].plot(t, ex)
        axes[1].set_ylabel("X err (m)")
        axes[1].grid(True, linestyle="--", alpha=0.4)

    # Y
    idx = 2 if args.errors else 1
    axes[idx].plot(t, ya, label="Y actual")
    axes[idx].plot(t, yd, linestyle="--", label="Y desired")
    axes[idx].set_ylabel("Y (m)")
    axes[idx].set_title(f"Y tracking  (RMSE: {rmse_y:.3f} m)")
    axes[idx].grid(True, linestyle="--", alpha=0.4)
    axes[idx].legend()
    if args.errors:
        axes[idx+1].plot(t, ey)
        axes[idx+1].set_ylabel("Y err (m)")
        axes[idx+1].grid(True, linestyle="--", alpha=0.4)

    # Z
    idx = 4 if args.errors else 2
    axes[idx].plot(t, za, label="Z actual")
    axes[idx].plot(t, zd, linestyle="--", label="Z desired")
    axes[idx].set_ylabel("Z (m)")
    axes[idx].set_title(f"Z tracking  (RMSE: {rmse_z:.3f} m)")
    axes[idx].grid(True, linestyle="--", alpha=0.4)
    axes[idx].legend()
    if args.errors:
        axes[idx+1].plot(t, ez)
        axes[idx+1].set_ylabel("Z err (m)")
        axes[idx+1].grid(True, linestyle="--", alpha=0.4)

    axes[-1].set_xlabel(t_label)
    fig.suptitle("Actual vs Desired Position Tracking", y=0.98)
    fig.tight_layout(rect=[0,0,1,0.96])

    out_png = args.out or os.path.splitext(os.path.basename(args.csv))[0] + "_tracking.png"
    fig.savefig(out_png, dpi=160)
    print(f"Saved: {os.path.abspath(out_png)}")

    if args.show:
        plt.show()
    plt.close(fig)

if __name__ == "__main__":
    main()