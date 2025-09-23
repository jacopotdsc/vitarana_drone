# plot_csv.py
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def main():
    parser = argparse.ArgumentParser(description="Plotta dati da CSV (header: t,x,y,z)")
    parser.add_argument("file", help="Percorso del file CSV")
    parser.add_argument("axes", nargs="?", default="xyz", help="Stringa che indica le colonne da plottare (es. xy, xz, yz, xyz). Default: xyz")
    args = parser.parse_args()

    # Legge CSV
    df = pd.read_csv(args.file)
    df.columns = [c.strip().lower() for c in df.columns]

    # Conversione numerica
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna()

    # Estrai le colonne richieste
    cols = list(args.axes.lower())
    for c in cols:
        if c not in df.columns:
            raise ValueError(f"Colonna '{c}' non trovata nel CSV. Colonne disponibili: {list(df.columns)}")

    if len(cols) == 2:
        a, b = cols
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.plot(df[a].to_numpy(), df[b].to_numpy(), linestyle="-")
        ax.set_xlabel(a.upper())
        ax.set_ylabel(b.upper())
        ax.set_title(f"{a.upper()} vs {b.upper()}")
        ax.grid(True)
        ax.axis("equal")

    elif len(cols) == 3:
        a, b, c = cols
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(df[a].to_numpy(), df[b].to_numpy(), df[c].to_numpy(), linestyle="-")
        ax.set_xlabel(a.upper())
        ax.set_ylabel(b.upper())
        ax.set_zlabel(c.upper())
        ax.set_title(f"Traiettoria 3D ({a.upper()}, {b.upper()}, {c.upper()})")
        try:
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            mins = np.array([df[a].min(), df[b].min(), df[c].min()])
            maxs = np.array([df[a].max(), df[b].max(), df[c].max()])
            centers = (mins + maxs) / 2
            max_range = (maxs - mins).max() or 1.0
            ax.set_xlim(centers[0] - max_range/2, centers[0] + max_range/2)
            ax.set_ylim(centers[1] - max_range/2, centers[1] + max_range/2)
            ax.set_zlim(centers[2] - max_range/2, centers[2] + max_range/2)

    else:
        raise ValueError("Puoi passare solo 2 o 3 lettere (es. xy, yz, xz, xyz).")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
