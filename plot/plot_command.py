# plot_compare_3xN.py
import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_three_columns(csv_path):
    df = pd.read_csv(csv_path)
    if df.shape[1] < 4:
        raise ValueError(f"{csv_path}: servono almeno 4 colonne, trovate {df.shape[1]}")
    # prendi colonne 2..4 (ignoriamo la 1ª)
    data = df.iloc[:, 1:4].copy()
    # porta a numerico
    for c in data.columns:
        data[c] = pd.to_numeric(data[c], errors="coerce")
    data = data.dropna()
    # asse X: prova a usare la 1ª colonna come tempo, se numerica; altrimenti indice
    x_time = pd.to_numeric(df.iloc[:, 0], errors="coerce")
    if x_time.notna().all():
        x = x_time.to_numpy()
        x_label = df.columns[0]
    else:
        x = data.index.to_numpy()
        x_label = "Indice campione"
    return x, data, x_label, [str(c) for c in data.columns]

def main():
    ap = argparse.ArgumentParser(
        description="Confronto 3×N: tre variabili (colonne 2..4) per 2 o 3 CSV (si ignora la prima colonna)."
    )
    ap.add_argument("files", nargs="+", help="2 o 3 CSV (con header; >=4 colonne ciascuno)")
    ap.add_argument("--markers", action="store_true", help="Mostra i punti oltre alla linea")
    args = ap.parse_args()

    if not (2 <= len(args.files) <= 3):
        raise SystemExit("Devi passare 2 o 3 file CSV.")

    # carica tutti i file
    loaded = []
    for path in args.files:
        x, data, xlab, cols = load_three_columns(path)
        loaded.append((x, data, xlab, cols, os.path.basename(path)))

    ncols = len(loaded)
    nrows = 3

    fig, axes = plt.subplots(nrows, ncols, figsize=(5*ncols + 2, 10), sharex='col')
    # se ncols==1, axes non è 2D; forziamo una matrice per semplicità
    if ncols == 1:
        axes = np.array([[axes]]*nrows).reshape(nrows, 1)

    # per ogni colonna/file
    for j, (x, data, xlab, cols, name) in enumerate(loaded):
        for i in range(nrows):
            y = data.iloc[:, i].to_numpy()
            ax = axes[i, j]
            ax.plot(x, y, linestyle="-")
            if args.markers:
                ax.scatter(x, y, s=10)
            ax.grid(True)
            ax.set_ylabel(cols[i])
            if i == 0:
                ax.set_title(name)
        axes[nrows-1, j].set_xlabel(xlab)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
