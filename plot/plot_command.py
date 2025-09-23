# plot_compare_3x2.py
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
        description="Confronto 3×2: tre variabili (colonne 2..4) per due CSV (si ignora la prima colonna)."
    )
    ap.add_argument("file_a", help="Primo CSV (con header; >=4 colonne)")
    ap.add_argument("file_b", help="Secondo CSV (con header; >=4 colonne)")
    ap.add_argument("--markers", action="store_true", help="Mostra i punti oltre alla linea")
    args = ap.parse_args()

    x_a, data_a, xlab_a, cols_a = load_three_columns(args.file_a)
    x_b, data_b, xlab_b, cols_b = load_three_columns(args.file_b)

    # figura 3x2
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex='col')

    # nomi file brevi per i titoli colonna
    name_a = os.path.basename(args.file_a)
    name_b = os.path.basename(args.file_b)

    # Colonna sinistra = file A
    for i in range(3):
        y = data_a.iloc[:, i].to_numpy()
        ax = axes[i, 0]
        ax.plot(x_a, y, linestyle="-")
        if args.markers: ax.scatter(x_a, y, s=10)
        ax.grid(True)
        ax.set_ylabel(cols_a[i])
        if i == 0:
            ax.set_title(name_a)
    axes[2, 0].set_xlabel(xlab_a)

    # Colonna destra = file B
    for i in range(3):
        y = data_b.iloc[:, i].to_numpy()
        ax = axes[i, 1]
        ax.plot(x_b, y, linestyle="-")
        if args.markers: ax.scatter(x_b, y, s=10)
        ax.grid(True)
        ax.set_ylabel(cols_b[i])
        if i == 0:
            ax.set_title(name_b)
    axes[2, 1].set_xlabel(xlab_b)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
