import pandas as pd
import matplotlib.pyplot as plt

# Leggi il file CSV
df = pd.read_csv('drone_positions.csv')

# Rinominare le colonne
df.columns = ['rcRoll', 'rcPitch', 'rcThrottle', 'pidX', 'pidY', 'pidZ']

# Crea una figura con 2 subplot (2 righe, 1 colonna)
fig, axs = plt.subplots(1, 1, figsize=(12, 10), sharex=True)

# Primo subplot: rcRoll, rcPitch, rcThrottle
axs.plot(df['rcRoll'], label='rcRoll')
axs.plot(df['rcPitch'], label='rcPitch')
axs.plot(df['rcThrottle'], label='rcThrottle')
axs.set_ylabel('Valori RC')
axs.set_title('Comandi RC')
axs.legend()
axs.grid(True)

# Secondo subplot: pidX, pidY, pidZ
'''
axs[1].plot(df['pidX'], label='pidX')
axs[1].plot(df['pidY'], label='pidY')
axs[1].plot(df['pidZ'], label='pidZ')
axs[1].set_xlabel('Index')
axs[1].set_ylabel('Valori PID')
axs[1].set_title('Valori PID X, Y, Z')
axs[1].legend()
axs[1].grid(True)
'''

# Salva e mostra
plt.tight_layout()
plt.savefig('drone_plots.png')
plt.show()
