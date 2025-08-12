import pandas as pd
import matplotlib.pyplot as plt

# Leggi il file CSV
df = pd.read_csv('drone_positions.csv')  # sostituisci 'file.csv' con il nome del tuo file

df.columns = ['rcRoll', 'rcPitch', 'rcThrottle']

# Plot
plt.figure(figsize=(10,6))
plt.plot(df['rcRoll'], label='rcRoll')
plt.plot(df['rcPitch'], label='rcPitch')
plt.plot(df['rcThrottle'], label='rcThrottle')

plt.xlabel('Index')
plt.ylabel('Valori')
plt.title('Plot colonne rcRoll, rcPitch, rcThrottle')
plt.legend()
plt.grid(True)
plt.savefig('saved_csv.png')
plt.show()