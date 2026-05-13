import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def compute_crlb_limit(sigma_gps, sigma_odo):
    """
    Calcule la limite théorique de l'écart-type de l'erreur (RMSE)
    en régime permanent pour ton filtre.
    """
    R = sigma_gps**2  # Variance du bruit de mesure
    Q = sigma_odo**2  # Variance du bruit de processus
    
    # Equation de Riccati pour la variance stationnaire P
    P_steady = (Q + np.sqrt(Q**2 + 4*Q*R)) / 2
    return np.sqrt(P_steady)

# 1. Chargement des données
df = pd.read_csv('robot_data.csv')

# 2. Préparation des distances pour l'analyse de l'odométrie
# Calcul de la distance parcourue par le GPS entre chaque pas de temps
df['dist_gps'] = np.sqrt(df['GPS_X'].diff()**2 + df['GPS_Y'].diff()**2)

# 3. Estimation rigoureuse des bruits (Variances)
# Bruit GPS : on utilise la différenciation pour isoler le bruit blanc
var_gps_x = np.var(np.diff(df['GPS_X'])) / 2
var_gps_y = np.var(np.diff(df['GPS_Y'])) / 2
sigma_gps_real = np.sqrt((var_gps_x + var_gps_y) / 2)

# Bruit Odométrie : écart entre le mouvement prédit (stepDist) et le mouvement vu par le GPS
# On enlève la première ligne (NaN à cause du diff)
residu_odo = (df['StepDist'] - df['dist_gps']).dropna()
sigma_odo_real = np.std(residu_odo)

# 4. Calcul de la borne théorique (BCR)
limit_bcr = compute_crlb_limit(sigma_gps_real, sigma_odo_real)

# 5. Calcul des erreurs réelles du filtre
error_simple = np.sqrt((df['Est_X'] - df['GPS_X'])**2 + (df['Est_Y'] - df['GPS_Y'])**2)
error_rk2 = np.sqrt((df['rk2_Est_X'] - df['GPS_X'])**2 + (df['rk2_Est_Y'] - df['GPS_Y'])**2)
time_final = df['Time'].iloc[-1]

print(f"--- RÉSULTATS DE L'ANALYSE ---")
print(f"Sigma GPS identifié : {sigma_gps_real:.4f} m")
print(f"Sigma Odo (par pas) : {sigma_odo_real:.4f} m")
print(f"Borne de Cramér-Rao (BCR) : {limit_bcr:.4f} m")
print(f"Erreur moyenne (Euler) : {error_simple.mean():.4f} m")
print(f"Erreur moyenne (RK2)   : {error_rk2.mean():.4f} m")
print(f"Time final : {time_final:.4f}")

# 6. Visualisation
plt.figure(figsize=(12, 9))

# GPS
plt.scatter(df['GPS_X'], df['GPS_Y'], s=2, c='c', alpha=0.5, label='GPS (Mesure)')

# Odométrie seule (la dérive)
plt.plot(df['Odo_X'], df['Odo_Y'], 'r--', linewidth=1, label='Odométrie seule')

# Filtres
plt.plot(df['Est_X'], df['Est_Y'], 'g-', linewidth=1.5, label='Filtre Euler')
plt.plot(df['rk2_Est_X'], df['rk2_Est_Y'], 'm:', linewidth=1.5, label='Filtre RK2')

plt.scatter(df['Est_X'].iloc[0], df['Est_Y'].iloc[0], c='yellow', s=100, edgecolors='black', label='Départ')

plt.xlabel('Position X (m)')
plt.ylabel('Position Y (m)')
plt.title(f'Analyse TSI : BCR = {limit_bcr:.4f}m vs Erreur RK2 = {error_rk2.mean():.4f}m')
plt.legend()
plt.grid(True, linestyle=':', alpha=0.6)
plt.axis('equal') 

plt.show()

# --- GRAPHIQUE DE DIAGNOSTIC DES ERREURS ---
plt.figure(figsize=(12, 6))

# Calcul de l'erreur instantanée pour RK2
inst_error = np.sqrt((df['rk2_Est_X'] - df['GPS_X'])**2 + (df['rk2_Est_Y'] - df['GPS_Y'])**2)

# Tracé de l'erreur au cours du temps
plt.plot(inst_error, label='Erreur instantanée (Filtre RK2)', color='magenta', alpha=0.7)

# Tracé de la moyenne
plt.axhline(y=inst_error.mean(), color='red', linestyle='--', label=f'Erreur moyenne ({inst_error.mean():.4f}m)')

# Tracé de la BCR (La limite physique)
plt.axhline(y=limit_bcr, color='black', linestyle='-', linewidth=2, label=f'Limite BCR ({limit_bcr:.4f}m)')

plt.fill_between(range(len(inst_error)), 0, limit_bcr, color='gray', alpha=0.2, label='Zone Inatteignable (Bruit capteur)')

plt.xlabel('Temps (Pas de simulation)')
plt.ylabel('Erreur de position (m)')
plt.title('Analyse de la Performance : Erreur Réelle vs Limite de Cramér-Rao')
plt.legend()
plt.grid(True, alpha=0.3)

plt.show()
