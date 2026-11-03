import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.stats import linregress
import math

# ==========================================
# 1. RESOLVER RUTA DEL ARCHIVO
# ==========================================
dir_path = os.path.dirname(os.path.realpath(__file__))
file_path = os.path.join(dir_path, 'respuesta_libre_amortiguada.csv')

# ==========================================
# 2. CARGAR Y ACONDICIONAR DATOS
# ==========================================
df = pd.read_csv(file_path)
df['Time_s'] = (df['Timestamp_us'] - df['Timestamp_us'].iloc[0]) / 1e6
df['Angle_Continuous'] = df['Angulo_Grados'].apply(lambda x: x - 360 if x > 180 else x)
equilibrium = df[df['Time_s'] > 11]['Angle_Continuous'].mean()
df['Angle_Centered'] = df['Angle_Continuous'] - equilibrium

# ==========================================
# 3. FILTRAR Y FORZAR INICIO EN t ≈ 6.38s
# ==========================================
t_start_idx = df[df['Time_s'] >= 6.38].index[0]
t0 = df.loc[t_start_idx, 'Time_s']
A0 = df.loc[t_start_idx, 'Angle_Centered']

df_sub = df[df['Time_s'] > t0].copy()
peaks, _ = find_peaks(df_sub['Angle_Centered'], height=2, distance=50)
valleys, _ = find_peaks(-df_sub['Angle_Centered'], height=2, distance=50)

# Picos Positivos (Forzando t0 como el primer pico)
peak_times = np.insert(df_sub['Time_s'].iloc[peaks].values, 0, t0)
peak_values = np.insert(df_sub['Angle_Centered'].iloc[peaks].values, 0, A0)

# Valles (Picos negativos)
valley_times = df_sub['Time_s'].iloc[valleys].values
valley_values = -df_sub['Angle_Centered'].iloc[valleys].values

# ==========================================
# 4. CÁLCULOS MATEMÁTICOS
# ==========================================
# A) Decremento Picos Positivos
delta_pos = math.log(peak_values[0] / peak_values[1])
zeta_pos = delta_pos / math.sqrt(4 * math.pi**2 + delta_pos**2)

# B) Decremento Valles
delta_neg = math.log(valley_values[0] / valley_values[1])
zeta_neg = delta_neg / math.sqrt(4 * math.pi**2 + delta_neg**2)

# C) Regresión Exponencial
env_times = np.concatenate((peak_times[:3], valley_times[:2]))
env_values = np.concatenate((peak_values[:3], valley_values[:2]))
sort_idx = np.argsort(env_times)
env_times = env_times[sort_idx]
env_values = env_values[sort_idx]
slope, intercept, _, _, _ = linregress(env_times, np.log(env_values))

# --- PERIODO CALCULADO EXCLUSIVAMENTE CON LOS VALLES ---
Td_valles = valley_times[1] - valley_times[0]
omega_d = 2 * math.pi / Td_valles

# Zeta de regresión calculado con la omega_d de los valles
zeta_reg = math.sqrt(slope**2 / (omega_d**2 + slope**2))

# ==========================================
# 5. CÁLCULO DE FRECUENCIAS Y FRICCIÓN VISCOSA
# ==========================================
J = 4.4823e-04
mgl = 0.0267
omega_n_teorica = math.sqrt(mgl / J)

# Frecuencias experimentales independientes para cada Zeta (asegura precisión matemática en la curva)
omega_n_pos = omega_d / math.sqrt(1 - zeta_pos**2)
omega_n_neg = omega_d / math.sqrt(1 - zeta_neg**2)
omega_n_reg = omega_d / math.sqrt(1 - zeta_reg**2)

c_pos = 2 * zeta_pos * J * omega_n_pos
c_neg = 2 * zeta_neg * J * omega_n_neg
c_reg = 2 * zeta_reg * J * omega_n_reg

# ==========================================
# 6. IMPRESIÓN DETALLADA EN CONSOLA (SERIAL)
# ==========================================
print("\n" + "="*70)
print("          REPORTE DETALLADO DE CÁLCULO Y FÓRMULAS")
print("="*70)
print("1. PARÁMETROS FÍSICOS DEL SISTEMA:")
print(f"   - Inercia (J) = {J} kg.m^2")
print(f"   - Rigidez (mgl) = {mgl} N.m/rad")
print(f"   - Wn_teorica = sqrt(mgl / J) = {omega_n_teorica:.4f} rad/s")

print("\n2. EXTREMOS EXTRAÍDOS DE LOS DATOS EXPERIMENTALES:")
print(f"   - Inicio forzado (t0) = {t0:.4f} s  --> Amplitud (A0) = {A0:.4f}°")
print(f"   - Pico Positivo 2 (t1) = {peak_times[1]:.4f} s --> Amplitud (A1) = {peak_values[1]:.4f}°")
print(f"   - Valle 1 (t_v1) = {valley_times[0]:.4f} s --> Amplitud abs (v1) = {valley_values[0]:.4f}°")
print(f"   - Valle 2 (t_v2) = {valley_times[1]:.4f} s --> Amplitud abs (v2) = {valley_values[1]:.4f}°")

print("\n3. FÓRMULAS DEL DECREMENTO LOGARÍTMICO:")
print("   * Formula Delta (δ): δ = ln(x1 / x2)")
print("   * Formula Zeta (ζ): ζ = δ / sqrt(4*π^2 + δ^2)")
print(f"   -> [Positivos] δ_pos = ln({peak_values[0]:.2f} / {peak_values[1]:.2f}) = {delta_pos:.4f}")
print(f"   -> [Positivos] ζ_pos = {delta_pos:.4f} / sqrt(4π^2 + {delta_pos:.4f}^2) = {zeta_pos:.4f}")
print(f"   -> [Valles]    δ_neg = ln({valley_values[0]:.2f} / {valley_values[1]:.2f}) = {delta_neg:.4f}")
print(f"   -> [Valles]    ζ_neg = {delta_neg:.4f} / sqrt(4π^2 + {delta_neg:.4f}^2) = {zeta_neg:.4f}")

print("\n4. FÓRMULAS DE REGRESIÓN GLOBAL Y FRECUENCIA (Basado en Valles):")
print("   * Ecuación Envolvente: ln(A) = -ζ*Wn*t + ln(A0) -> Pendiente (m) = -ζ*Wn")
print(f"   -> Pendiente hallada (m): {slope:.4f}")
print("   * Periodo amortiguado (Td): Extraído SOLO de la distancia entre los dos primeros valles")
print(f"   -> Td_valles = {valley_times[1]:.4f} - {valley_times[0]:.4f} = {Td_valles:.4f} s")
print("   * Frecuencia amortiguada (Wd): Wd = 2*π / Td")
print(f"   -> Wd = 2*π / {Td_valles:.4f} = {omega_d:.4f} rad/s")
print("   * Formula Zeta Regresión: ζ_reg = sqrt(m^2 / (Wd^2 + m^2))")
print(f"   -> ζ_reg = sqrt({slope:.4f}^2 / ({omega_d:.4f}^2 + {slope:.4f}^2)) = {zeta_reg:.4f}")

print("\n5. CÁLCULO DE FRICCIÓN VISCOSA FINAL (c):")
print("   * Ecuación: c = 2 * ζ * J * Wn_exp  |  Donde: Wn_exp = Wd / sqrt(1 - ζ^2)")
print(f"   -> c_pos = 2 * {zeta_pos:.4f} * {J} * {omega_n_pos:.4f} = {c_pos:.6f} N.m.s/rad")
print(f"   -> c_neg = 2 * {zeta_neg:.4f} * {J} * {omega_n_neg:.4f} = {c_neg:.6f} N.m.s/rad")
print(f"   -> c_reg = 2 * {zeta_reg:.4f} * {J} * {omega_n_reg:.4f} = {c_reg:.6f} N.m.s/rad")
print("="*70 + "\n")

# ==========================================
# 7. GRÁFICAS DE SUPERPOSICIÓN (Iniciando en t0)
# ==========================================
t_sim = np.linspace(t0, t0 + 4.5, 500)

def calc_env(zeta, wn, t_arr):
    # Envolvente usando omega_n experimental y desfasada t0
    return A0 * np.exp(-zeta * wn * (t_arr - t0))

plt.figure(figsize=(12, 6))

plt.plot(df['Time_s'], df['Angle_Centered'], label='Datos Experimentales', color='black', alpha=0.75, linewidth=1.5)

# Valles
plt.plot(t_sim, calc_env(zeta_neg, omega_n_neg, t_sim), 'b--', label=rf'Valles ($\zeta$={zeta_neg:.3f})', linewidth=1.5)
plt.plot(t_sim, -calc_env(zeta_neg, omega_n_neg, t_sim), 'b--', linewidth=1.5)

# Picos
plt.plot(t_sim, calc_env(zeta_pos, omega_n_pos, t_sim), 'g--', label=rf'Picos Positivos ($\zeta$={zeta_pos:.3f})', linewidth=1.5)
plt.plot(t_sim, -calc_env(zeta_pos, omega_n_pos, t_sim), 'g--', linewidth=1.5)

# Regresión
plt.plot(t_sim, calc_env(zeta_reg, omega_n_reg, t_sim), 'r-', linewidth=2, label=rf'Regresión Global ($\zeta$={zeta_reg:.3f})')
plt.plot(t_sim, -calc_env(zeta_reg, omega_n_reg, t_sim), 'r-', linewidth=2)

plt.xlim(t0 - 0.2, t0 + 4)
plt.ylim(-150, 150)
plt.title(f'Superposición de Decaimiento (t0={t0:.2f}s | Td obtenido de valles)')
plt.xlabel('Tiempo (s)')
plt.ylabel('Amplitud (Grados)')
plt.legend()
plt.grid(True)

plt.savefig(os.path.join(dir_path, 'grafica_completa_valles.png'), bbox_inches='tight')
plt.show()