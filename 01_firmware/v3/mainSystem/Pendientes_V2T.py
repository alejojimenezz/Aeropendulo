import numpy as np
import matplotlib.pyplot as plt

# 1. Definir los datos
T_lut = np.array([
    0, 0, 0, 0, 0, 0.0000, 0.0003, 0.0006, 0.0010, 0.0015,
    0.0027, 0.0033, 0.0044, 0.0053, 0.0064, 0.0070, 0.0081,
    0.0090, 0.0098, 0.0104, 0.0112, 0.0125, 0.0134, 0.0143,
    0.0155, 0.0156, 0.0159, 0.0163, 0.0165
])

V_lut = np.array([
    0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
    1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9,
    2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8
])

# 2. Crear máscaras para filtrar los datos en los dos tramos
mask1 = (V_lut >= 1.0) & (V_lut <= 2.4)
mask2 = (V_lut >= 2.4) & (V_lut <= 2.8)

# 3. Calcular la regresión lineal (polinomio de grado 1) para cada tramo
# np.polyfit devuelve un arreglo con [pendiente, intercepto]
coef1 = np.polyfit(V_lut[mask1], T_lut[mask1], 1)
coef2 = np.polyfit(V_lut[mask2], T_lut[mask2], 1)

m1, b1 = coef1  # Pendiente e intercepto tramo 1
m2, b2 = coef2  # Pendiente e intercepto tramo 2

print(f"Pendiente de regresión (1.0V a 2.4V): {m1:.5f}")
print(f"Pendiente de regresión (2.4V a 2.8V): {m2:.5f}")

# 4. Graficar los resultados
plt.figure(figsize=(10, 6))

# Trazar datos originales
plt.plot(V_lut, T_lut, 'ko-', label='Datos Originales (T_lut)', markersize=5)

# Crear puntos X para trazar las líneas de regresión
V_line1 = np.linspace(1.0, 2.4, 50)
V_line2 = np.linspace(2.4, 2.8, 50)

# Trazar las rectas calculadas (y = mx + b)
plt.plot(V_line1, m1 * V_line1 + b1, 'r-', linewidth=2.5, label=f'Regresión 1 (m={m1:.5f})')
plt.plot(V_line2, m2 * V_line2 + b2, 'b-', linewidth=2.5, label=f'Regresión 2 (m={m2:.5f})')

# Personalizar gráfica
plt.xlabel('Voltaje (V_lut) [V]', fontsize=12)
plt.ylabel('T_lut', fontsize=12)
plt.title('Regresión Lineal por Tramos sobre Datos LUT', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=11)

# Mostrar gráfica
plt.tight_layout()
plt.show()