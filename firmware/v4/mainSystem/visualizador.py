"""
Visualizador en tiempo real (lee el CSV que escribe communicate.py).

Uso:
    1) Ejecuta communicate.py (es el unico que abre el puerto COM).
    2) En otra terminal:  python visualizador.py
       Opciones:  python visualizador.py [archivo.csv] [--ventana 10]

Sin argumento toma el registro_datos_*.csv mas reciente y, si empieza una
captura nueva, cambia solo al archivo nuevo.

Grafica 1: Angulo real y Referencia (grados)
Grafica 2: PWM (%) vs tiempo
Grafica 3: Giroscopio Gyr_Z (rad/s)
"""
import argparse
import glob
import math
import os
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DIR_SCRIPT = os.path.dirname(os.path.abspath(__file__))
MAX_PUNTOS_DIBUJO = 2000      # decimacion para que la grafica siga fluida a 500 Hz
BYTES_INICIALES = 300_000     # al abrir un archivo grande, solo se lee el final
NAN = float("nan")


class LectorCSV:
    """Sigue (tail) un CSV que otro proceso esta escribiendo."""

    def __init__(self, ruta_fija=None, ventana_s=10.0):
        self.ruta_fija = ruta_fija
        self.ventana = ventana_s
        self.ruta = None
        self.f = None
        self.ultima_busqueda = 0.0
        self._vaciar()

    def _vaciar(self):
        self.t = deque()
        self.ang = deque()
        self.ref = deque()
        self.pwm = deque()
        self.gz = deque()
        self.resto = b""
        self.ts_last = None
        self.t_acum = 0.0

    def _buscar_ruta(self):
        if self.ruta_fija:
            return self.ruta_fija if os.path.exists(self.ruta_fija) else None
        archivos = glob.glob(os.path.join(DIR_SCRIPT, "registro_datos_*.csv"))
        return max(archivos, key=os.path.getmtime) if archivos else None

    def _abrir(self, ruta):
        if self.f:
            self.f.close()
        self._vaciar()
        self.f = open(ruta, "rb")
        self.ruta = ruta
        cab = self.f.readline().decode(errors="ignore").strip().split(",")
        self.n_cols = len(cab)
        idx = {nombre: i for i, nombre in enumerate(cab)}
        self.i_ts = idx.get("Timestamp_us")
        self.i_pwm = idx.get("PWM_Porcentaje")
        self.i_ang = idx.get("Angulo_Grados")
        self.i_gz = idx.get("Gyr_Z")
        self.i_ref = idx.get("Referencia")      # None en CSV antiguos
        # Archivo grande: saltar al final y descartar la linea parcial
        inicio = self.f.tell()
        tam = os.path.getsize(ruta)
        if tam - inicio > BYTES_INICIALES:
            self.f.seek(tam - BYTES_INICIALES)
            self.f.readline()

    def actualizar(self):
        """Lee lo nuevo del archivo. Devuelve True si hay datos para graficar."""
        ahora = time.time()
        if ahora - self.ultima_busqueda > 1.0:
            self.ultima_busqueda = ahora
            ruta = self._buscar_ruta()
            if ruta and ruta != self.ruta:
                self._abrir(ruta)
        if not self.f:
            return False

        datos = self.f.read()
        if datos:
            lineas = (self.resto + datos).split(b"\n")
            self.resto = lineas.pop()          # ultima linea posiblemente incompleta
            for ln in lineas:
                self._procesar(ln)
        return len(self.t) > 0

    def _procesar(self, ln):
        campos = ln.decode(errors="ignore").strip().split(",")
        if len(campos) < self.n_cols:
            return
        try:
            ts = int(float(campos[self.i_ts]))
            ang = float(campos[self.i_ang])
            pwm = float(campos[self.i_pwm])
            gz = float(campos[self.i_gz])
            ref = float(campos[self.i_ref]) if self.i_ref is not None else NAN
        except (ValueError, IndexError, TypeError):
            return

        # Tiempo continuo (tolera el desbordamiento de micros() cada ~71 min)
        if self.ts_last is not None:
            self.t_acum += ((ts - self.ts_last) & 0xFFFFFFFF) / 1e6
        self.ts_last = ts

        if ang > 180.0:                       # 359.6 -> -0.4
            ang -= 360.0
        if not math.isnan(ref) and ref > 180.0:
            ref -= 360.0

        self.t.append(self.t_acum)
        self.ang.append(ang)
        self.ref.append(ref)
        self.pwm.append(pwm)
        self.gz.append(gz)

        while self.t[-1] - self.t[0] > self.ventana:
            self.t.popleft(); self.ang.popleft(); self.ref.popleft()
            self.pwm.popleft(); self.gz.popleft()


class Visor:
    def __init__(self, lector):
        self.lector = lector
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(
            3, 1, sharex=True, figsize=(11, 9),
            gridspec_kw={"height_ratios": [3, 2, 2]})

        self.l_ang, = self.ax1.plot([], [], color="tab:blue", lw=1.6, label="Ángulo real [°]")
        self.l_ref, = self.ax1.plot([], [], "--", color="tab:green", lw=1.6, label="Referencia [°]")
        self.l_pwm, = self.ax2.plot([], [], color="tab:red", lw=1.4, label="PWM [%]")
        self.l_gz, = self.ax3.plot([], [], color="tab:purple", lw=1.2, label="Gyr_Z [rad/s]")

        self.ax1.set_ylabel("Ángulo [°]")
        self.ax2.set_ylabel("PWM [%]")
        self.ax2.set_ylim(-5, 105)
        self.ax3.set_ylabel("Gyro Z [rad/s]")
        self.ax3.set_xlabel("Tiempo [s]")
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.grid(True, linestyle="--", alpha=0.5)
        self.ax1.legend(handles=[self.l_ang, self.l_ref], loc="upper left")
        self.ax2.legend(loc="upper left")
        self.ax3.legend(loc="upper left")
        self.ax1.set_title(" ", fontsize=10)   # reserva espacio para el titulo dinamico
        self.fig.tight_layout()

    @staticmethod
    def _limites(*series, margen=0.1, minimo=1.0):
        vals = np.concatenate([s[np.isfinite(s)] for s in series if len(s)]) if series else []
        if len(vals) == 0:
            return None
        lo, hi = float(vals.min()), float(vals.max())
        pad = max((hi - lo) * margen, minimo)
        return lo - pad, hi + pad

    def actualizar(self, _frame=None):
        if not self.lector.actualizar():
            return []
        L = self.lector
        paso = max(1, len(L.t) // MAX_PUNTOS_DIBUJO)
        t = np.fromiter(L.t, float)[::paso]
        ang = np.fromiter(L.ang, float)[::paso]
        ref = np.fromiter(L.ref, float)[::paso]
        pwm = np.fromiter(L.pwm, float)[::paso]
        gz = np.fromiter(L.gz, float)[::paso]

        self.l_ang.set_data(t, ang)
        self.l_ref.set_data(t, ref)
        self.l_pwm.set_data(t, pwm)
        self.l_gz.set_data(t, gz)

        t_fin = L.t[-1]
        self.ax1.set_xlim(max(L.t[0], t_fin - L.ventana), t_fin + 0.05 * L.ventana)
        lim = self._limites(ang, ref, minimo=2.0)
        if lim:
            self.ax1.set_ylim(*lim)
        lim = self._limites(gz, minimo=0.2)
        if lim:
            self.ax3.set_ylim(*lim)

        r = L.ref[-1]
        ref_txt = "—" if math.isnan(r) else f"{r:6.1f}°"
        self.ax1.set_title(
            f"{os.path.basename(L.ruta)}   |   Ang {L.ang[-1]:6.1f}°   Ref {ref_txt}   "
            f"PWM {L.pwm[-1]:5.1f}%   GyrZ {L.gz[-1]:6.2f} rad/s", fontsize=10)
        return [self.l_ang, self.l_ref, self.l_pwm, self.l_gz]


def main():
    ap = argparse.ArgumentParser(description="Visualizador en tiempo real de la telemetría")
    ap.add_argument("archivo", nargs="?", help="CSV a seguir (por defecto, el registro_datos_*.csv más reciente)")
    ap.add_argument("--ventana", type=float, default=10.0, help="segundos visibles (default 10)")
    args = ap.parse_args()

    visor = Visor(LectorCSV(args.archivo, args.ventana))
    anim = FuncAnimation(visor.fig, visor.actualizar, interval=50, cache_frame_data=False)
    plt.show()
    return anim


if __name__ == "__main__":
    main()