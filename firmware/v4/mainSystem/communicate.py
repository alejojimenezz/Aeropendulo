import serial
import serial.tools.list_ports
import struct
import csv
import time
from datetime import datetime
import threading
import sys
import os

PUERTO = 'COM10'     # Puerto por defecto. Tambien: python communicate.py COM5
BAUD_RATE = 921600   

def elegir_puerto():
    """Usa el puerto pedido (argumento o PUERTO); si no existe, lista los disponibles."""
    pedido = sys.argv[1] if len(sys.argv) > 1 else PUERTO
    disponibles = list(serial.tools.list_ports.comports())
    nombres = [p.device for p in disponibles]
    if pedido in nombres:
        return pedido
    print(f"\n[!] El puerto {pedido} no esta disponible.")
    if not disponibles:
        print("    No hay ningun puerto serie. Revisa el cable USB (de datos) y que el ESP32 este conectado.")
        sys.exit(1)
    print("    Puertos disponibles:")
    for p in disponibles:
        print(f"      {p.device}: {p.description}")
    if len(disponibles) == 1:
        print(f"    Usando {disponibles[0].device} (es el unico disponible).\n")
        return disponibles[0].device
    print("    Ejecuta de nuevo indicando el puerto, por ejemplo: python communicate.py " + nombres[0])
    sys.exit(1)

def enviar_comandos(esp32_serial):
    while True:
        try:
            comando = input()
            if comando:
                comando_bytes = (comando + '\n').encode('ascii')
                esp32_serial.write(comando_bytes)
                sys.stdout.write(f"\033[A\r> Comando ejecutado: {comando.upper()} " + " "*20 + "\n")
                sys.stdout.flush()
        except EOFError:
            break
        except Exception:
            break

fecha_hora = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
nombre_base = f"registro_datos_{fecha_hora}.csv"
directorio_script = os.path.dirname(os.path.abspath(__file__))
nombre_archivo = os.path.join(directorio_script, nombre_base)

cabeceras_csv = [
    "Timestamp_us", "PWM_Porcentaje", "Angulo_Grados", 
    "Acc_X", "Acc_Y", "Acc_Z", 
    "Gyr_X", "Gyr_Y", "Gyr_Z", "Referencia", "Estado"
]

# CSV del experimento (solo rampa/escalon): tiempo relativo desde la 1ra muestra
cabeceras_exp = ["Tiempo_s"] + cabeceras_csv[:-1]
NOMBRES_ESTADO = {1: "escalon", 2: "rampa", 3: "rampa_desc", 4: "rampa_ascdesc"}

# Estado del experimento en curso
archivo_exp = None
escritor_exp = None
ts_inicio_exp = 0
nombre_exp = ""
muestras_exp = 0
contador_exp = 0

def abrir_experimento(estado, timestamp):
    global archivo_exp, escritor_exp, ts_inicio_exp, nombre_exp, muestras_exp, contador_exp
    contador_exp += 1
    fecha = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    etiqueta = NOMBRES_ESTADO.get(estado, f"perfil{estado}")
    nombre_exp = os.path.join(directorio_script, f"{etiqueta}_{contador_exp:02d}_{fecha}.csv")
    archivo_exp = open(nombre_exp, mode='w', newline='')
    escritor_exp = csv.writer(archivo_exp)
    escritor_exp.writerow(cabeceras_exp)
    ts_inicio_exp = timestamp
    muestras_exp = 0
    sys.stdout.write(f"\n[EXP] Inicio de {etiqueta} -> {os.path.basename(nombre_exp)}\n")

def cerrar_experimento():
    global archivo_exp, escritor_exp
    if archivo_exp is not None:
        archivo_exp.flush()
        archivo_exp.close()
        sys.stdout.write(f"\n[EXP] Fin. {muestras_exp} muestras guardadas en {os.path.basename(nombre_exp)}\n")
    archivo_exp = None
    escritor_exp = None

paquetes_recibidos = 0
tiempo_inicio = None
puerto = elegir_puerto()

try:
    esp32 = serial.Serial(puerto, BAUD_RATE)
    time.sleep(0.5) 
    print(f"Conectado a {puerto} a {BAUD_RATE} baudios.")
    
    with open(nombre_archivo, mode='w', newline='') as archivo_csv:
        escritor = csv.writer(archivo_csv)
        escritor.writerow(cabeceras_csv) 
        
        print(f"\n[!] Guardando telemetría en:\n    -> {nombre_archivo}")
        print("Presiona Ctrl+C para detener todo y guardar el archivo de forma segura.\n")
        print("="*48)
        print("      MENÚ DE CONTROL Y PID (SyncMenu)")
        print("="*48)
        print(" K k,ti,td    -> PID Global   (Ej: K2.5,0.5,0.01)")
        print(" K º,k,ti,td  -> PID en Ángulo(Ej: K90,3.5,0.3,0.05)")
        print(" P xx,yy      -> PID: Ir a xx° con yy% PWM (Ej: P90,50)")
        print(" R paso,t_ms[,max] -> RAMPA ascendente:  sube paso% cada t_ms ms (Ej: R5,2000 o R5,2000,80)")
        print(" D paso,t_ms[,max] -> RAMPA descendente: baja de max a paso (Ej: D5,2000)")
        print(" T paso,t_ms[,max] -> RAMPA sube y baja: asciende y luego desciende (Ej: T5,2000)")
        print(" Hxx / Axx    -> MANUAL: Horario/Antihorario xx%")
        print(" S / E        -> Stop Suave / Paro de Emergencia")
        print(" Z            -> Calibrar Ceros de Sensores")
        print("="*48 + "\n")
        
        hilo_teclado = threading.Thread(target=enviar_comandos, args=(esp32,), daemon=True)
        hilo_teclado.start()
        
        paquetes_recibidos = 0
        tiempo_inicio = time.time()
        
        while True:
            if esp32.read(1) == b'\xAA' and esp32.read(1) == b'\xBB':
                raw_data = esp32.read(42)
                
                if len(raw_data) == 42:
                    datos = struct.unpack('<I9fBB', raw_data)
                    timestamp, pwm, angulo = datos[0], datos[1], datos[2]
                    ax, ay, az = datos[3], datos[4], datos[5]
                    gx, gy, gz = datos[6], datos[7], datos[8]
                    ref = datos[9]
                    estado = datos[10]
                    footer = datos[11]
                    
                    if footer == 0xCC:
                        fila = [timestamp, pwm, angulo, ax, ay, az, gx, gy, gz, ref]
                        escritor.writerow(fila + [estado])
                        paquetes_recibidos += 1
                        if paquetes_recibidos % 50 == 0:
                            archivo_csv.flush()  # permite al visualizador leer casi en tiempo real

                        # --- CSV de experimento: sincronizado con el perfil del ESP32 ---
                        if estado != 0 and archivo_exp is None:
                            abrir_experimento(estado, timestamp)
                        elif estado == 0 and archivo_exp is not None:
                            cerrar_experimento()

                        if archivo_exp is not None:
                            t_rel = ((timestamp - ts_inicio_exp) & 0xFFFFFFFF) / 1e6  # tolera overflow de micros()
                            escritor_exp.writerow([f"{t_rel:.6f}"] + fila)
                            muestras_exp += 1
                        
                        if paquetes_recibidos % 200 == 0:
                            # Visualización actualizada con el eje Z del giroscopio
                            sys.stdout.write(f"\r [Grabando...] Ang: {angulo:6.2f}° | GyroZ: {gz:6.2f} rad/s | PWM: {pwm:5.1f}% | Muestras: {paquetes_recibidos}   ")
                            sys.stdout.flush()
                    else:
                        sys.stdout.write("\n[Advertencia] Trama corrupta omitida.\n")
                        
except KeyboardInterrupt:
    print("\n\nGrabación detenida por el usuario.")
except serial.SerialException as e:
    print(f"\nError de puerto serial: {e}")
finally:
    cerrar_experimento()
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()
    if tiempo_inicio is None:
        print("\nNo se inicio la captura (sin conexion con el ESP32); no se genero ningun archivo.")
    else:
        tiempo_total = time.time() - tiempo_inicio
        frecuencia_aprox = paquetes_recibidos / tiempo_total if tiempo_total > 0 else 0
        print("\n" + "=" * 40)
        print(" RESUMEN DE CAPTURA")
        print("=" * 40)
        print(f" Archivo guardado en:\n {nombre_archivo}")
        print(f" Total de muestras: {paquetes_recibidos}")
        print(f" Frecuencia promedio: {frecuencia_aprox:.2f} Hz")
        print("=" * 40)
    os._exit(0)
