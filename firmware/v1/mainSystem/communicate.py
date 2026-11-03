import serial
import struct
import csv
import time
from datetime import datetime
import threading
import sys
import os

# ==========================================
# CONFIGURACIÓN DEL PUERTO SERIAL
# ==========================================
PUERTO = 'COM8'      # <- CAMBIA ESTO POR TU PUERTO
BAUD_RATE = 921600   

# ==========================================
# HILO 1: TRANSMISIÓN DE COMANDOS (SyncMenu)
# ==========================================
def enviar_comandos(esp32_serial):
    # El menú ya no se imprime aquí para evitar choques de consola
    while True:
        try:
            comando = input()
            if comando:
                comando_bytes = (comando + '\n').encode('ascii')
                esp32_serial.write(comando_bytes)
                
                # Borramos la línea actual y mostramos la confirmación
                sys.stdout.write(f"\033[A\r> Comando ejecutado: {comando.upper()} " + " "*20 + "\n")
                sys.stdout.flush()
        except EOFError:
            break
        except Exception:
            break

# ==========================================
# HILO PRINCIPAL: RECEPCIÓN Y DATALOGGER
# ==========================================
fecha_hora = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
nombre_base = f"registro_datos_{fecha_hora}.csv"

directorio_script = os.path.dirname(os.path.abspath(__file__))
nombre_archivo = os.path.join(directorio_script, nombre_base)

cabeceras_csv = [
    "Timestamp_us", "PWM_Porcentaje", "Angulo_Grados", 
    "Acc_X", "Acc_Y", "Acc_Z", 
    "Gyr_X", "Gyr_Y", "Gyr_Z"
]

try:
    esp32 = serial.Serial(PUERTO, BAUD_RATE)
    time.sleep(0.5) 
    print(f"Conectado a {PUERTO} a {BAUD_RATE} baudios.")
    
    with open(nombre_archivo, mode='w', newline='') as archivo_csv:
        escritor = csv.writer(archivo_csv)
        escritor.writerow(cabeceras_csv) 
        
        # 1. IMPRIMIR TODO EL TEXTO DESDE EL HILO PRINCIPAL PRIMERO
        print(f"\n[!] Guardando telemetría en:\n    -> {nombre_archivo}")
        print("Presiona Ctrl+C para detener todo y guardar el archivo de forma segura.\n")
        
        print("="*48)
        print("      MENÚ DE CONTROL Y PID (SyncMenu)")
        print("="*48)
        print(" Escribe un comando y presiona Enter:")
        print("  K k,ti,td    -> PID Global   (Ej: K2.5,0.5,0.01)")
        print("  K º,k,ti,td  -> PID en Ángulo(Ej: K90,3.5,0.3,0.05)")
        print("  P xx,yy      -> PID: Ir a xx° con yy% PWM (Ej: P90,50)")
        print("  R...         -> RAMPA: (Ej: R0,100,10000,15)")
        print("  Hxx / Axx    -> MANUAL: Horario/Antihorario xx%")
        print("  S / E        -> Stop Suave / Paro de Emergencia")
        print("  Z            -> Calibrar Ceros de Sensores")
        print("="*48 + "\n")
        
        # 2. INICIAR EL HILO DEL TECLADO DESPUÉS DE IMPRIMIR LA CONSOLA
        hilo_teclado = threading.Thread(target=enviar_comandos, args=(esp32,), daemon=True)
        hilo_teclado.start()
        
        paquetes_recibidos = 0
        tiempo_inicio = time.time()
        
        while True:
            if esp32.read(1) == b'\xAA' and esp32.read(1) == b'\xBB':
                raw_data = esp32.read(37)
                
                if len(raw_data) == 37:
                    datos = struct.unpack('<I8fB', raw_data)
                    timestamp, pwm, angulo = datos[0], datos[1], datos[2]
                    ax, ay, az = datos[3], datos[4], datos[5]
                    gx, gy, gz = datos[6], datos[7], datos[8]
                    footer = datos[9]
                    
                    if footer == 0xCC:
                        escritor.writerow([timestamp, pwm, angulo, ax, ay, az, gx, gy, gz])
                        paquetes_recibidos += 1
                        
                        if paquetes_recibidos % 200 == 0:
                            sys.stdout.write(f"\r [Grabando...] Angulo: {angulo:6.2f}° | PWM: {pwm:5.1f}% | Muestras: {paquetes_recibidos}")
                            sys.stdout.flush()
                    else:
                        sys.stdout.write("\n[Advertencia] Trama corrupta omitida.\n")
                        
except KeyboardInterrupt:
    print("\n\nGrabación detenida por el usuario.")
    
except serial.SerialException as e:
    print(f"\nError de puerto serial: {e}")
    print("Revisa que el cable esté conectado y el Arduino IDE cerrado.")
    
finally:
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()
        
    tiempo_total = time.time() - tiempo_inicio if 'tiempo_inicio' in locals() else 0
    frecuencia_aprox = paquetes_recibidos / tiempo_total if tiempo_total > 0 else 0
    
    print("\n" + "=" * 40)
    print(" RESUMEN DE CAPTURA")
    print("=" * 40)
    print(f" Archivo guardado en:\n {nombre_archivo}")
    print(f" Total de muestras: {paquetes_recibidos}")
    print(f" Frecuencia promedio: {frecuencia_aprox:.2f} Hz")
    print("=" * 40)
    
    # 3. FORZAR CIERRE LIMPIO DEL SISTEMA PARA EVITAR ERROR DE DAEMON THREADS
    os._exit(0)   