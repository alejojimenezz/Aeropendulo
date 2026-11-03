#include <Arduino.h>
#include "systemConfig.h"
#include "plantStatus.h"
#include "plantController.h" 
#include "syncMenu.h" 

// ============================================================
// CONFIGURACIÓN MAESTRA DE TIEMPO DE MUESTREO (Ts)
// ============================================================
const float Ts_sec = 0.01f;                 // Ts Fundamental: 10 milisegundos (100 Hz)
const uint32_t Ts_ms = (uint32_t)(Ts_sec * 1000.0f); // Para timers en ms (10 ms)
const float FREQ_BASE = 1.0f / Ts_sec;      // Frecuencia base matemática (100 Hz)

// Multiplicadores (Productos del Ts)
// Si quieres telemetría más rápida (sobremuestreo), multiplicas la frecuencia base.
const float MULTIPLICADOR_TELEMETRIA = 5.0f; // 5 x 100Hz = 500Hz
// ============================================================

byte motorPins[] = {25, 26, 27}; 
byte stbyPin = 33;          
byte i2cPins[] = {21, 22};  
byte spiPins[] = {23, 19, 18, 5}; 
byte targetI2CAddress = 0x68; 

SystemConfig sistema(spiPins, i2cPins, motorPins, stbyPin, targetI2CAddress);
plantStatus planta(&sistema);
plantController controlador(&planta); 
SyncMenu menu(&planta, &controlador); 

void setup() {
  sistema.start();
  planta.calibrateSensors();
  
  // 1. PROPAGAR EL TIEMPO MAESTRO A LOS MÓDULOS
  controlador.setSamplingTime(Ts_sec); // Al PID en segundos para cálculo integral/derivativo
  planta.setSamplingTime(Ts_ms);       // A la planta en ms para perfiles de rampa

  // 2. Tabla PID Dinámica
// ---------------------------------------------------------
  // TABLA DE GANANCIAS (Gain Scheduling) - Versión Anti-Oscilación
  // Formato: setTablePID(angulo, Kp, Ki, Kd)
  // ---------------------------------------------------------
  
  // Rango Bajo (0°-30°): Bajamos Kp para evitar el sobreimpulso inicial y las oscilaciones largas
  controlador.setTablePID(0,  140.0, 20.0, 25.0); 
  controlador.setTablePID(10, 130.0, 20.0, 25.0); 
  controlador.setTablePID(20, 120.0, 22.0, 28.0); 
  controlador.setTablePID(30, 110.0, 25.0, 28.0); 
  controlador.setTablePID(40,  95.0, 28.0, 30.0); 
  controlador.setTablePID(50,  85.0, 30.0, 30.0); 
  controlador.setTablePID(60,  75.0, 32.0, 32.0); 
  controlador.setTablePID(70,  70.0, 35.0, 35.0); 
  controlador.setTablePID(80,  65.0, 40.0, 38.0); // Kd reducido para que no oscile 2 minutos
  controlador.setTablePID(90,  60.0, 45.0, 40.0); // Ki alto y Kd moderado para converger rápido en horizontal
}

void loop() {

  planta.updateSensors();
  
  // Se ejecuta vinculado estricamente a _Ts_ms
  planta.updateProfile(); 
  
  // Se ejecuta vinculado estrictamente a _Ts
  controlador.update(); 
  
  menu.processSerialCommands(); 
  
  // La transmisión ahora es un producto matemático directo del tiempo de muestreo
  menu.transmitData(FREQ_BASE * MULTIPLICADOR_TELEMETRIA); 
}