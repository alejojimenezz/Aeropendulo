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
  controlador.setTablePID(0,  0.7, 3.0, 0.02); 
  controlador.setTablePID(10, 0.7, 2.8, 0.03); 
  controlador.setTablePID(20, 0.8, 2.5, 0.04); 
  controlador.setTablePID(30, 0.8, 2.2, 0.06); 
  controlador.setTablePID(40, 0.9, 1.8, 0.08); 
  controlador.setTablePID(50, 1.0, 1.5, 0.10); 
  controlador.setTablePID(60, 1.1, 1.3, 0.12); 
  controlador.setTablePID(70, 1.2, 1.1, 0.14); 
  controlador.setTablePID(80, 1.4, 0.9, 0.16); 
  controlador.setTablePID(90, 1.6, 0.7, 0.18);
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