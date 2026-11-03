#include "syncMenu.h"

// Actualizado para recibir _controlador[cite: 23]
SyncMenu::SyncMenu(plantStatus* plantaPtr, plantController* ctrlPtr) {
    _planta = plantaPtr;
    _controlador = ctrlPtr;
    _lastTxTime = 0;
}

void SyncMenu::processSerialCommands() {
    static String inputBuffer = ""; 
    
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                executeCommand(inputBuffer);
                inputBuffer = ""; 
            }
        } else {
            inputBuffer += c; 
        }
    }
}

void SyncMenu::executeCommand(String input) {
    input.trim();
    char cmd = input.charAt(0);
    
    if (cmd == 'Z' || cmd == 'z') {
        _planta->calibrateSensors();
    }
    else if (cmd == 'S' || cmd == 's') { 
        // Stop limpio, un solo llamado al hardware[cite: 23]
        _controlador->stop(); 
        _planta->stopProfile(); 
        _planta->applyMotor(0.0, 0, false);
    }
    else if (cmd == 'E' || cmd == 'e') { 
        // Emergencia limpio[cite: 23]
        _controlador->stop(); 
        _planta->stopProfile(); 
        _planta->applyMotor(0.0, 0, true);
    }
    else if (cmd == 'P' || cmd == 'p') { 
        _planta->stopProfile(); 
        int comaIdx = input.indexOf(',');
        float target = 0;
        float maxPWM = 100.0;
        if (comaIdx != -1) {
            target = input.substring(1, comaIdx).toFloat();
            maxPWM = input.substring(comaIdx + 1).toFloat();
        } else {
            target = input.substring(1).toFloat();
        }
        // Llamado con el tipo de controlador explícito[cite: 23]
        _controlador->setTarget(target, maxPWM, CTRL_PID_POSITION);
    }
    else if (cmd == 'H' || cmd == 'h' || cmd == 'A' || cmd == 'a') {
        _controlador->stop(); 
        _planta->stopProfile(); 
        if (input.length() > 1) {
            float pwr = input.substring(1).toFloat();
            int dir = (cmd == 'H' || cmd == 'h') ? 1 : -1;
            _planta->applyMotor(pwr, dir, false);
        }
    }
    else if (cmd == 'R' || cmd == 'r') {
        _controlador->stop(); 
        _planta->stopProfile(); 
        
        // Parsear el comando con 4 parámetros separados por comas
        // Formato esperado: R[startDuty],[endDuty],[duration_ms],[stepSize]
        // Ejemplo: R0,20,10000,5
        int c1 = input.indexOf(',');
        int c2 = input.indexOf(',', c1 + 1);
        int c3 = input.indexOf(',', c2 + 1);

        if (c1 != -1 && c2 != -1 && c3 != -1) {
            float startDuty = input.substring(1, c1).toFloat();
            float endDuty   = input.substring(c1 + 1, c2).toFloat();
            uint32_t duration = input.substring(c2 + 1, c3).toInt();
            float stepSize  = input.substring(c3 + 1).toFloat();

            // Dirección -1 corresponde a "Empuje" en la lógica del aeropéndulo
            _planta->startRamp(startDuty, endDuty, -1, duration, stepSize);
        }
    }
    else if (cmd == 'K' || cmd == 'k') {
        int c1 = input.indexOf(',');
        int c2 = input.indexOf(',', c1 + 1);
        int c3 = input.indexOf(',', c2 + 1);

        if (c1 != -1 && c2 != -1 && c3 != -1) {
            // Formato de 4 parámetros: K angulo, K, Ti, Td (Ej: K90,2.5,0.5,0.01)[cite: 23]
            int angulo = input.substring(1, c1).toInt();
            float k_val  = input.substring(c1 + 1, c2).toFloat();
            float ti_val = input.substring(c2 + 1, c3).toFloat();
            float td_val = input.substring(c3 + 1).toFloat();
            _controlador->setTablePID(angulo, k_val, ti_val, td_val);
        } 
        else if (c1 != -1 && c2 != -1 && c3 == -1) {
            // Formato de 3 parámetros: K K, Ti, Td (Ej: K2.5,0.5,0.01)[cite: 23]
            float k_val  = input.substring(1, c1).toFloat();
            float ti_val = input.substring(c1 + 1, c2).toFloat();
            float td_val = input.substring(c2 + 1).toFloat();
            // Esto escribe el mismo valor en TODOS los 10 espacios de la tabla[cite: 23]
            _controlador->setGlobalPID(k_val, ti_val, td_val); 
        }
    }
}

void SyncMenu::transmitData(uint32_t frequencyHz) {
    uint32_t interval = 1000000 / frequencyHz; 
    uint32_t currentMicros = micros();

    if (currentMicros - _lastTxTime >= interval) {
        _lastTxTime = currentMicros;

        SensorDataPacket packet;
        packet.header[0] = 0xAA; 
        packet.header[1] = 0xBB;
        packet.timestamp = currentMicros;
        packet.pwm = _planta->currentDuty;
        packet.angulo = _planta->angulo_as5600;
        packet.ax = _planta->ax;
        packet.ay = _planta->ay;
        packet.az = _planta->az;
        packet.gx = _planta->gx;
        packet.gy = _planta->gy;
        packet.gz = _planta->gz;
        packet.footer = 0xCC;

        Serial.write((uint8_t*)&packet, sizeof(packet));
    }
}