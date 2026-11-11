#include "syncMenu.h"

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
        _controlador->stop();               // Detiene el PID o la Secuencia activa
        _planta->stopProfile();             // Detiene cualquier rampa en curso
        _planta->applyMotor(0.0, 0, false); // Apaga el motor para evitar movimientos
        _planta->calibrateSensors();        // Calcula el nuevo cero con el sistema en reposo
    }
    else if (cmd == 'S' || cmd == 's') { 
        _controlador->stop(); 
        _planta->stopProfile(); 
        _planta->applyMotor(0.0, 0, false);
    }
    else if (cmd == 'E' || cmd == 'e') { 
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
        _controlador->setTarget(target, maxPWM, CTRL_PID_POSITION);
    }
    else if (cmd == 'F' || cmd == 'f') { 
        // Formato: F<ang1>,<ang2>,...,<angN>,<t_ms>   (max. 20 referencias)
        // Ej: F20,30,50,70,10,2000 -> 20°,30°,50°,70°,10° con 2000 ms en cada una
        _planta->stopProfile(); 
        
        float angles[20];
        int count = 0;
        int startIdx = 1;
        int commaIdx = input.indexOf(',', startIdx);
        
        // Extraemos todos los ángulos separados por comas
        while (commaIdx != -1 && count < 20) {
            angles[count] = input.substring(startIdx, commaIdx).toFloat();
            count++;
            startIdx = commaIdx + 1;
            commaIdx = input.indexOf(',', startIdx);
        }
        
        // El último valor que queda después de la última coma es el tiempo.
        // Si aún quedaban comas (commaIdx != -1) hubo más de 20 referencias: se rechaza.
        if (commaIdx == -1 && count > 0 && startIdx < (int)input.length()) {
            uint32_t t_ms = input.substring(startIdx).toInt();
            float maxPWM = 100.0; // Se asume velocidad máxima para la secuencia
            
            _controlador->setSequence(angles, count, t_ms, maxPWM); // ignora t_ms = 0
        }
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
    else if (cmd == 'R' || cmd == 'r' || cmd == 'D' || cmd == 'd' || cmd == 'T' || cmd == 't') {
        _controlador->stop(); 
        _planta->stopProfile(); 
        
        // Formato: R|D|T<paso%>,<t_escalon_ms>[,<pwm_max%>]
        //   R = rampa ascendente, D = descendente, T = sube y baja (triangular)
        MotorProfile tipo = PROFILE_RAMP;
        if (cmd == 'D' || cmd == 'd') tipo = PROFILE_RAMP_DOWN;
        else if (cmd == 'T' || cmd == 't') tipo = PROFILE_RAMP_UPDOWN;

        int c1 = input.indexOf(',');
        int c2 = input.indexOf(',', c1 + 1);

        if (c1 != -1) {
            float stepSize     = input.substring(1, c1).toFloat();
            uint32_t stepTime  = (c2 != -1) ? input.substring(c1 + 1, c2).toInt()
                                            : input.substring(c1 + 1).toInt();
            float maxDuty      = (c2 != -1) ? input.substring(c2 + 1).toFloat() : 100.0f;
            _planta->startRamp(stepSize, stepTime, maxDuty, -1, tipo);
        }
    }
    else if (cmd == 'K' || cmd == 'k') {
        int c1 = input.indexOf(',');
        int c2 = input.indexOf(',', c1 + 1);
        int c3 = input.indexOf(',', c2 + 1);

        if (c1 != -1 && c2 != -1 && c3 != -1) {
            int angulo = input.substring(1, c1).toInt();
            float k_val  = input.substring(c1 + 1, c2).toFloat();
            float ti_val = input.substring(c2 + 1, c3).toFloat();
            float td_val = input.substring(c3 + 1).toFloat();
            _controlador->setTablePID(angulo, k_val, ti_val, td_val);
        } 
        else if (c1 != -1 && c2 != -1 && c3 == -1) {
            float k_val  = input.substring(1, c1).toFloat();
            float ti_val = input.substring(c1 + 1, c2).toFloat();
            float td_val = input.substring(c2 + 1).toFloat();
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
        
        // Transmisión de datos del giroscopio
        packet.gx = _planta->gx;
        packet.gy = _planta->gy;
        packet.gz = _planta->gz;
        
        packet.ref = _controlador->getSetpoint();
        // Estado: perfil de motor (1=escalon, 2/3/4=rampas) o secuencia de referencias PID (5)
        uint8_t estado = (uint8_t)_planta->getProfile();
        if (estado == 0 && _controlador->isSequenceActive()) estado = ESTADO_SECUENCIA;
        packet.estado = estado;
        
        packet.footer = 0xCC;

        Serial.write((uint8_t*)&packet, sizeof(packet));
    }
}