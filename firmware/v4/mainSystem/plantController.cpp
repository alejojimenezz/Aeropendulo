#include "plantController.h"
#include <math.h> // Para funciones sin(), fabs() y constantes como PI

plantController::plantController(plantStatus* plantaPtr) {
    _planta = plantaPtr;
    _currentMode = CTRL_NONE;
    _Kp = 1.0; _Ki = 0.0; _Kd = 0.0; 
    _errorPrev = 0; 
    _integral = 0;
    
    // Tiempo de muestreo matemático fundamental (Ts = 10ms -> 100Hz)
    _Ts = 0.004f; 

    // Variables físicas medibles
    _plantVoltage = 5.3f;
    _J = 4.4823e-04f;
    _m1 = 0.0120f;
    _m2 = 0.0120f;
    _l1 = 0.1590f;
    _l2 = 0.0680f;
    _b = 0.000675f;
    _mgl = 0.0267f;

    // Pendientes predefinidas y valores LUT
    _slopeTorque1 = 0.0121f;//0.008107f;//0.00901f;
    _slopeTorque2 = 0.00270f;
    _b_mot = -0.0081f;//-0.0065//-0.002372f;
    // Inicializar la tabla con valores por defecto
    setGlobalPID(1.0, 0.0, 0.0);
}

void plantController::setSamplingTime(float Ts_seconds) {
    if (Ts_seconds > 0.0f) {
        _Ts = Ts_seconds;
    }
}

void plantController::setGlobalPID(float K, float Ti, float Td) {
    for (int i = 0; i < 10; i++) {
        _gainTable[i] = {K, Ti, Td};
    }
    Serial.println("Tabla PID sobrescrita globalmente.");
}

void plantController::setTablePID(int targetAngle, float K, float Ti, float Td) {
    int index = targetAngle / 10;
    if (index >= 0 && index <= 9) {
        _gainTable[index] = {K, Ti, Td};
        Serial.print("Ganancia guardada para ángulo "); Serial.print(index * 10); Serial.println("°");
    }
}

void plantController::calcDynGains(float currentAngle) {
    // Escoger la ganancia según la decena del ángulo actual
    int index = (int)(currentAngle / 10.0f);
    if (index < 0) index = 0;
    if (index > 9) index = 9;
    
    _Kp = _gainTable[index].K;
    _Ki = _gainTable[index].Ti;
    _Kd = _gainTable[index].Td;
}

void plantController::setTarget(float targetAngle, float maxSpeedPWM, ControlMode mode) {
    _setpointAngle = targetAngle;
    _maxSpeedPWM = (maxSpeedPWM > 100.0f) ? 100.0f : maxSpeedPWM;
    _errorPrev = _setpointAngle - _planta->angulo_as5600;
    _integral = 0.0f;
    _lastTime = micros();
    _currentMode = mode;
}

void plantController::setSequence(float* angles, int count, uint32_t timePerRefMs, float maxSpeedPWM) {
    if (count <= 0 || timePerRefMs == 0) return; // Secuencia invalida: se ignora
    _seqCount = (count > 20) ? 20 : count; // Protegemos la memoria (máximo 20 pasos)
    for(int i = 0; i < _seqCount; i++) {
        _seqAngles[i] = angles[i];
    }
    _seqTimeMs = timePerRefMs;
    _maxSpeedPWM = (maxSpeedPWM > 100.0f) ? 100.0f : maxSpeedPWM;
    _seqIndex = 0;
    
    // Inicializamos el PID con el primer ángulo
    _setpointAngle = _seqAngles[0];
    _errorPrev = _setpointAngle - _planta->angulo_as5600;
    _integral = 0.0f;
    
    _lastSeqTimeMs = millis();
    _lastTime = micros(); // Para el delta de tiempo del PID
    _currentMode = CTRL_SEQUENCE;
}

void plantController::stop() {
    _currentMode = CTRL_NONE;
}

void plantController::update() {
    if (_currentMode == CTRL_NONE) return;

    if (_currentMode == CTRL_SEQUENCE) {
        // Verificamos si ya pasó el tiempo de la referencia actual.
        // Se suma _seqTimeMs (en vez de millis()) para no acumular deriva entre pasos.
        if (millis() - _lastSeqTimeMs >= _seqTimeMs) {
            _lastSeqTimeMs += _seqTimeMs;
            if (_seqIndex < _seqCount - 1) {
                _seqIndex++;
                _setpointAngle = _seqAngles[_seqIndex];
            } else {
                // Terminó la última referencia: fin de la secuencia (la telemetría vuelve a estado 0).
                // El PID sigue sosteniendo el último ángulo para que la planta no caiga;
                // se detiene con S / E como siempre.
                _currentMode = CTRL_PID_POSITION;
            }
        }
        PIDcontroller();
    } 
    else if (_currentMode == CTRL_PID_POSITION) {
        PIDcontroller();
    }
}

// =========================================================
// FUNCIONES PREVIAS AL PWM PARA RELACIONAR MAGNITUDES
// =========================================================
float plantController::PWM2Voltage(float duty_PWM) {
    return (duty_PWM / 100.0f) * _plantVoltage;
}

float plantController::Voltage2PWM(float voltage) {
    return (voltage / _plantVoltage) * 100.0f;
}

float plantController::Voltage2Torque(float voltage) {
    float v_abs = fabs(voltage);
    float torque = 0.0f;
    if (v_abs <= 2.4f) {
        torque = v_abs * _slopeTorque1;
    } else {
        // Corrección: Función lineal continua (y = mx + b)
        // Sumamos el torque límite de la primera fase (2.4 * slope1) al exceso de voltaje
        torque = (2.4f * _slopeTorque1) + _b_mot; //+ ((v_abs - 2.4f) * _slopeTorque2);
    }
    return (voltage < 0.0f) ? -torque : torque; // Mantiene el sentido de empuje
}

float plantController::Torque2Voltage(float torque) {
    float t_abs = fabs(torque);
    float voltage = 0.0f;
    float umbral_torque = 2.4f * _slopeTorque1; // Aprox 0.021624 Nm

    // Se asume simetría para giro horario/antihorario según magnitud
    if (t_abs <= umbral_torque) {
        voltage = t_abs / _slopeTorque1;
    } else {
        // Corrección: Inversa continua de la función de arriba
        voltage = 2.4f + ((t_abs - umbral_torque) / _slopeTorque2);
    }
    return (torque < 0.0f) ? -voltage : voltage;
}
// =========================================================
// LÓGICA DE CONTROLADORES
// =========================================================
void plantController::PIDcontroller() {
    uint32_t currentTime = micros();
    float dt = (currentTime - _lastTime) / 1000000.0f; 
    
    // Si el tiempo es ridículamente pequeño o negativo, omitimos el cálculo.
    // Si es demasiado grande (se colgó el sistema), lo limitamos para evitar windup masivo.
    if (dt <= 0.0001f) return;
    if (dt > 0.05f) dt = 0.05f; 
    
    _lastTime = currentTime;

    if (_planta->angulo_as5600 > 135.0f && _planta->angulo_as5600 < 300.0f) {
        _planta->applyMotor(0.0, 0, false);
        _integral = 0.0f; 
        return; 
    }
    
    calcDynGains(_planta->angulo_as5600);

    float error = _setpointAngle - _planta->angulo_as5600;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    float error_rad = error * (PI / 180.0f);
    float theta_rad = _planta->angulo_as5600 * (PI / 180.0f);
    
    // Filtro del Giroscopio
    static float filtered_gz = 0.0f;
    float alpha = 0.71f; 
    filtered_gz = (alpha * (-_planta->gz)) + ((1.0f - alpha) * filtered_gz);
    float theta_dot = filtered_gz; 

    // Lógica Integral usando el 'dt' dinámico
    if (_Ki > 0.0001f) { 
        _integral += error_rad * dt;
        
        float max_voltage = PWM2Voltage(_maxSpeedPWM);
        float max_torque = Voltage2Torque(max_voltage);
        float max_integral = (max_torque * 2.5f) / (_J * _Ki); 
        
        if (_integral > max_integral) _integral = max_integral;
        if (_integral < -max_integral) _integral = -max_integral; 
    } else {
        _integral = 0.0f;
    }

    // Ecuación de Torque
    float term_PID = _J * ((_Kp * error_rad) + (_Ki * _integral) - (_Kd * theta_dot));
    float comp_viscosa = _b * theta_dot;
    float comp_gravedad = _mgl * sin(theta_rad);

    float tau_total = term_PID + comp_viscosa + comp_gravedad;

    // Traducción final a Hardware
    float required_voltage = Torque2Voltage(tau_total);
    float duty_calc = Voltage2PWM(required_voltage);

    if (duty_calc > _maxSpeedPWM) duty_calc = _maxSpeedPWM;
    if (duty_calc < -_maxSpeedPWM) duty_calc = -_maxSpeedPWM;

    float duty = fabs(duty_calc);
    int direction = (duty_calc >= 0.0f) ? -1 : 1; 

    _planta->applyMotor(duty, direction, false);
}