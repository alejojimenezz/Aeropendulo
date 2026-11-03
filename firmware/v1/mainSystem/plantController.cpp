#include "plantController.h"

plantController::plantController(plantStatus* plantaPtr) {
    _planta = plantaPtr;
    _currentMode = CTRL_NONE;
    _Kp = 1.0; _Ki = 0.0; _Kd = 0.0; 
    _errorPrev = 0; 
    _integral = 0;
    
    // Tiempo de muestreo matemático fundamental (Ts = 10ms -> 100Hz)
    _Ts = 0.01f; 

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
    float opAngle = currentAngle;
    
    // 1. Límite máximo en 120 grados
    if (opAngle > 120.0f) opAngle = 120.0f;
    if (opAngle < 0.0f) opAngle = 0.0f; 
    
    // 2. Efecto Espejo (Mirror) para mayores de 90
    if (opAngle > 90.0f) {
        opAngle = 180.0f - opAngle;
    }

    // 3. Interpolación Lineal Equiespaciada
    float K_dyn, Ti_dyn, Td_dyn;
    int idx = (int)(opAngle / 10.0f); // Índice inferior
    
    if (idx >= 9) {
        K_dyn = _gainTable[9].K;
        Ti_dyn = _gainTable[9].Ti;
        Td_dyn = _gainTable[9].Td;
    } else {
        // Fracción entre los dos índices
        float t = (opAngle - (idx * 10.0f)) / 10.0f;
        K_dyn  = _gainTable[idx].K  + t * (_gainTable[idx+1].K  - _gainTable[idx].K);
        Ti_dyn = _gainTable[idx].Ti + t * (_gainTable[idx+1].Ti - _gainTable[idx].Ti);
        Td_dyn = _gainTable[idx].Td + t * (_gainTable[idx+1].Td - _gainTable[idx].Td);
    }

    // 4. Actualizar ganancias finales a Forma Paralela
    _Kp = K_dyn;
    _Ki = (Ti_dyn > 0.001f) ? (K_dyn / Ti_dyn) : 0.0f;
    _Kd = K_dyn * Td_dyn;
}

void plantController::setTarget(float targetAngle, float maxSpeedPWM, ControlMode mode) {
    _setpointAngle = targetAngle;
    _maxSpeedPWM = (maxSpeedPWM > 100.0f) ? 100.0f : maxSpeedPWM;
    _errorPrev = _setpointAngle - _planta->angulo_as5600;
    _integral = 0; 
    _lastTime = micros();
    
    // Se guarda el modo de control que solicitó el usuario/menú
    _currentMode = mode;
}

void plantController::stop() {
    _currentMode = CTRL_NONE;
}

// =========================================================
// DESPACHADOR MAESTRO
// =========================================================
void plantController::update() {
    if (_currentMode == CTRL_NONE) return;

    // Llama al método correspondiente según el modo indicado en setTarget
    if (_currentMode == CTRL_PID_POSITION) {
        PIDcontroller();
    }
    else if (_currentMode == CTRL_FUTURE_OTHER) {
        // futuroControlador();
    }
}

// =========================================================
// LÓGICA DE CONTROLADORES
// =========================================================
void plantController::PIDcontroller() {
    uint32_t currentTime = micros();
    float dt = (currentTime - _lastTime) / 1000000.0f; 
    
    // 1. FORZAR TIEMPO DE MUESTREO MATEMÁTICO (Ts)
    if (dt < _Ts) {
        return; // Salimos si aún no se cumple el tiempo de muestreo exacto
    }
    // Límite de seguridad: si el loop se cuelga por más de 10 veces el Ts, reseteamos el temporizador
    if (dt > (_Ts * 10.0f)) {
        _lastTime = currentTime; 
        return; 
    }
    _lastTime = currentTime;

    // --- LÍMITE DE SEGURIDAD ANTI-ENROSCADO ---
    // Si el brazo pasa de 135 grados, apagamos el motor inmediatamente.
    // Se usa < 300.0f para ignorar el "rebote" cerca de los 0 grados (ej. 359°).
    if (_planta->angulo_as5600 > 135.0f && _planta->angulo_as5600 < 300.0f) {
        _planta->applyMotor(0.0, 0, false);
        _integral = 0.0f; // Purgar el integrador para que no guarde memoria del error.
        return; // Salir de la función para que caiga libremente.
    }
    // ------------------------------------------------

    // Calcular las ganancias basándose en el ángulo en el que está el brazo AHORA
    calcDynGains(_planta->angulo_as5600);

    float error = _setpointAngle - _planta->angulo_as5600;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    float P = error;

    if (_Ki > 0.0001f) { 
        _integral += error * dt;
        
        // 2. LIMITAR EL PESO DE LA INTEGRAL (Anti-Windup ajustado)
        // Permitimos que la integral aporte hasta el 85% de la fuerza total
        float max_integral = (_maxSpeedPWM * 0.85f) / _Ki; 
        if (_integral > max_integral) _integral = max_integral;
        if (_integral < -max_integral) _integral = -max_integral; 
    } else {
        _integral = 0.0f;
    }

    float D = (error - _errorPrev) / dt;
    _errorPrev = error;

    float u = (_Kp * P) + (_Ki * _integral) + (_Kd * D);

    // Lógica Aeroplanta (Cuadrante único)
    float duty = 0.0f;
    int direction = 0;

    if (u > 0.01f) {
        duty = u; 
        if (duty > _maxSpeedPWM) duty = _maxSpeedPWM; 
        direction = -1; // Empuje
    } else {
        duty = 0.0f;
        direction = 0;  // Caída libre
    }

    _planta->applyMotor(duty, direction, false);
}