#include "plantStatus.h"

plantStatus::plantStatus(SystemConfig* sysPtr){
    _sys = sysPtr; 
    _Ts_ms = 10;
    _currentProfile = PROFILE_NONE;
    currentFreq = 20000;
    
    offset_angulo = 0; offset_ax = 0; offset_ay = 0; offset_az = 0;
    offset_gx = 0; offset_gy = 0; offset_gz = 0;
}


void plantStatus::setSamplingTime(uint32_t Ts_ms) {
    if (Ts_ms > 0) _Ts_ms = Ts_ms;
}


// ---------- Motor control ----------

void plantStatus::applyMotor(float duty, int direction, bool eStop) {
    // Saturacion del ciclo de trabajo entre 0.0% y 100.0%
    if (duty > 100.0) duty = 100.0;
    if (duty < 0.0) duty = 0.0;

    // Paro de emergencia (Freno en corto del TB6612)
    if (eStop) {
        digitalWrite(_sys->motorPins[1], HIGH); // AIN1
        digitalWrite(_sys->motorPins[2], HIGH); // AIN2
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
        return;
    }

    // Control de dirección TB6612
    if (direction == 1) { // Horario
        digitalWrite(_sys->motorPins[1], HIGH);
        digitalWrite(_sys->motorPins[2], LOW);
    } 
    else if (direction == -1) { // Antihorario
        digitalWrite(_sys->motorPins[1], LOW);
        digitalWrite(_sys->motorPins[2], HIGH);
    } 
    else { // Coast (Libre) / Stop normal
        digitalWrite(_sys->motorPins[1], LOW);
        digitalWrite(_sys->motorPins[2], LOW);
        duty = 0.0;
    }

    // Aplicar porcentaje de PWM con mcpwm
    currentDuty = duty; // Guardar para telemetría
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
}

// ---------- Lectura de Sensores ----------

void plantStatus::readAS5600() {
    Wire.beginTransmission(0x36); 
    Wire.write(0x0C); // CAMBIO IMPORTANTE: Leemos RAW_ANGLE en lugar de ANGLE para ignorar configuraciones internas pasadas
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)0x36, (uint8_t)2, true);
    
    if (Wire.available() == 2) {
        uint16_t angleData = Wire.read() << 8;
        angleData |= Wire.read();
        
        // 1. Calculamos el ángulo bruto
        float angulo_bruto = (angleData * 360.0f) / 4096.0f;
        
        // 2. Aplicamos el offset
        angulo_as5600 = angulo_bruto - offset_angulo;
        
        // 3. Forzamos un comportamiento circular estricto entre 0 y 360 grados
        if (angulo_as5600 < 0.0f) angulo_as5600 += 360.0f;
        if (angulo_as5600 >= 360.0f) angulo_as5600 -= 360.0f;
    }
}

void plantStatus::readMPU6050() {
    sensors_event_t accel, gyro, temp;
    _sys->mpu_temp->getEvent(&temp);
    _sys->mpu_accel->getEvent(&accel);
    _sys->mpu_gyro->getEvent(&gyro);

    ax = accel.acceleration.x - offset_ax;
    ay = accel.acceleration.y - offset_ay;
    az = accel.acceleration.z - offset_az;
    gx = gyro.gyro.x - offset_gx;
    gy = gyro.gyro.y - offset_gy;
    gz = gyro.gyro.z - offset_gz;
}

void plantStatus::updateSensors() {
    readAS5600(); readMPU6050();
}

// ---------- Sensor Reading y Calibration ----------

void plantStatus::calibrateSensors() {
    Serial.println("Iniciando calibracion de sensores. Mantenga el sistema quieto...");
    
    offset_angulo = 0; offset_ax = 0; offset_ay = 0; offset_az = 0;
    offset_gx = 0; offset_gy = 0; offset_gz = 0;

    long sum_raw_angle = 0;
    int num_samples = 100;
    
    for (int i = 0; i < num_samples; i++) {
        // Leemos el RAW_ANGLE para el AS5600
        Wire.beginTransmission(0x36); 
        Wire.write(0x0C);             
        Wire.endTransmission(false);
        Wire.requestFrom((uint16_t)0x36, (uint8_t)2, true);
        if (Wire.available() == 2) {
            uint16_t rawAngle = Wire.read() << 8;
            rawAngle |= Wire.read();
            sum_raw_angle += rawAngle;
        }

        sensors_event_t accel, gyro, temp;
        _sys->mpu_temp->getEvent(&temp);
        _sys->mpu_accel->getEvent(&accel);
        _sys->mpu_gyro->getEvent(&gyro);
        
        offset_ax += accel.acceleration.x; 
        offset_ay += accel.acceleration.y; 
        offset_az += accel.acceleration.z;
        offset_gx += gyro.gyro.x; 
        offset_gy += gyro.gyro.y; 
        offset_gz += gyro.gyro.z;
        
        delay(10);
    }

    offset_ax /= num_samples; offset_ay /= num_samples; offset_az /= num_samples;
    offset_gx /= num_samples; offset_gy /= num_samples; offset_gz /= num_samples;
    
    // Calibración por software limpia:
    uint16_t avg_raw_angle = sum_raw_angle / num_samples;
    offset_angulo = (avg_raw_angle * 360.0f) / 4096.0f;
    
    Serial.println("Calibracion por software finalizada.");
}

// ---------- Generador de Perfiles ----------
void plantStatus::stopProfile() {
    _currentProfile = PROFILE_NONE; 
    // Aseguramos de que apague el motor por si acaso
    applyMotor(0.0, 0, false);
}

void plantStatus::startStep(float targetDuty, int direction, uint32_t duration_ms) {
    _currentProfile = PROFILE_STEP;
    _profileStartDuty = targetDuty;
    _profileDirection = direction;
    _profileDuration = duration_ms;
    _profileStartTime = millis();
    
    // Aplicar el escalón de inmediato
    applyMotor(_profileStartDuty, _profileDirection, false);
}

// Rampa escalonada: niveles stepSize, 2*stepSize, ... hasta maxDuty, cada uno durante stepTime_ms
void plantStatus::startRamp(float stepSize, uint32_t stepTime_ms, float maxDuty, int direction) {
    // Evitar divisiones por cero o parametros invalidos
    _profileStepSize = (stepSize > 0.0f) ? stepSize : 1.0f;
    _profileStepTime = (stepTime_ms > 0) ? stepTime_ms : 1000;
    if (maxDuty > 100.0f) maxDuty = 100.0f;
    if (maxDuty < _profileStepSize) maxDuty = _profileStepSize;

    uint32_t numSteps = (uint32_t)ceilf(maxDuty / _profileStepSize - 1e-4f);

    _currentProfile = PROFILE_RAMP;
    _profileStartDuty = _profileStepSize;   // Primer escalon
    _profileEndDuty = maxDuty;              // Tope de PWM
    _profileDirection = direction;
    _profileDuration = numSteps * _profileStepTime;  // Duracion total derivada
    _profileStartTime = millis();

    applyMotor(_profileStartDuty > maxDuty ? maxDuty : _profileStartDuty, _profileDirection, false);
}

void plantStatus::updateProfile() {
    if (_currentProfile == PROFILE_NONE) return;

    uint32_t currentMillis = millis();
    uint32_t elapsed = currentMillis - _profileStartTime;

    // Condicion de finalizacion
    if (elapsed >= _profileDuration) {
        applyMotor(0.0, 0, false); // Coast al terminar
        _currentProfile = PROFILE_NONE;
        return;
    }

    if (_currentProfile == PROFILE_RAMP) {
        uint32_t updateInterval_ms = _Ts_ms;
        static uint32_t lastProfileUpdate = 0;

        // Solo recalcula segun el tiempo, no en cada ciclo del procesador
        if (currentMillis - lastProfileUpdate >= updateInterval_ms) {
            lastProfileUpdate = currentMillis;

            // Escalon actual segun el tiempo transcurrido
            uint32_t stepIndex = elapsed / _profileStepTime;
            float steppedDuty = (stepIndex + 1) * _profileStepSize;
            if (steppedDuty > _profileEndDuty) steppedDuty = _profileEndDuty;

            // Aplicar solo si cambio el escalon
            if (fabs(currentDuty - steppedDuty) >= 0.01f) {
                applyMotor(steppedDuty, _profileDirection, false);
            }
        }
    }
}