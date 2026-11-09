#ifndef PLANTCONTROLLER_H
#define PLANTCONTROLLER_H

#include <Arduino.h>
#include "plantStatus.h"

// Lista de modos para el método setTarget
enum ControlMode { CTRL_NONE, CTRL_PID_POSITION, CTRL_SEQUENCE, CTRL_FUTURE_OTHER };

// Codigo de 'estado' que se transmite por telemetria mientras corre una secuencia (comando F)
// (0 = ninguno, 1 = escalon, 2 = rampa -> definidos por MotorProfile en plantStatus.h)
const uint8_t ESTADO_SECUENCIA = 3;

// Estructura para almacenar los Taos en la tabla
struct PIDGains {
    float K;
    float Ti;
    float Td;
};

class plantController {
private:
    plantStatus* _planta;

    // Tabla de ganancias dinámicas (10 puntos: de 0° a 90°)
    PIDGains _gainTable[10];

    // Ganancias activas actuales
    float _Kp, _Ki, _Kd;
    float _Ts;

    float _setpointAngle, _maxSpeedPWM, _errorPrev, _integral;
    uint32_t _lastTime;
    ControlMode _currentMode;

    // Variables para el generador de secuencias (comando F)
    float _seqAngles[20];
    int _seqCount;
    int _seqIndex;
    uint32_t _seqTimeMs;
    uint32_t _lastSeqTimeMs;

    // Variables físicas medibles del aeropéndulo
    float _plantVoltage;
    float _J, _m1, _m2, _l1, _l2, _b, _mgl;
    float _slopeTorque1, _slopeTorque2, _b_mot;

    // Métodos de controladores específicos
    void calcDynGains(float currentAngle);
    void PIDcontroller(); 
    
    // Funciones matemáticas auxiliares
    float PWM2Voltage(float duty_PWM);
    float Voltage2PWM(float voltage);
    float Voltage2Torque(float voltage);
    float Torque2Voltage(float torque);

public:
    plantController(plantStatus* plantaPtr);
    
    // metodo para muestreo
    void setSamplingTime(float Ts_seconds);
    
    // Métodos para la tabla
    void setGlobalPID(float K, float Ti, float Td); 
    void setTablePID(int targetAngle, float K, float Ti, float Td); 
    
    // Configurador general de objetivos
    void setTarget(float targetAngle, float maxSpeedPWM, ControlMode mode);

    // Configurador de secuencias de referencia (Comando F)
    void setSequence(float* angles, int count, uint32_t timePerRefMs, float maxSpeedPWM);
    
    void stop();

    // Referencia activa (NAN si el PID no esta controlando)
    float getSetpoint() const { return (_currentMode == CTRL_NONE) ? NAN : _setpointAngle; }

    // true mientras la secuencia de referencias (comando F) esta en curso
    bool isSequenceActive() const { return _currentMode == CTRL_SEQUENCE; }
    
    // Despachador maestro
    void update(); 
};

#endif