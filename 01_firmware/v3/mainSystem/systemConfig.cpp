#include "systemConfig.h"

SystemConfig::SystemConfig(byte* spiPins, byte* i2cPins, byte* mPins, byte stbyPin, byte i2cAddress) {
    _pinSPI = spiPins;
    _pinI2C = i2cPins;
    motorPins = mPins;
    _pinSTBY = stbyPin;
    _i2cAddress = i2cAddress;
    
    _bauds = 921600;
    timeout = 5000;
    
    _SDStatus = false;
    _I2CStatus = false;
    _SerialStatus = false;
}

bool SystemConfig::start() {
    Serial.begin(_bauds);
    unsigned long SerialTimer = millis(); 
    while (millis() - SerialTimer < timeout) {
        if (Serial) { _SerialStatus = true; break; }
        yield();
    }

    // --- Configuración H-Bridge y MCPWM ---
    pinMode(motorPins[1], OUTPUT); // AIN1
    pinMode(motorPins[2], OUTPUT); // AIN2
    
    if (_pinSTBY != 255) {
        pinMode(_pinSTBY, OUTPUT);
        digitalWrite(_pinSTBY, HIGH); // Despertar el driver
    }

    digitalWrite(motorPins[1], LOW);
    digitalWrite(motorPins[2], LOW);

    // Inicializar el pin de PWM para MCPWM (Unidad 0, Timer 0, Operador A)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorPins[0]);
    
    // Configurar los parámetros del MCPWM
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;    // 20kHz
    pwm_config.cmpr_a = 0.0;         // Duty cycle inicial 0%
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // --- Inicialización I2C y Sensores ---
    Wire.begin(_pinI2C[0], _pinI2C[1]);
    Wire.setClock(400000); 
    Wire.beginTransmission(0x36);
    if (Wire.endTransmission() == 0) {
        if (_SerialStatus) Serial.println("AS5600 Encontrado.");
    } else {
        if (_SerialStatus) Serial.println("Error: AS5600 NO encontrado.");
    }

    if (!mpu.begin()) {
        if (_SerialStatus) Serial.println("Fallo al encontrar MPU6050");
        _I2CStatus = false;
    } else {
        if (_SerialStatus) Serial.println("MPU6050 Encontrado e Inicializado!");
        mpu_temp = mpu.getTemperatureSensor();
        mpu_accel = mpu.getAccelerometerSensor();
        mpu_gyro = mpu.getGyroSensor();
        _I2CStatus = true;
    }

    // --- Configuración SPI y SD ---
    SPI.begin(_pinSPI[2], _pinSPI[1], _pinSPI[0], _pinSPI[3]);
    if (!SD.begin(_pinSPI[3], SPI)) {
        _SDStatus = false;
        if (_SerialStatus) Serial.println("Error: Fallo SD.");
    } else {
        _SDStatus = true;
    }

    return (_SDStatus && _I2CStatus);
}