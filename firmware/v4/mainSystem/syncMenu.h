#ifndef SYNCMENU_H
#define SYNCMENU_H

#include <Arduino.h>
#include "plantStatus.h"
#include "plantController.h" 

struct __attribute__((packed)) SensorDataPacket {
    uint8_t header[2]; 
    uint32_t timestamp; 
    float pwm;
    float angulo;
    float ax, ay, az;
    float gx, gy, gz;
    float ref;         // Referencia del PID en grados (NaN si no hay control activo)
    uint8_t estado;    // Perfil activo: 0=ninguno, 1=escalon, 2=rampa, 3=secuencia (F)
    uint8_t footer;    
};

class SyncMenu {
private:
    plantStatus* _planta; 
    plantController* _controlador; 
    uint32_t _lastTxTime;

    void executeCommand(String input); 

public:
    SyncMenu(plantStatus* plantaPtr, plantController* ctrlPtr); 
    
    void processSerialCommands();
    void transmitData(uint32_t frequencyHz);
};

#endif