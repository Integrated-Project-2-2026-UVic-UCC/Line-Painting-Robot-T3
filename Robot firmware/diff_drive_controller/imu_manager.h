#pragma once

#include <Arduino.h>
#include <Wire.h>

class IMUManager {
    private:
        const uint8_t _addr = 0x50; // Dirección I2C del WT901B
        float _currentYaw = 0;      // Yaw actual en Radianes
        float _yawOffset = 0;       // Offset acumulado para correcciones de pose

    public:
        // Constructor vacío
        IMUManager();

        // Inicializa el bus I2C y verifica la conexión con el sensor
        bool begin(int sda, int scl);

        // Lee los registros del sensor y actualiza el valor de Yaw
        void update();

        // Devuelve la orientación actual en radianes (-PI a PI)
        float getYawRad();

        // Fuerza una nueva orientación (útil para correcciones externas de SLAM/Lidar)
        void setYawRad(float newYawRad);

        // Pone la orientación actual a cero (define el "frente" del robot)
        void resetYaw();
};