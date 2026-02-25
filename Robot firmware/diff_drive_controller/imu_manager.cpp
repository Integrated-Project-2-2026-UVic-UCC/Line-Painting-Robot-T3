#include "imu_manager.h"

IMUManager::IMUManager() {}

bool IMUManager::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    Wire.setClock(400000); // Velocidad rápida (400kHz) para no retrasar el loop
    
    // Test de conexión: intentamos leer el registro de ID o simplemente conectar
    Wire.beginTransmission(_addr);
    return (Wire.endTransmission() == 0);
}

void IMUManager::update() {
    // Registro 0x3f es el Yaw (según el manual del WT901B)
    Wire.beginTransmission(_addr);
    Wire.write(0x3f); 
    Wire.endTransmission(false);
    
    // Pedimos 2 bytes (el valor es un entero de 16 bits)
    if (Wire.requestFrom(_addr, (uint8_t)2) == 2) {
        int16_t low = Wire.read();
        int16_t high = Wire.read();
        int16_t raw = (high << 8) | low;

        // Conversión matemática del datasheet:
        // El rango es +/- 180 grados mapeado en un entero de 16 bits
        float deg = (float)raw / 32768.0f * 180.0f;
        
        // Pasamos a radianes y normalizamos
        float rad = deg * (PI / 180.0f);
        _currentYaw = rad - _yawOffset;

        // Normalización profesional entre -PI y PI
        while (_currentYaw >  PI) _currentYaw -= 2.0f * PI;
        while (_currentYaw < -PI) _currentYaw += 2.0f * PI;
    }
}

float IMUManager::getYawRad() { return _currentYaw; }

void IMUManager::setYawRad(float newYawRad) {
    // Calculamos el nuevo offset para que la lectura actual coincida con newYawRad
    // _currentYaw (lo que queremos) = rawRad - _yawOffset
    // Por tanto: _yawOffset = rawRad - newYawRad
    
    // Primero necesitamos la lectura "pura" sin el offset anterior
    float rawRad = _currentYaw + _yawOffset; 
    _yawOffset = rawRad - newYawRad;
    _currentYaw = newYawRad; // Actualización inmediata
}

void IMUManager::resetYaw() {
    setYawRad(0.0f);
}