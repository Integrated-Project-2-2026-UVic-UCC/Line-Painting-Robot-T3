#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h> // Filtros ya incluidos. Usa PCNT en vez interrupciones, que hace que el conteo ocurra de forma independiente al código principal.

// Configuración física del robot para cálculos de conversión
struct RobotPhysics {
    float wheelDiameter; // Diámetro de la rueda en metros
    int ppr;             // Pulsos por revolución del encoder (fase A)
    float gearRatio;     // Relación de la reductora (ej: 30.0)
};

// Configuración de pines para cada encoder
struct EncoderConfig {
    int pinA;
    int pinB;
};

class EncoderHandler {
private:
    // Referencias a los objetos de hardware de la librería ESP32Encoder
    ESP32Encoder _encoderDer;
    ESP32Encoder _encoderIzq;

    // Parámetros de configuración
    RobotPhysics _phys;
    float _metersPerTick;

    // Almacenamiento de pines para la inicialización en begin()
    int _pinDerA, _pinDerB;
    int _pinIzqA, _pinIzqB;

    // Variables de control de tiempo y estado anterior
    unsigned long _lastUpdateTime;
    long _lastTicksIzq;
    long _lastTicksDer;

    // Variables de salida (resultados)
    float _velIzq = 0;
    float _velDer = 0;
    float _distIzq = 0;
    float _distDer = 0;

public:
    // Constructor: recibe las configuraciones y las referencias a los objetos encoder
    EncoderHandler(EncoderConfig configDer, EncoderConfig configIzq, 
                   ESP32Encoder &encDer, ESP32Encoder &encIzq, 
                   RobotPhysics phys);

    // Configura el hardware y los filtros
    void begin();

    // Realiza los cálculos de velocidad y distancia (llamar en el loop principal)
    void update();

    // Getters para obtener los datos procesados
    float getVelocityIzq();
    float getVelocityDer();
    float getDistIzq();
    float getDistDer();
};