#include "encoder_handler.h"

EncoderHandler::EncoderHandler(EncoderConfig configDer, EncoderConfig configIzq, RobotPhysics phys) : _phys(phys) {
    _pinDerA = configDer.pinA; _pinDerB = configDer.pinB;
    _pinIzqA = configIzq.pinA; _pinIzqB = configIzq.pinB;

    // Distancia por cada pulso (m)
    _metersPerTick = (PI * phys.wheelDiameter) / (phys.ppr * phys.gearRatio * 4.0);
}

void EncoderHandler::begin(){
    // Activar el pull up, para que mientras no detecte nada no se quede flotando. suelen ser low al detectar (conecta a gnd) i high sin conectar. 
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Configura los pines (Fase A, Fase B)
    _encoderIzq.attachFullQuad(_pinIzqA, _pinIzqB);
    _encoderDer.attachFullQuad(_pinDerA, _pinDerB);
    
    // Opcional: Filtro para evitar el "tembleque high-low-high-low" que decías
    // Ignora pulsos de menos de 100 ciclos de reloj (ajustar entre 100 y 500)
    _encoderIzq.setFilter(100); 
    _encoderDer.setFilter(100);

    // Inicialización de tiempos para evitar saltos en el primer update
    _lastUpdateTime = micros();
    _lastTicksIzq = _encoderIzq.getCount();
    _lastTicksDer = _encoderDer.getCount();
}

void EncoderHandler::update() {
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - _lastUpdateTime) / 1000000.0; // Convertir a segundos

    if (deltaTime >= 0.01) { // Actualizar máximo cada 10ms para evitar ruidos
        long currentTicksIzq = _encoderIzq.getCount();
        long currentTicksDer = _encoderDer.getCount();

        // Velocidad = (Delta Pasos * Factor Metros) / Delta Tiempo
        _velIzq = ((currentTicksIzq - _lastTicksIzq) * _metersPerTick) / deltaTime;
        _velDer = ((currentTicksDer - _lastTicksDer) * _metersPerTick) / deltaTime;

        // Odometría acumulada (Distancia total en metros)
        _distIzq += deltaIzq * _metersPerTick;
        _distDer += deltaDer * _metersPerTick;

        // Guardar estado para el próximo cálculo
        _lastTicksIzq = currentTicksIzq;
        _lastTicksDer = currentTicksDer;
        _lastUpdateTime = currentTime;
    }
}

float EncoderHandler::getVelocityIzq() { return _velIzq; }
float EncoderHandler::getVelocityDer() { return _velDer; }
float EncoderHandler::getDistIzq() { return _distIzq; }
float EncoderHandler::getDistDer() { return _distDer; }