#include "motors_controller.h"

MotorsController::MotorsController(MotorHW hwA, MotorHW hwB): 
    _AIN1(hwA.IN1), _AIN2(hwA.IN2), _PWMA(hwA.PWM), 
    _BIN1(hwB.IN1), _BIN2(hwB.IN2), _PWMB(hwB.PWM) {}

void MotorsController::begin() {
    pinMode(_AIN1, OUTPUT);
    pinMode(_AIN2, OUTPUT);  
    pinMode(_BIN1, OUTPUT);
    pinMode(_BIN2, OUTPUT);

    // Configurar el canal PWM para el motor A y B
    ledcSetup(0, 20000, 8); 
    ledcSetup(1, 20000, 8); 
    // Canal 0 -> generador de pwm que tiene la esp32. por eso se usa ledc en vez de analogwrite. hay de 0-15 canales.
    // 20kHz -> Es lo suficientemente alto para ser inaudible i lo suficientemente bajo para no generar demasiado calor por pérdidas de conmutación en el driver i ser fino.
    // 8 bits -> define tus pasos de velocidad. Con 8 bits, tienes 256 niveles (de 0 a 255) de sobras.
    ledcAttachPin(_PWMA, 0); // Pin PWMA al canal 0
    ledcAttachPin(_PWMB, 1); // Pin PWMB al canal 1
}

void MotorsController::move(int velA, int velB) {
    // --- Escalado Inteligente ---
    apply_speed_limits(velA, velB);

    // --- Ejecución ---
    setMotor(_AIN1, _AIN2, 0, velA); // Motor A en Canal 0
    setMotor(_BIN1, _BIN2, 1, velB); // Motor B en Canal 1
}

void MotorsController::apply_speed_limits(int &velA, int &velB) {
    // --- Escalado Inteligente ---
    int max_speed = max(abs(velA), abs(velB));
    if (max_speed > 255) {
        float scaling_factor = 255.0 / max_speed;
        velA = (int)(velA * scaling_factor);
        velB = (int)(velB * scaling_factor);
    }
}

// Este es el nuevo método privado que simplifica todo
void MotorsController::setMotor(int IN1, int IN2, int channel, int speed) {
    // Escribimos en los pines directamente el resultado de la comparación
    digitalWrite(IN1, speed > 0); 
    digitalWrite(IN2, speed < 0);
    ledcWrite(channel, abs(speed));
}
