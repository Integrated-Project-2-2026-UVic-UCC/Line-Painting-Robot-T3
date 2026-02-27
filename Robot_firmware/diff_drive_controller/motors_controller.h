#pragma once

#include <Arduino.h>

// Struct para agrupar los pines de cada motor
struct MotorHW {
    int IN1;
    int IN2;
    int PWM;
};

class MotorsController {
private:
    // Pines del Motor A (Izquierdo)
    int _AIN1, _AIN2, _PWMA;
    // Pines del Motor B (Derecho)
    int _BIN1, _BIN2, _PWMB;

    // Método privado para aplicar el escalado de velocidad (0-255)
    void apply_speed_limits(int &velA, int &velB);

    // Método privado para la ejecución física en el driver
    void setMotor(int IN1, int IN2, int channel, int speed);

public:
    // Constructor que recibe los structs de hardware
    MotorsController(MotorHW hwA, MotorHW hwB);

    // Configura los pines y los canales PWM (ledc)
    void begin();

    // Mueve los motores con valores de -255 a 255
    void move(int velA, int velB);
};