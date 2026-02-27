#pragma once

#include <Arduino.h>

// Representa las velocidades objetivo (v, w)
struct Twist {
    float linear_x;  // m/s
    float angular_z; // rad/s
};

// Representa las velocidades individuales de cada rueda
struct WheelSpeeds {
    float left;  // m/s
    float right; // m/s
};

// Representa la posición y orientación global del robot
struct Pose {
    float x;     // metros
    float y;     // metros
    float theta; // radianes
};

class Kinematics {
private:
    float _trackWidth; // Distancia entre el centro de las dos ruedas (metros)
    Pose _currentPose = {0, 0, 0};

public:
    // Constructor: requiere el ancho de vía del robot
    Kinematics(float trackWidth);

    // Cinemática Inversa: De comando de velocidad global a velocidad de ruedas
    WheelSpeeds calculateWheelSpeeds(float v, float w);

    // Odometría: Actualiza la posición (X, Y, Theta) basada en desplazamientos
    void updateOdometry(float dDistIzq, float dDistDer, float imuYawRad);

    // Getters y Setters de Pose
    Pose getPose();
    void setPose(float x, float y, float theta);
    void resetPose();
};