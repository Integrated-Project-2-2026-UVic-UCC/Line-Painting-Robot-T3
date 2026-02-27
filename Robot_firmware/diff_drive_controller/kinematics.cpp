#include "kinematics.h"

Kinematics::Kinematics(float trackWidth) : _trackWidth(trackWidth) {}

// CINEMÁTICA INVERSA: ¿Cuánto debe correr cada rueda?
WheelSpeeds Kinematics::calculateWheelSpeeds(float v, float w) { // La v es la x del twist, la y al ser diferencial y no poder mover de lateral es 0.
    WheelSpeeds ws;
    // v = velocidad lineal (m/s), w = velocidad angular (rad/s)
    ws.left  = v - (w * _trackWidth / 2.0);
    ws.right = v + (w * _trackWidth / 2.0);
    return ws;
}

// ODOMETRÍA: ¿Dónde estoy ahora?
// dDist son los metros que ha recorrido cada rueda DESDE el último update
void Kinematics::updateOdometry(float dDistIzq, float dDistDer, float imuYawRad) {
    // 1. Distancia media lineal recorrida por el centro del robot
    float ds = (dDistDer + dDistIzq) / 2.0;

    // 2. El ángulo lo tomamos directamente de la IMU (es lo más fiable)
    _currentPose.theta = imuYawRad;

    // 3. Proyectamos el movimiento en el plano X e Y
    _currentPose.x += ds * cos(_currentPose.theta);
    _currentPose.y += ds * sin(_currentPose.theta);
}

// Para cuando Zenoh te pida la posición
Pose Kinematics::getPose() {
    return _currentPose;
}

// Para corregir la posición si el Lidar detecta que te has desviado
void Kinematics::setPose(float x, float y, float theta) {
    _currentPose.x = x;
    _currentPose.y = y;
    _currentPose.theta = theta;
}

void Kinematics::resetPose() {
    _currentPose.x = 0;
    _currentPose.y = 0;
    _currentPose.theta = 0;
}