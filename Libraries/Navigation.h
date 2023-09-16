#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <PID.h>
#include <Robot.h>

typedef struct Movement
{
    bool moveF = false;  // MOVE FORWARD
    bool moveB = false;  // MOVE BACKWARD
    bool moveR = false;  // MOVE RIGHT
    bool moveL = false;  // MOVE LEFT
    bool rotate = false; // PURE ROTATION
    bool stop = false;   // stop
};
#define STOP false // STOP THE ROBOT
class Navigation
{
private:
    void inverseKinematics(float vy, float vx, float r, float lx, float ly, float w);
    void pathPlanning_Generator(Robot &robot);
    Movement movement;
    // VELOCIDAD ANGULAR QUE DEBEN ALCANZAR LAS RUEDAS (TARGET DEL PID)
    float w1T = 0;
    float w2T = 0;
    float w3T = 0;
    float w4T = 0;
    bool disable = STOP; // Apagar Mercury
    // Velocidades que debe tener el robot (Salidas del PID de posicion)
    float vx = 0;
    float vy = 0;
    float w = 0;
    // posicion inicial del robot:
    float Xi;
    float Yi;
    float Tetai;
    // Posicion de objetivo
    float XT = 0;
    float YT = 0;
    float TETA_T = 0;
    float dutyCicle = 0; // dutycycle al iniciar;
    float minDuty = 100; // velocidad maxima para el controlador de los motores
    float maxDuty = 200; // velocidad min para el controlador de los motores

public:
    // variables para la deteccion de algun obstaculo
    float xFront = 30;
    float xRight = 30;
    float xLeft = 30;
    float minObstacle = 10; // distancia minima a la cual el robot detecta un objeto como obstaculo
    // variables para validar la ruta al evadir los obstaculos, si fue viable o no
    bool noRight = false;
    bool noLeft = false;
    Navigation(float xi, float yi, float tetai, float minObs, float minDT, float maxDT)
    {
        Xi = xi;
        Yi = yi;
        Tetai = tetai;
        minObstacle = minObs;
        minDuty = minDT;
        maxDuty = maxDT;
        dutyCicle = minDT;
    };
    Navigation() = default;
    void MovementPlanning(Robot robot, float minX, float minY, float minT, bool enable);
    void Navigate(Robot &r, float xT, float yT, float tetaT, float disFront, float disRight, float disLeft);
};

#endif