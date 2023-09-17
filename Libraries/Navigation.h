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
    bool disable = false; // Apagar Mercury
    bool disableX = false;
    bool disableY = false;
    bool disableW = false;
    // posicion inicial del robot:
    float Xi;
    float Yi;
    float Tetai;
    // Posicion de objetivo
    float XT = 0;
    float YT = 0;
    float TETA_T = 0;
    int dutyCycle = 0; // dutycycle al iniciar;
    int minDuty = 100; // velocidad maxima para el controlador de los motores
    int maxDuty = 200; // velocidad min para el controlador de los motores
    bool noRight = false;
    bool noForward1 = false;
    bool noForward2 = false;
    float Ux = 0;
    float Uy = 0;
    float Uw = 0;
    float U = 0;

public:
    // variables para la deteccion de algun obstaculo
    float xFront = 30;
    float xRight = 30;
    float xLeft = 30;
    // cambio inicial de posicion (Util para saber los tipos de movimientos a ejecutar)
    float DXi = 0;
    float DYi = 0;
    float DTetai = 0;
    // Limites fisicos del robot
    float Vxmax;
    float Vymax;
    float Wmax;
    float Vxmin;
    float Vymin;
    float Wmin;
    float minObstacle = 10; // distancia minima a la cual el robot detecta un objeto como obstaculo
    // variables para validar la ruta al evadir los obstaculos, si fue viable o no
    Navigation(float xi, float yi, float tetai, float minObs, int minDty, int maxDTy, float vxM, float vyM, float wM)
    {
        Xi = xi;
        Yi = yi;
        Tetai = tetai;
        minObstacle = minObs;
        minDuty = minDty;
        maxDuty = maxDTy;
        Vxmax = vxM;
        Vymax = vyM;
        Wmax = wM;
        Vxmin = -vxM;
        Vymin = -vyM;
        Wmin = -wM;
    };
    Navigation() = default;
    void wheelVelocityPID(PID_CONTROL &pid, Robot &robot, long ct, long prevT, float kp, float ki);
    void positionPID(PID_CONTROL &pid, Robot &robot, float Lx, float Ly, float r, long ct, long prevT, float kp, float ki, float kd, float tao);
    void MovementPlanning(Robot &robot, float minX, float minY, float minT);
    void Navigate(Robot &r, float xT, float yT, float tetaT, float disFront, float disRight, float disLeft, bool enable);
    float showX();
    float showY();
    float showA();
    float showU();
    float showUx();
    float showUy();
    float showUw();
    int showDuty();
    float showWt();
    float showDXi();
    bool enableX();
    bool enableY();
    bool enableW();
    bool enable();
};

#endif