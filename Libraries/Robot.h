#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wheel.h>

#define duty_cycle 0 // DEFINO UN DUTY_CYCLE  POR DEFECTO

class Robot : public Wheel
{
private:
    char state = 'S';
    void setMotor(int in, int in2, int dtc, int state1, int state2, int pwmC);

public:
    Wheel *wheel_1;
    Wheel *wheel_2;
    Wheel *wheel_3;
    Wheel *wheel_4;
    float X, Y, TETA, Vx, Vy, W;
    Robot(Wheel *wheel1,
          Wheel *wheel2,
          Wheel *wheel3,
          Wheel *wheel4, float x, float y, float teta, float vx, float vy, float w)
    {
        // Create Wheel objects and assign pointers
        wheel_1 = wheel1;
        wheel_2 = wheel2;
        wheel_3 = wheel3;
        wheel_4 = wheel4;
        X = x;
        Y = y;
        TETA = teta;
        Vx = vx;
        Vy = vy;
        W = w;
    }

    // Destructor para liberar la memoria de los objetos Wheel
    ~Robot()
    {
        delete wheel_1;
        delete wheel_2;
        delete wheel_3;
        delete wheel_4;
    }
    // defino mis métodos
    void moveLeft(int dutyC);
    void moveForward(int dutyC);
    void moveBackward(int dutyC);
    void moveRight(int dutyC);
    void rotation(int dutyC);
    void stop();
    void setPosition(float x, float y, float teta);
    float posX();
    float posY();
    float posA();
    float velX();
    float velY();
    float velTeta();
    char robotState();
};

#endif