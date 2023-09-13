#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wheel.h>

#define duty_cycle 150 // DEFINO UN DUTY_CYCLE  POR DEFECTO

class Robot : public Wheel
{
private:
    int lx, ly; /*half of the distance between front wheels and 𝑙𝑖𝑦
    half of the distance between front wheel and the rear wheels.*/
    void setMotor(int in, int in2, int dtc, int state1, int state2, int pwmC);

public:
    Wheel *wheel_1;
    Wheel *wheel_2;
    Wheel *wheel_3;
    Wheel *wheel_4;

    Robot(Wheel *wheel1,
          Wheel *wheel2,
          Wheel *wheel3,
          Wheel *wheel4, int lix, int liy)
    {
        // Create Wheel objects and assign pointers
        wheel_1 = wheel1;
        wheel_2 = wheel2;
        wheel_3 = wheel3;
        wheel_4 = wheel4;

        lx = lix;
        ly = liy;
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
};
#endif