#ifndef WHEEL_H
#define WHEEL_H
#include <Arduino.h>
#define Pi 3.14159;

class Wheel
{
private:
    int in1;
    int in2;
    int en;
    int r;
    int pwm;

public:
    Wheel(int in_1, int in_2, int en_); // parametros para el mdulo l298n

    Wheel(int in_1, int in_2, int en_, int radius, int pwmCh) // parametros para el modulo l298n con el esp32
    {
        in1 = in_1;
        in2 = in_2;
        en = en_;
        r = radius;
        pwm = pwmCh;
    } // parametros para el módulo L298N
      // Tell the compiler to do what it would have if we didn't define a ctor:
    float w = 0.0;
    Wheel() = default;
    int Radius()
    {
        return r;
    }
    void setVelocity(float wn) // set the velocity of the wheel in ct/s
    {
        w = wn;
    }
    float Velocity()
    { // velocity of the wheel in ct/s
        return w;
    }
    float Velocityradians()
    { // velocity of the wheel in rad/s
        float wr = w * 2 * Pi;
        return wr;
    }
    int inPin(int n)
    { // funcion devuelve los pines para controlar el motor
        if (n == 1)
        {
            return in1;
        }
        else if (n == 2)
        {
            return in2;
        }
        else if (n == 3)
        {
            return en;
        }
        else if (n == 4)
        {
            return pwm;
        }
        else
        {
            return 0;
        }
    }
};

#endif