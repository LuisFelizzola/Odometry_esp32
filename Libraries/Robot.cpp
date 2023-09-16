#include <Robot.h>

void Robot::moveForward(int dutyC = duty_cycle)
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), dutyC, 0, 1, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), dutyC, 0, 1, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), dutyC, 1, 0, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), dutyC, 1, 0, wheel_4->inPin(4));
}

void Robot::moveBackward(int dutyC = duty_cycle)
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), dutyC, 1, 0, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), dutyC, 1, 0, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), dutyC, 0, 1, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), dutyC, 0, 1, wheel_4->inPin(4));
}

void Robot::moveLeft(int dutyC = duty_cycle)
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), dutyC, 0, 1, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), dutyC, 1, 0, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), dutyC, 1, 0, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), dutyC, 0, 1, wheel_4->inPin(4));
}

void Robot::rotation(int dutyC = duty_cycle)
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), dutyC, 0, 1, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), dutyC, 1, 0, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), dutyC, 0, 1, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), dutyC, 1, 0, wheel_4->inPin(4));
}

void Robot::moveRight(int dutyC = duty_cycle)
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), dutyC, 1, 0, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), dutyC, 0, 1, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), dutyC, 0, 1, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), dutyC, 1, 0, wheel_4->inPin(4));
}
void Robot::stop()
{
    setMotor(wheel_1->inPin(1), wheel_1->inPin(2), 0, 1, 0, wheel_1->inPin(4));
    setMotor(wheel_2->inPin(1), wheel_2->inPin(2), 0, 0, 1, wheel_2->inPin(4));
    setMotor(wheel_3->inPin(1), wheel_3->inPin(2), 0, 1, 0, wheel_3->inPin(4));
    setMotor(wheel_4->inPin(1), wheel_4->inPin(2), 0, 0, 1, wheel_4->inPin(4));
}
void Robot::setMotor(int in, int in2, int dtc, int state1, int state2, int pwmC)
{
    digitalWrite(in, state1);
    digitalWrite(in2, state2);
    ledcWrite(pwmC, dtc);
}
void Robot::setPosition(float x, float y, float teta)
{

    X = x;
    Y = y;
    TETA = teta;
}
float Robot::posX()
{
    return X;
}
float Robot::posY()
{
    return Y;
}
float Robot::posA()
{
    return TETA;
}
float Robot::velX()
{
    return Vx;
}
float Robot::velY()
{
    return Vy;
}
float Robot::velTeta()
{
    return W;
}