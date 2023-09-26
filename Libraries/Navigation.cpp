#include <Navigation.h>
void Navigation::positionPID(PID_CONTROL &pid, Robot &robot, float Lx, float Ly, float r, long ct, long prevT, float kp, float ki, float kd, float tao)
{
    Xi = robot.posX();
    Yi = robot.posY();
    Tetai = robot.posA();
    float dx = XT - Xi;           // DESPLAZAMIENTO EN X
    float dy = YT - Yi;           // DESPLAZAMIENTO EN Y
    float dTeta = TETA_T - Tetai; // desplazamiento del ángulo teta
    float deltaT = ((float)(ct - prevT)) / 1.0e6;

    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.tau = tao;
    pid.T = deltaT;
    Ux = 0;
    Uy = 0;
    Uw = 0;
    if (!disableX) // si existe un cambio en la posicion o si ya ha llegado a su destino
    {              // si el robot se esta movilizano el eje x
        // AJUSTO LOS LIMITES DEL INTEGRADOR (PARA EVITAR EL WINDUP), CON LA CONVENCION DEL 70% DE LOS LIMITES FISICOS
        pid.limMaxInt = 0.7 * Vxmax;
        pid.limMinInt = 0.7 * Vxmin;
        pid.limMax = Vxmax;
        pid.limMin = Vxmin;
        Ux = PIDController_Update(pid, XT, Xi);
    }
    else if (!disableY)
    { // si el robot se esta moviendo en el eje Y
        pid.limMaxInt = 0.7 * Vymax;
        pid.limMinInt = 0.7 * Vymin;
        pid.limMax = Vymax;
        pid.limMin = Vymin;
        Uy = PIDController_Update(pid, YT, Yi);
    }
    else if (!disableW)
    { // si el robot esta en rotacion pura
        pid.limMaxInt = 0.7 * Wmax;
        pid.limMinInt = 0.7 * Wmin;
        pid.limMax = Wmax;
        pid.limMin = Wmin;
        Uw = PIDController_Update(pid, TETA_T, Tetai);
    }
    else
    {
        robot.stop();
    }

    inverseKinematics(Ux, Uy, r, Lx, Ly, Uw);
}
void Navigation::wheelVelocityPID(PID_CONTROL &pid, Robot &robot, long ct, long prevT, float kp, float ki)
{
    if (!disable)
    {
        float deltaT = ((float)(ct - prevT)) / 1.0e6;
        pid.Kp = kp;
        pid.Ki = ki;
        pid.T = deltaT;
        // limites del integrador y de la señal de control
        pid.limMaxInt = 0.5 * maxDuty;
        pid.limMinInt = 0.5 * -maxDuty;
        pid.limMax = maxDuty;
        pid.limMin = -maxDuty;

        // aplico el PID con respecto a la velocidad w1, debido a la aproximacion y que tomo en cuenta el mejor encoder
        U = PIController_Update(pid, w1T, robot.wheel_1->Velocityradians());
        dutyCycle = (int)abs(U); // me aseguro que el valor es positivo
        // Me aseguro que el valor es positivo
        if (!disableX)
        {
            if (Ux >= 0)
            {
                robot.moveForward(dutyCycle);
            }
            else
            {
                robot.moveBackward(dutyCycle);
            }
        }
        else if (!disableY)
        {
            if (Uy >= 0)
            {
                robot.moveRight(dutyCycle);
            }
            else
            {
                robot.moveLeft(dutyCycle);
            }
        }
        else if (!disableW)
        {
            robot.rotation(dutyCycle);
        }
    }
    else
    {
        // robot.stop();
    }
}
void Navigation::inverseKinematics(float vx, float vy, float r, float lx, float ly, float w)
{
    if (!disable)
    {
        if (vx != 0 || vy != 0 || w != 0)
        {
            w1T = (1 / r) * (vx - vy - (lx + ly) * w);
            if (disableW) // if the robot is not rotating
            {
                w1T = 0.785 * w1T;
            }
        }
        else
        {
            w1T = 0;
            w2T = 0;
            w3T = 0;
            w4T = 0;
        }

        // w2T = 1 / r + (vx + vy + (lx + ly) * w);
        // w3T = 1 / r + (vx + vy - (lx + ly) * w);
        // w4T = 1 / r + (vx - vy + (lx + ly) * w);
    }
    else if (!obstacle)
    {
        w1T = 0;
        w2T = 0;
        w3T = 0;
        w4T = 0;
    }
}
// Modifico los estados para alcanzar el objetivo (tengo en cuenta obstaculos)
void Navigation::MovementPlanning(Robot &robot, float minX, float minY, float minT)
{
    if ((XT - Xi) > 0.2 || -(XT - Xi) > 0.2)
    {
        disableX = false;
    }
    else
    {
        disableX = true;
    }
    if (((YT - Yi) > 0.2 || -(YT - Yi) > 0.2))
    {
        disableY = false;
    }
    else if (disableX)
    {
        disableY = true;
    }
    if (-(TETA_T - Tetai) > 0.2)
    {
        disableW = false;
    }
    else if (disableY)
    {
        disableW = true;
    }
    if (disableX && disableY && disableW)
    {
        disable = true;
    }
    else
    {
        disable = false;
    }
}

void Navigation::Navigate(Robot &r, float xT, float yT, float tetaT, float disFront, float disRight, float disLeft, bool disA = STOP)
{
    xFront = disFront;
    xRight = disRight;
    xLeft = disLeft;
    // si el objetivo cambio:
    if (XT != xT) // el objetivo en X cambio:
    {
        disableX = false;
    }
    if (YT != yT) // el objetivo en Y cambio:
    {
        disableY = false;
    }
    if (TETA_T != tetaT) // el objetivo en Teta_T cambio:
    {
        disableW = false;
    }
    XT = xT;
    YT = yT;
    TETA_T = tetaT;
    // si encuentra algun obstaculo mientras se esta moviendo:
    if (((xFront < minObstacle) && !disableX) || ((xLeft < minObstacle) && !disableY && DYi < 0) || ((xRight < minObstacle) && !disableY && DYi > 0))
    {
        obstacle = true;
        disable = true;
        // reinicio las variables para la evasion de obstaculos:
    }
    else
    {
        // reinicio las variables para la evasion de obstaculos:
        noRight = false;
        noForward1 = false;
        noForward2 = false;
        // habilito nuevamente el PID de velocidad y deshabilito la evacion de obstaculos
        disable = false;
        obstacle = false;
    }
    pathPlanning_Generator(r); // cambiar su movimiento en caso de encontrar algun obstaculo
    if (disA)                  // si es deshabilitado por orden externa
    {
        disable = true;
        obstacle = false;
    }
    else
    {
        disable = false;
    }
}
void Navigation::pathPlanning_Generator(Robot &robot)
{
    // Si se encuentra algun obstaculo durante el movimiento:
    if (obstacle)
    {
        if (!disableX)
        { // si tiene movimiento en el ejeX
            if (!noRight && xRight > minObstacle)
            {
                robot.moveRight(120);
            }
            else if (xFront < minObstacle && xLeft > minObstacle)
            {
                noRight = true;
                robot.moveLeft(120);
            }
        }
        else if (!disableY && DYi > 0)
        { // si el obstaculo esta cuando el robot se mueve hacia la derecha
            if (xRight < minObstacle && !noForward1 && xFront > minObstacle)
            {
                robot.moveForward(100);
            }
            else if (xRight < minObstacle)
            {
                noForward1 = true;
                robot.moveBackward(100);
            }
        }
        else if (!disableY && DYi < 0)
        { // si el obstaculo esta cuando se mueve a l aizquierda
            if (xLeft < minObstacle && !noForward2 && xFront > minObstacle)
            {
                robot.moveForward(100);
            }
            else if (xLeft < minObstacle)
            {
                noForward2 = true;
                robot.moveBackward(100);
            }
        }
        else
        {
            obstacle = false;
        }
    }
}
float Navigation::showX()
{
    return Xi;
}
float Navigation::showY()
{
    return Yi;
}
float Navigation::showA()
{
    return Tetai;
}
float Navigation::showUx()
{
    return Ux;
}
float Navigation::showUy()
{
    return Uy;
}
float Navigation::showUw()
{
    return Uw;
}
float Navigation::showU()
{
    return U;
}
int Navigation::showDuty()
{
    return dutyCycle;
}
float Navigation::showWt()
{
    return w1T; // setPoint del PID de velocidad
}
float Navigation::showDXi()
{
    return DXi;
}
bool Navigation::enableX()
{
    return disableX;
}
bool Navigation::enableY()
{
    return disableY;
}
bool Navigation::enableW()
{
    return disableW;
}
bool Navigation::enable()
{
    return disable;
}