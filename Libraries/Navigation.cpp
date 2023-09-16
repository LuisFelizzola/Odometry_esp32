#include <Navigation.h>

void Navigation::inverseKinematics(float vy, float vx, float r, float lx, float ly, float w)
{
    w1T = 1 / r + (vx - vy - (lx + ly) * w);
    w2T = 1 / r + (vx + vy + (lx + ly) * w);
    w3T = 1 / r + (vx + vy - (lx + ly) * w);
    w4T = 1 / r + (vx - vy + (lx + ly) * w);
}
// Defino los posibles movimientos para alcanzar mi objetivo
void Navigation::MovementPlanning(Robot robot, float minX, float minY, float minT, bool disA = STOP)
{
    uint8_t d = 0; // contador para verificar si el robot ha llegado a su destino
    Xi = robot.posX();
    Yi = robot.posY();
    Tetai = robot.posA();

    float dx = XT - Xi;           // DESPLAZAMIENTO EN X
    float dy = YT - Yi;           // DESPLAZAMIENTO EN Y
    float dTeta = TETA_T - Tetai; // desplazamiento del ángulo teta
    if ((disA) || (disable))
    {
        movement.stop = true;
    }
    else
    {
        movement.stop = false;
    }

    if (dx > minX && Xi < XT)
    { // si el desplazamiento es una avance (1.5cm in)
        movement.moveF = true;
        movement.moveB = false;
    }
    else if (-dx > minX && Xi > XT)
    { // retroceso
        movement.moveB = true;
        movement.moveF = false;
    }
    else
    {
        movement.moveB = false;
        movement.moveF = false;
        d++;
    }

    if (dy > minY && Yi < YT)
    {
        movement.moveR = true;
        movement.moveL = false;
    }
    else if (-dy > minY && Yi > YT)
    {
        movement.moveL = true;
        movement.moveR = false;
    }
    else
    {
        d++;
        movement.moveR = false;
        movement.moveL = false;
    }
    if (dTeta > minT && TETA_T > Tetai)
    {
        movement.rotate = true;
    }
    else
    {
        d++;
        movement.rotate = false;
    }

    if (d == 3) // si no hay cambio significativo en la posicion del robot o este ha llegado a su destino
    {
        disable = true;
    }
    else
    {
        disable = false;
    }
}

void Navigation::Navigate(Robot &r, float xT, float yT, float tetaT, float disFront, float disRight, float disLeft)
{
    // X,Y AND TETA TARGET:
    XT = xT;
    YT = yT;
    TETA_T = tetaT;
    xFront = disFront;
    xLeft = disLeft;
    xRight = disRight;

    pathPlanning_Generator(r);
}
void Navigation::pathPlanning_Generator(Robot &robot)
{
    if (movement.moveF && xFront > minObstacle)
    { // una vez el obstaculo de enfrente ha sido evadido, puedo reiniciar las evaciones
        noRight = false;
    }
    else if (movement.moveR && xRight > minObstacle)
    { // para un obstaculo de la derecha
        noForward1 = false;
    }
    else if (movement.moveL && xLeft > minObstacle)
    { // para un obstaculo de la izquiera
        noForward2 = false;
    }

    if (movement.moveF)
    {
        // PID for velocities of the wheels
        if (xFront > minObstacle)
        {
            robot.moveForward(dutyCicle);
        }
        else if (xFront < minObstacle && !noRight && xRight > minObstacle)
        {
            robot.moveRight(100);
        }
        else if (xFront < minObstacle && xLeft > minObstacle)
        {
            noRight = true;
            robot.moveLeft(100);
        }
    }
    else if (movement.moveB)
    {
        // PID for velocities of the wheels
        robot.moveBackward(dutyCicle);
    }
    else if (movement.moveR)
    {
        if (xRight > minObstacle)
        {
            // PID for velocities of the wheels
            robot.moveRight(dutyCicle);
        }
        else if (xRight < minObstacle && !noForward1 && xFront > minObstacle)
        {
            robot.moveForward(100);
        }
        else if (xRight < minObstacle)
        {
            noForward1 = true;
            robot.moveBackward(100);
        }
    }
    else if (movement.moveL)
    {
        if (xLeft > minObstacle)
        {
            // PID for velocities of the wheels
            robot.moveLeft(dutyCicle);
        }
        else if (xLeft < minObstacle && !noForward2 && xFront > minObstacle)
        {
            robot.moveForward(100);
        }
        else if (xLeft < minObstacle)
        {
            noForward2 = true;
            robot.moveBackward(100);
        }
    }
    else if (movement.rotate)
    {
        if (xLeft > minObstacle && xRight > minObstacle)
        {
            // PID for velocities of the wheels
            robot.rotation(dutyCicle);
        }
    }
    else if (movement.stop)
    {
        robot.stop();
    }
    else
    {
        robot.stop();
    }
}