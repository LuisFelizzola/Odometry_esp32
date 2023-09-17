#include <Navigation.h>
void Navigation::positionPID(PID_CONTROL &pid, Robot &robot, float Lx, float Ly, float r, long ct, long prevT, float kp, float ki, float kd, float tao)
{
    float deltaT = ((float)(ct - prevT)) / 1.0e6;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.tau = tao;
    pid.T = deltaT;
    float Ux = 0, Uy = 0, Uw = 0;
    if (movement.moveF || movement.moveB)
    { // si el robot se esta movilizano el eje x
        // AJUSTO LOS LIMITES DEL INTEGRADOR (PARA EVITAR EL WINDUP), CON LA CONVENCION DEL 70% DE LOS LIMITES FISICOS
        pid.limMaxInt = 0.7 * Vxmax;
        pid.limMinInt = 0.7 * Vxmin;
        pid.limMax = Vxmax;
        pid.limMin = Vxmin;
        Ux = PIDController_Update(pid, XT, Xi);
    }
    else if (movement.moveR || movement.moveL)
    { // si el robot se esta moviendo en el eje Y
        pid.limMaxInt = 0.7 * Vymax;
        pid.limMinInt = 0.7 * Vymin;
        pid.limMax = Vymax;
        pid.limMin = Vymin;
        Uy = PIDController_Update(pid, YT, Yi);
    }
    else if (movement.moveR)
    { // si el robot esta en rotacion pura
        pid.limMaxInt = 0.7 * Wmax;
        pid.limMinInt = 0.7 * Wmin;
        pid.limMax = Wmax;
        pid.limMin = Wmin;
        Uw = PIDController_Update(pid, TETA_T, Tetai);
    }

    inverseKinematics(Uy, Ux, r, Lx, Ly, Uw);
}
void Navigation::wheelVelocityPID(PID_CONTROL &pid, Robot &robot, long ct, long prevT, float kp, float ki)
{
    float deltaT = ((float)(ct - prevT)) / 1.0e6;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.T = deltaT;
    float U = 0;
    // limites del integrador y de la señal de control
    pid.limMaxInt = 0.7 * maxDuty;
    pid.limMinInt = 0.7 * -maxDuty;
    pid.limMax = maxDuty;
    pid.limMin = -maxDuty;
    // aplico el PID con respecto a la velocidad w1, debido a la aproximacion y que tomo en cuenta el mejor encoder
    U = PIController_Update(pid, w1T, robot.wheel_1->Velocityradians());
    // Me aseguro que el valor es positivo
    dutyCycle = (int)abs(U); // me aseguro que el valor es positivo
}
void Navigation::inverseKinematics(float vy, float vx, float r, float lx, float ly, float w)
{

    w1T = 1 / r + (vx - vy - (lx + ly) * w);
    w2T = 1 / r + (vx + vy + (lx + ly) * w);
    w3T = 1 / r + (vx + vy - (lx + ly) * w);
    w4T = 1 / r + (vx - vy + (lx + ly) * w);
}
// Defino los posibles movimientos para alcanzar mi objetivo
void Navigation::MovementPlanning(Robot &robot, float minX, float minY, float minT)
{
    uint8_t d = 0; // contador para verificar si el robot ha llegado a su destino
    Xi = robot.posX();
    Yi = robot.posY();
    Tetai = robot.posA();

    float dx = XT - Xi;           // DESPLAZAMIENTO EN X
    float dy = YT - Yi;           // DESPLAZAMIENTO EN Y
    float dTeta = TETA_T - Tetai; // desplazamiento del ángulo teta

    if (dx > 0 && XT > 0)
    { // si el desplazamiento es una avance (1.5cm in)
        movement.moveF = true;
        movement.moveB = false;
    }
    else if (dx < 0 && XT < 0)
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

    if (dy > 0 && YT > 0)
    {
        movement.moveR = true;
        movement.moveL = false;
    }
    else if (dy < 0 && YT < 0)
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
    if (TETA_T > Tetai)
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
    if ((disable))
    {
        movement.stop = true;
    }
    else
    {
        movement.stop = false;
    }
}

void Navigation::Navigate(Robot &r, float xT, float yT, float tetaT, float disFront, float disRight, float disLeft, bool disA = STOP)
{
    // X,Y AND TETA TARGET:
    XT = xT;
    YT = yT;
    TETA_T = tetaT;
    xFront = disFront;
    xLeft = disLeft;
    xRight = disRight;
    if (disA)
    {
        disable = true;
    }
    else
    {
        disable = false;
    }
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
            robot.moveForward(dutyCycle);
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

        robot.moveBackward(dutyCycle);
    }
    else if (movement.moveR)
    {
        if (xRight > minObstacle)
        {
            // PID for velocities of the wheels
            robot.moveRight(dutyCycle);
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
            robot.moveLeft(dutyCycle);
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
            robot.rotation(dutyCycle);
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
float Navigation::showX()
{
    return Xi;
}
float Navigation::showY()
{
    return Yi;
}