// funcion para incrementar la posicion de cada rueda cuando el flanco del encoder es activado
struct odometry
{

    float x = 0;
    float y = 0;
    float teta = 0;
};
int incrementarPos(int b, int posI)
{
    int i = 0;
    if (b > 0)
    { // si el motor se mueve counter clockwise
        i = 1;
    }
    else
    {
        i = -1;
    }
    return posI + i; // retorno el valor de la posición inicial más el incremento
}

float calculateW(long ct, int prevPos, int pos, long prevT, float &vFlt, float &vPrev)
{
    float vel = 0;
    float deltaT = ((float)(ct - prevT)) / 1.0e6; // paso el dx a s
    vel = (pos - prevPos) / deltaT;

    // aplico el filtro pasa-bajas
    vFlt = 0.854 * vFlt + 0.0728 * vel + 0.0728 * vPrev;
    vPrev = vel; // velocidad previa
    return vFlt;
}

void forwardKinematics(struct odometry &o, float w1, float w2, float w3, float w4, float r, float Lx, float Ly, long prevT, long ct)
{
    float Vx = (w1 + w2 + w3 + w4) * r / 4;  // Velocidad longitudinal del robot
    float Vy = (-w1 + w2 + w3 - w4) * r / 4; // velocidad vertical del robot
    float wz = (-w1 + w2 - w3 + w4) * (r / 4 * (Lx + Ly));
    float deltaT = ((float)(ct - prevT)) / 1.0e6;

    o.x = o.x + Vx * deltaT;
    // return o.x;
    o.y = o.y + Vy * deltaT;
    o.teta = o.teta + wz * deltaT;
}
