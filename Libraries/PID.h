#ifndef PID_H
#define PID_H
// Extracted from: https://github.com/pms67/PID
typedef struct PID_CONTROL
{

    /* Controller gains */
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;

    /* Derivative low-pass filter time constant */
    float tau = 0;

    /* Output limits */
    float limMin = 0;
    float limMax = 0;

    /* Integrator limits */
    float limMinInt = 0;
    float limMaxInt = 0;

    /* Sample time (in seconds) */
    float T = 0;

    /* Controller "memory" */
    float integrator = 0;
    float prevError = 0; /* Required for integrator */
    float differentiator = 0;
    float prevMeasurement = 0; /* Required for differentiator */

    /* Controller output */
    float out = 0;
};

void PIDController_Init(PID_CONTROL &pid);
float PIDController_Update(PID_CONTROL &pid, float setpoint, float measurement);
float PIController_Update(PID_CONTROL &pid, float setpoint, float measurement);
#endif