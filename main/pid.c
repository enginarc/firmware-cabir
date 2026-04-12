#include "pid.h"
#include <math.h>

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_max, float dt_s)
{
    p->kp           = kp;
    p->ki           = ki;
    p->kd           = kd;
    p->output_min   = out_min;
    p->output_max   = out_max;
    p->integral_max = integral_max;
    p->dt_s         = dt_s;
    p->integral     = 0.0f;
    p->prev_error   = 0.0f;
}

float pid_compute(pid_t *p, float setpoint, float measured)
{
    float error      = setpoint - measured;
    float derivative = (error - p->prev_error) / p->dt_s;

    /* Integrate with anti-windup clamp */
    p->integral += error * p->dt_s;
    p->integral  = clamp(p->integral, -p->integral_max, p->integral_max);

    float output = p->kp * error
                 + p->ki * p->integral
                 + p->kd * derivative;

    p->prev_error = error;
    return clamp(output, p->output_min, p->output_max);
}

void pid_reset(pid_t *p)
{
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
}
