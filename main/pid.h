#pragma once
#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_max;
    float dt_s;          /* Sample period in seconds */
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max,
               float integral_max, float dt_s);

/*
 * Compute PID output given setpoint and measured value.
 * Returns clamped output in [out_min, out_max].
 */
float pid_compute(pid_t *p, float setpoint, float measured);

/* Reset integrator and derivative state (call on mode change) */
void  pid_reset(pid_t *p);
