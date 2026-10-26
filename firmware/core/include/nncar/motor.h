#ifndef NNCAR_MOTOR_H
#define NNCAR_MOTOR_H

#include "nncar/profile.h"

typedef struct {
    float integral;
    float u;
    bool enable_prev;
} nncar_motor_state_t;

void nncar_motor_reset(nncar_motor_state_t *st);

/* PI + feedforward with back-calculation anti-windup. u in [-1, 1]. */
float nncar_pi_step(const nncar_profile_t *p,
                    nncar_motor_state_t *st,
                    float target_mps,
                    float actual_mps,
                    float dt_s,
                    float v_max_eff,
                    bool enable);

/* Inverse deadband mapping applied by the actuator adapter. */
float nncar_deadband_compensate(const nncar_profile_t *p, float u);

#endif
