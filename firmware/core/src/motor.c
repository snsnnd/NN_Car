#include "nncar/motor.h"

#include <string.h>

void nncar_motor_reset(nncar_motor_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

float nncar_deadband_compensate(const nncar_profile_t *p, float u)
{
    float db = nncar_clampf(p->motor_deadband, 0.0f, 0.5f);
    float mag = fabsf(u);

    if (mag < 1.0e-6f) {
        return 0.0f;
    }

    float compensated = db + (1.0f - db) * fminf(mag, 1.0f);
    return (u > 0.0f) ? compensated : -compensated;
}

float nncar_pi_step(const nncar_profile_t *p,
                    nncar_motor_state_t *st,
                    float target_mps,
                    float actual_mps,
                    float dt_s,
                    float v_max_eff,
                    bool enable)
{
    if (!enable) {
        st->integral = 0.0f;
        st->u = 0.0f;
        st->enable_prev = false;
        return 0.0f;
    }

    float e = target_mps - actual_mps;
    float u_ff = (v_max_eff > 1.0e-3f) ? (p->motor_kff * target_mps / v_max_eff) : 0.0f;

    float integral = st->integral + e * dt_s;
    float u_unsat = u_ff + p->motor_kp * e + p->motor_ki * integral;
    float u_sat = nncar_clampf(u_unsat, -1.0f, 1.0f);

    /* Back-calculation: pull the integrator back by the saturation excess. */
    float tt = fmaxf(p->motor_tt_s, 1.0e-4f);
    float corrected = integral + (u_sat - u_unsat) * dt_s / tt;
    st->integral = nncar_clampf(corrected, -p->motor_i_limit, p->motor_i_limit);

    st->u = u_sat;
    st->enable_prev = true;
    return st->u;
}
