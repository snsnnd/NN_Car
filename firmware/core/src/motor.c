#include "nncar/motor.h"

#include "nncar/nn.h"

#include <math.h>
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

void nncar_motor_nn_zero(nncar_motor_nn_t *nn)
{
    memset(nn, 0, sizeof(*nn));
}

float nncar_motor_nn_eval(const nncar_motor_nn_t *nn,
                          const float x[NNCAR_MOTOR_INPUT_DIM])
{
    const nncar_nn_layer_t layers[4] = {
        {NNCAR_MOTOR_INPUT_DIM, NNCAR_MOTOR_H1, NNCAR_NN_ACT_RELU, &nn->w1[0][0], nn->b1},
        {NNCAR_MOTOR_H1, NNCAR_MOTOR_H2, NNCAR_NN_ACT_RELU, &nn->w2[0][0], nn->b2},
        {NNCAR_MOTOR_H2, NNCAR_MOTOR_H3, NNCAR_NN_ACT_RELU, &nn->w3[0][0], nn->b3},
        {NNCAR_MOTOR_H3, 1, NNCAR_NN_ACT_NONE, &nn->w4[0][0], nn->b4},
    };
    const nncar_nn_model_t model = {layers, 4};
    float scratch[2 * NNCAR_MOTOR_H1];
    float z[1];

    nncar_nn_forward(&model, x, z, scratch);

    return tanhf(z[0]);
}

void nncar_motor_nn_input(const nncar_profile_t *p,
                          const nncar_motor_input_t *in,
                          float u_pi,
                          float previous_pwm,
                          float x[NNCAR_MOTOR_INPUT_DIM])
{
    float v_ref = fmaxf(p->v_max_nom_mps, 1.0e-3f);

    x[0] = nncar_clampf(in->target_mps / v_ref, -2.0f, 2.0f);
    x[1] = nncar_clampf(in->actual_mps / v_ref, -2.0f, 2.0f);
    x[2] = nncar_clampf(previous_pwm, -1.0f, 1.0f);
    x[3] = nncar_clampf(u_pi, -1.0f, 1.0f);
    x[4] = nncar_clampf(in->rail_ratio, p->rail_scale_min, p->rail_scale_max);
    x[5] = in->rail_exist ? 1.0f : 0.0f;
    x[6] = in->rail_valid ? 1.0f : 0.0f;
}

nncar_motor_out_t nncar_motor_step(const nncar_profile_t *p,
                                   nncar_motor_state_t *st,
                                   const nncar_motor_cfg_t *cfg,
                                   const nncar_motor_input_t *in,
                                   float dt_s)
{
    nncar_motor_out_t out;
    memset(&out, 0, sizeof(out));

    if (!in->enable) {
        st->integral = 0.0f;
        st->u = 0.0f;
        st->applied_pwm = 0.0f;
        st->enable_prev = false;
        return out;
    }

    /* Standalone policy: no PI, no integrator, no memory. Falls back to PI+FF
     * when the network is absent, which is the only safe degradation here. */
    if (cfg->mode == NNCAR_MOTOR_MODE_NN_DIRECT && cfg->nn != NULL) {
        float x[NNCAR_MOTOR_INPUT_DIM];
        nncar_motor_nn_input(p, in, 0.0f, st->applied_pwm, x);
        float t = nncar_motor_nn_eval(cfg->nn, x);
        float u = nncar_finite(t) ? nncar_clampf(t, -1.0f, 1.0f) : 0.0f;

        st->integral = 0.0f;
        st->u = u;
        st->applied_pwm = nncar_deadband_compensate(p, u);
        st->enable_prev = true;

        out.u_pi = 0.0f;
        out.u_nn = u;
        out.u_cmd = u;
        out.pwm = st->applied_pwm;
        return out;
    }

    const bool feedforward = cfg->mode != NNCAR_MOTOR_MODE_PI;

    float error = in->target_mps - in->actual_mps;
    float u_ff = 0.0f;

    if (feedforward && in->v_max_eff > 1.0e-3f) {
        u_ff = p->motor_kff * in->target_mps / in->v_max_eff;
    }

    float integral = st->integral + error * dt_s;
    float u_unsat = u_ff + p->motor_kp * error + p->motor_ki * integral;
    float u_pi = nncar_clampf(u_unsat, -1.0f, 1.0f);

    float u_nn = 0.0f;

    if (cfg->mode == NNCAR_MOTOR_MODE_PI_RESIDUAL && cfg->nn != NULL) {
        float x[NNCAR_MOTOR_INPUT_DIM];
        nncar_motor_nn_input(p, in, u_pi, st->applied_pwm, x);
        float residual = nncar_motor_nn_eval(cfg->nn, x);
        if (nncar_finite(residual)) {
            u_nn = p->motor_alpha * nncar_clampf(residual, -1.0f, 1.0f);
        }
    }

    float u_cmd = nncar_clampf(u_pi + u_nn, -1.0f, 1.0f);

    /* Back-calculation against the applied command, not the raw integrator. */
    float tt = fmaxf(p->motor_tt_s, 1.0e-4f);
    float corrected = integral + (u_cmd - u_unsat) * dt_s / tt;
    st->integral = nncar_clampf(corrected, -p->motor_i_limit, p->motor_i_limit);

    out.u_pi = u_pi;
    out.u_nn = u_nn;
    out.u_cmd = u_cmd;
    out.pwm = feedforward ? nncar_deadband_compensate(p, u_cmd) : u_cmd;

    st->u = u_cmd;
    st->applied_pwm = out.pwm;
    st->enable_prev = true;

    return out;
}
