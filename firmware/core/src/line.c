#include "nncar/line.h"

#include "nncar/nn.h"

#include <string.h>

static float omega_limit(const nncar_profile_t *p, float v, float v_max_eff)
{
    if (v <= 1.0e-3f || p->track_width_m <= 1.0e-6f) {
        return 0.0f;
    }

    float kin = 2.0f * (v_max_eff - v) / p->track_width_m;
    float lat = p->a_lat_max_mps2 / fmaxf(v, 0.05f);
    float limit = fminf(kin, lat);

    return (limit > 0.0f) ? limit : 0.0f;
}

nncar_twist_t nncar_twist_envelope(const nncar_profile_t *p,
                                   nncar_twist_t twist,
                                   float v_max_eff)
{
    nncar_twist_t out;

    out.v = nncar_clampf(twist.v, 0.0f, fmaxf(v_max_eff, 0.0f));

    if (out.v <= 1.0e-4f) {
        out.omega = 0.0f;
        return out;
    }

    float limit = omega_limit(p, out.v, v_max_eff);
    out.omega = nncar_clampf(twist.omega, -limit, limit);
    return out;
}

nncar_twist_t nncar_twist_from_kappa(const nncar_profile_t *p,
                                     float s_v,
                                     float kappa_norm,
                                     float v_desired,
                                     float v_max_eff)
{
    nncar_twist_t twist;

    float s = nncar_clampf(s_v, 0.0f, 1.0f);
    float kn = nncar_clampf(kappa_norm, -1.0f, 1.0f);

    twist.v = s * fmaxf(v_desired, 0.0f);

    if (twist.v <= 1.0e-4f) {
        twist.omega = 0.0f;
        return twist;
    }

    float kappa = kn * p->kappa_ref;
    twist.omega = kappa * twist.v;
    return nncar_twist_envelope(p, twist, v_max_eff);
}

void nncar_line_nn_zero(nncar_line_nn_t *nn)
{
    memset(nn, 0, sizeof(*nn));
}

void nncar_line_nn_input(const nncar_profile_t *p,
                         const nncar_gray_frame_t *frame,
                         const nncar_line_estimate_t *est,
                         float v_desired,
                         float x[NNCAR_LINE_INPUT_DIM])
{
    const float pos_ref = fmaxf(p->position_ref_m, 1.0e-6f);
    const float axle_ref = fmaxf(p->axle_ref_m, 1.0e-6f);
    const float speed_ref = fmaxf(p->speed_ref_mps, 1.0e-6f);

    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        /* Mask first, then clear the value: invalid sensors contribute 0. */
        bool usable = frame->exist[i] != 0u && frame->valid[i] != 0u;
        x[i] = usable ? nncar_clampf(frame->value[i], -1.0f, 1.0f) : 0.0f;
        x[16 + i] = (frame->exist[i] != 0u) ? 1.0f : 0.0f;
        x[32 + i] = (frame->valid[i] != 0u) ? 1.0f : 0.0f;
        x[48 + i] = nncar_clampf(p->sensor_position_m[i] / pos_ref, -1.0f, 1.0f);
    }

    x[64] = nncar_clampf(p->sensor_to_axle_m / axle_ref, -1.0f, 1.0f);
    x[65] = nncar_clampf(v_desired / speed_ref, 0.0f, 1.0f);
    x[66] = nncar_clampf(est->confidence, 0.0f, 1.0f);
    x[67] = nncar_clampf(est->valid_count_norm, 0.0f, 1.0f);
}

void nncar_line_history_update(const nncar_profile_t *p,
                               nncar_line_state_t *st,
                               const nncar_line_estimate_t *est,
                               float dt_s)
{
    const float e = est->centroid_m;
    float de = 0.0f;

    if (st->has_prev && dt_s > 1.0e-6f) {
        de = (e - st->prev_centroid_m) / dt_s;
    }

    float fc = p->pid_d_filter_hz;
    float alpha = (fc > 0.0f) ? (1.0f - expf(-2.0f * NNCAR_PI * fc * dt_s)) : 1.0f;
    alpha = nncar_clampf(alpha, 0.0f, 1.0f);
    st->d_filtered += alpha * (de - st->d_filtered);

    st->history_centroid_m = st->has_prev ? st->prev_centroid_m : e;
    st->prev_centroid_m = e;
    st->has_prev = true;
}

void nncar_line_nn_eval(const nncar_line_nn_t *nn,
                        const float x[NNCAR_LINE_INPUT_DIM], float y[2])
{
    /*
     * The runtime carries the three ReLU hidden layers; the head stays here
     * because s_v and kappa need different activations.
     */
    const nncar_nn_layer_t layers[4] = {
        {NNCAR_LINE_INPUT_DIM, NNCAR_LINE_H1, NNCAR_NN_ACT_RELU, &nn->w1[0][0], nn->b1},
        {NNCAR_LINE_H1, NNCAR_LINE_H2, NNCAR_NN_ACT_RELU, &nn->w2[0][0], nn->b2},
        {NNCAR_LINE_H2, NNCAR_LINE_H3, NNCAR_NN_ACT_RELU, &nn->w3[0][0], nn->b3},
        {NNCAR_LINE_H3, 2, NNCAR_NN_ACT_NONE, &nn->w4[0][0], nn->b4},
    };
    const nncar_nn_model_t model = {layers, 4};
    float scratch[2 * NNCAR_LINE_H1];
    float z[2];

    nncar_nn_forward(&model, x, z, scratch);

    y[0] = 1.0f / (1.0f + expf(-z[0]));
    y[1] = tanhf(z[1]);
}

void nncar_line_reset(nncar_line_state_t *st)
{
    nncar_line_ctrl_t ctrl = st->ctrl;
    const nncar_line_nn_t *nn = st->nn;

    memset(st, 0, sizeof(*st));
    st->mode = NNCAR_LINE_TRACKING;
    st->ctrl = ctrl;
    st->nn = nn;
}

void nncar_line_set_ctrl(nncar_line_state_t *st, nncar_line_ctrl_t ctrl,
                         const nncar_line_nn_t *nn)
{
    st->ctrl = ctrl;
    st->nn = nn;
}

static nncar_twist_t recovery_step(const nncar_profile_t *p,
                                   nncar_line_state_t *st,
                                   float t_s,
                                   float v_max_eff,
                                   float dt_s)
{
    nncar_twist_t cmd;

    st->recovery_time_s += dt_s;

    if (st->recovery_time_s > p->recovery_timeout_s || !st->has_last_centroid) {
        cmd.v = 0.0f;
        cmd.omega = 0.0f;
        return cmd;
    }

    float v_search = fminf(p->recovery_speed_mps, 0.4f * v_max_eff);
    float sign = (st->last_centroid_m >= 0.0f) ? 1.0f : -1.0f;
    float kappa = sign * p->recovery_kappa_scale * p->kappa_ref;

    cmd.v = v_search;
    cmd.omega = kappa * v_search;

    (void)t_s;
    return nncar_twist_envelope(p, cmd, v_max_eff);
}

/* Shared comfort policy: shed speed as the commanded yaw rate grows. */
static float curve_slow(const nncar_profile_t *p, float omega)
{
    float slow = 1.0f - p->curve_slow_factor *
                            fminf(1.0f, fabsf(omega) / fmaxf(p->omega_ref_radps, 1.0e-3f));
    return nncar_clampf(slow, 0.1f, 1.0f);
}

/* Deterministic PID teacher on lateral centroid offset (+ = line on the left). */
static nncar_twist_t pid_step(const nncar_profile_t *p,
                              const nncar_line_state_t *st,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff)
{
    float e = est->centroid_m;
    float omega = p->pid_kp * e + p->pid_kd * st->d_filtered;

    float slow = curve_slow(p, omega);

    nncar_twist_t cmd;
    cmd.v = fminf(v_desired, v_max_eff) * slow;
    cmd.omega = omega;
    return nncar_twist_envelope(p, cmd, v_max_eff);
}

float nncar_line_pid_kappa(const nncar_profile_t *p, const nncar_line_state_t *st,
                          const nncar_line_estimate_t *est, float v_desired,
                          float v_max_eff)
{
    const nncar_twist_t base = pid_step(p, st, est, v_desired, v_max_eff);

    if (base.v <= 1.0e-3f) {
        return 0.0f;
    }

    return nncar_clampf(base.omega / base.v / p->kappa_ref, -1.0f, 1.0f);
}

static nncar_twist_t residual_step(const nncar_profile_t *p,
                                   const nncar_gray_frame_t *frame,
                                   const nncar_line_estimate_t *est,
                                   nncar_line_state_t *st,
                                   float v_desired,
                                   float v_max_eff,
                                   const nncar_line_nn_t *nn)
{
    float x[NNCAR_LINE_INPUT_DIM];
    float y[2];

    nncar_line_nn_input(p, frame, est, v_desired, x);
    nncar_line_nn_eval(nn, x, y);

    /* Baseline in curvature space, at the PID's own speed. */
    const float kappa_pid = nncar_line_pid_kappa(p, st, est, v_desired, v_max_eff);
    const float kappa = nncar_clampf(kappa_pid + p->line_kappa_alpha * y[1],
                                     -1.0f, 1.0f);

    /* Speed still comes from the net, so only the baseline differs from MLP. */
    nncar_twist_t twist;
    twist.v = nncar_clampf(y[0], 0.0f, 1.0f) * fmaxf(v_desired, 0.0f);

    if (twist.v <= 1.0e-4f) {
        twist.omega = 0.0f;
        return twist;
    }

    float slow = curve_slow(p, kappa * p->kappa_ref * twist.v);
    twist.v *= slow;
    twist.omega = kappa * p->kappa_ref * twist.v;

    return nncar_twist_envelope(p, twist, v_max_eff);
}

void nncar_line_nn_eval_q(const nncar_nn_q_model_t *nn,
                          const float x[NNCAR_LINE_INPUT_DIM], float y[2])
{
    /* 2 x widest hidden layer + the quantized input buffer. */
    int8_t scratch[2 * NNCAR_LINE_H1 + NNCAR_LINE_INPUT_DIM];
    float z[2];

    nncar_nn_q_forward(nn, x, z, scratch);

    y[0] = 1.0f / (1.0f + expf(-z[0]));
    y[1] = tanhf(z[1]);
}

/* Shared tail: [s_v, kappa_norm] -> enveloped twist, with the loop gain. */
static nncar_twist_t mlp_apply(const nncar_profile_t *p, const float y[2],
                               float v_desired, float v_max_eff)
{
    /*
     * Per-sample MSE training makes the network return the conditional mean of
     * kappa, which under-shoots the amplitude the closed loop needs. The bench
     * calibrated loop gain restores it; 1.0 reproduces the raw network.
     */
    float kg = nncar_clampf(y[1] * p->line_kappa_gain, -1.0f, 1.0f);

    nncar_twist_t twist = nncar_twist_from_kappa(p, y[0], kg, v_desired, v_max_eff);

    /* Same comfort envelope as the PID: shed speed as the yaw rate grows. */
    float kappa = kg * p->kappa_ref;
    float slow = curve_slow(p, twist.omega);
    twist.v *= slow;
    twist.omega = kappa * twist.v;

    return nncar_twist_envelope(p, twist, v_max_eff);
}

static nncar_twist_t mlp_step(const nncar_profile_t *p,
                              const nncar_gray_frame_t *frame,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff,
                              const nncar_line_nn_t *nn)
{
    float x[NNCAR_LINE_INPUT_DIM];
    float y[2];

    nncar_line_nn_input(p, frame, est, v_desired, x);
    nncar_line_nn_eval(nn, x, y);

    return mlp_apply(p, y, v_desired, v_max_eff);
}

static nncar_twist_t mlp_step_q(const nncar_profile_t *p,
                                const nncar_gray_frame_t *frame,
                                const nncar_line_estimate_t *est,
                                float v_desired,
                                float v_max_eff,
                                const nncar_nn_q_model_t *nnq)
{
    float x[NNCAR_LINE_INPUT_DIM];
    float y[2];

    nncar_line_nn_input(p, frame, est, v_desired, x);
    nncar_line_nn_eval_q(nnq, x, y);

    return mlp_apply(p, y, v_desired, v_max_eff);
}

nncar_twist_t nncar_line_step(const nncar_profile_t *p,
                              nncar_line_state_t *st,
                              const nncar_gray_frame_t *frame,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff,
                              float t_s,
                              float dt_s)
{
    if (v_desired <= 0.0f) {
        nncar_twist_t zero = {0.0f, 0.0f};
        return zero;
    }

    if (est->line_found) {
        st->last_centroid_m = est->centroid_m;
        st->has_last_centroid = true;
    }

    /* Hysteretic handover between tracking and recovery. */
    if (st->mode == NNCAR_LINE_TRACKING) {
        if (est->confidence < p->conf_enter_low) {
            st->low_hold_s += dt_s;
            if (st->low_hold_s >= p->conf_enter_hold_s) {
                st->mode = NNCAR_LINE_RECOVERY;
                st->recovery_time_s = 0.0f;
                st->high_hold_s = 0.0f;
                st->transitions += 1;
            }
        } else {
            st->low_hold_s = 0.0f;
        }
    } else {
        if (est->confidence > p->conf_exit_high) {
            st->high_hold_s += dt_s;
            if (st->high_hold_s >= p->conf_exit_hold_s) {
                st->mode = NNCAR_LINE_TRACKING;
                st->ramp_s = 0.0f;
                st->low_hold_s = 0.0f;
                st->has_prev = false;
                st->transitions += 1;
            }
        } else {
            st->high_hold_s = 0.0f;
        }
    }

    if (st->mode == NNCAR_LINE_RECOVERY) {
        st->recovery_cmd = recovery_step(p, st, t_s, v_max_eff, dt_s);
        return st->recovery_cmd;
    }

    nncar_line_history_update(p, st, est, dt_s);

    nncar_twist_t cmd = {0.0f, 0.0f};
    const bool net_ready = (st->nn != NULL) && (frame != NULL) &&
                           (frame->value != NULL);
    const bool use_direct = (st->ctrl == NNCAR_LINE_CTRL_MLP) && net_ready;
    const bool use_residual = (st->ctrl == NNCAR_LINE_CTRL_MLP_RESIDUAL) && net_ready;
    const bool use_int8 = (st->ctrl == NNCAR_LINE_CTRL_MLP_INT8) &&
                          (nncar_line_nn_int8_trained() != NULL) &&
                          (frame != NULL) && (frame->value != NULL);

    if (use_direct) {
        cmd = mlp_step(p, frame, est, v_desired, v_max_eff, st->nn);
    } else if (use_residual) {
        cmd = residual_step(p, frame, est, st, v_desired, v_max_eff, st->nn);
    } else if (use_int8) {
        cmd = mlp_step_q(p, frame, est, v_desired, v_max_eff,
                         nncar_line_nn_int8_trained());
    }

    if ((!use_direct && !use_residual && !use_int8) || !nncar_finite(cmd.v) ||
        !nncar_finite(cmd.omega)) {
        /* Deterministic fallback; the derivative state is shared, so it is warm. */
        cmd = pid_step(p, st, est, v_desired, v_max_eff);
    }

    if (st->ramp_s < p->recovery_ramp_s) {
        st->ramp_s += dt_s;
        float k = nncar_clampf(st->ramp_s / fmaxf(p->recovery_ramp_s, 1.0e-3f), 0.0f, 1.0f);
        cmd.v = (1.0f - k) * st->recovery_cmd.v + k * cmd.v;
        cmd.omega = (1.0f - k) * st->recovery_cmd.omega + k * cmd.omega;
        cmd = nncar_twist_envelope(p, cmd, v_max_eff);
    }

    return cmd;
}
