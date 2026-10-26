#include "nncar/line.h"

#include <string.h>

static float relu(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

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

void nncar_line_nn_eval(const nncar_line_nn_t *nn,
                        const float x[NNCAR_LINE_INPUT_DIM], float y[2])
{
    float h1[NNCAR_LINE_H1];
    float h2[NNCAR_LINE_H2];
    float h3[NNCAR_LINE_H3];

    for (int i = 0; i < NNCAR_LINE_H1; ++i) {
        float acc = nn->b1[i];
        for (int j = 0; j < NNCAR_LINE_INPUT_DIM; ++j) {
            acc += nn->w1[i][j] * x[j];
        }
        h1[i] = relu(acc);
    }

    for (int i = 0; i < NNCAR_LINE_H2; ++i) {
        float acc = nn->b2[i];
        for (int j = 0; j < NNCAR_LINE_H1; ++j) {
            acc += nn->w2[i][j] * h1[j];
        }
        h2[i] = relu(acc);
    }

    for (int i = 0; i < NNCAR_LINE_H3; ++i) {
        float acc = nn->b3[i];
        for (int j = 0; j < NNCAR_LINE_H2; ++j) {
            acc += nn->w3[i][j] * h2[j];
        }
        h3[i] = relu(acc);
    }

    for (int k = 0; k < 2; ++k) {
        float acc = nn->b4[k];
        for (int j = 0; j < NNCAR_LINE_H3; ++j) {
            acc += nn->w4[k][j] * h3[j];
        }
        y[k] = (k == 0) ? 1.0f / (1.0f + expf(-acc)) : tanhf(acc);
    }
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

/* Deterministic PID teacher on lateral centroid offset (+ = line on the left). */
static nncar_twist_t pid_step(const nncar_profile_t *p,
                              nncar_line_state_t *st,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff,
                              float dt_s)
{
    float e = est->centroid_m;
    float de = 0.0f;

    if (st->has_prev && dt_s > 1.0e-6f) {
        de = (e - st->prev_centroid_m) / dt_s;
    }
    st->prev_centroid_m = e;
    st->has_prev = true;

    float fc = p->pid_d_filter_hz;
    float alpha = (fc > 0.0f) ? (1.0f - expf(-2.0f * NNCAR_PI * fc * dt_s)) : 1.0f;
    alpha = nncar_clampf(alpha, 0.0f, 1.0f);
    st->d_filtered += alpha * (de - st->d_filtered);

    float omega = p->pid_kp * e + p->pid_kd * st->d_filtered;

    float slow = 1.0f - p->curve_slow_factor *
                            fminf(1.0f, fabsf(omega) / fmaxf(p->omega_ref_radps, 1.0e-3f));
    slow = nncar_clampf(slow, 0.1f, 1.0f);

    nncar_twist_t cmd;
    cmd.v = fminf(v_desired, v_max_eff) * slow;
    cmd.omega = omega;
    return nncar_twist_envelope(p, cmd, v_max_eff);
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

    return nncar_twist_from_kappa(p, y[0], y[1], v_desired, v_max_eff);
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

    nncar_twist_t cmd;
    bool use_mlp = (st->ctrl == NNCAR_LINE_CTRL_MLP) && (st->nn != NULL) &&
                   (frame != NULL) && (frame->value != NULL);

    if (use_mlp) {
        cmd = mlp_step(p, frame, est, v_desired, v_max_eff, st->nn);
        if (!nncar_finite(cmd.v) || !nncar_finite(cmd.omega)) {
            use_mlp = false;   /* non-finite output: deterministic fallback */
        }
    }

    if (!use_mlp) {
        cmd = pid_step(p, st, est, v_desired, v_max_eff, dt_s);
    } else {
        /* Keep the PID derivative state warm so a mid-run fallback is smooth. */
        st->prev_centroid_m = est->centroid_m;
        st->has_prev = true;
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
