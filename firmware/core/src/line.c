#include "nncar/line.h"

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

void nncar_line_reset(nncar_line_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->mode = NNCAR_LINE_TRACKING;
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

nncar_twist_t nncar_line_step(const nncar_profile_t *p,
                              nncar_line_state_t *st,
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

    /* PID teacher on lateral centroid offset (+ = line on the left). */
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
    cmd = nncar_twist_envelope(p, cmd, v_max_eff);

    if (st->ramp_s < p->recovery_ramp_s) {
        st->ramp_s += dt_s;
        float k = nncar_clampf(st->ramp_s / fmaxf(p->recovery_ramp_s, 1.0e-3f), 0.0f, 1.0f);
        cmd.v = (1.0f - k) * st->recovery_cmd.v + k * cmd.v;
        cmd.omega = (1.0f - k) * st->recovery_cmd.omega + k * cmd.omega;
        cmd = nncar_twist_envelope(p, cmd, v_max_eff);
    }

    return cmd;
}
