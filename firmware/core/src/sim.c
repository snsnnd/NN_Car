#include "nncar/sim.h"

#include <string.h>

void nncar_sim_init(nncar_sim_t *s, const nncar_profile_t *p,
                    float x0, float y0, float theta0, float v_rail)
{
    memset(s, 0, sizeof(*s));

    s->x = x0;
    s->y = y0;
    s->theta = theta0;
    s->v_rail = v_rail;

    s->line_width_m = 0.02f;
    s->track_amp_m = 0.0f;
    s->track_wavelength_m = 1.0f;

    s->wheel_radius_m = p->wheel_radius_m;
    s->track_width_m = p->track_width_m;
    s->inertia_kgm2 = 2.0e-5f;
    s->damping_nms = 4.0e-4f;
    s->torque_gain_nm = s->damping_nms * (p->v_max_nom_mps / p->wheel_radius_m);

    s->rng = 0x9e3779b9u;
    s->gray_valid_prob = 1.0f;
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        s->gray_gain[i] = 1.0f;
    }
}

void nncar_sim_seed(nncar_sim_t *s, uint32_t seed)
{
    s->rng = (seed != 0u) ? seed : 0x9e3779b9u;
}

static float sim_rand(nncar_sim_t *s)
{
    uint32_t x = s->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->rng = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

float nncar_sim_track_y(float x, const nncar_sim_t *s)
{
    if (s->track_amp_m == 0.0f) {
        return 0.0f;
    }
    return s->track_amp_m * sinf(2.0f * NNCAR_PI * x / s->track_wavelength_m);
}

void nncar_sim_read_gray(nncar_sim_t *s, const nncar_profile_t *p,
                         float value[NNCAR_MAX_SENSORS],
                         uint8_t exist[NNCAR_MAX_SENSORS],
                         uint8_t valid[NNCAR_MAX_SENSORS])
{
    float fwd_x = cosf(s->theta);
    float fwd_y = sinf(s->theta);
    float left_x = -sinf(s->theta);
    float left_y = cosf(s->theta);

    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        if (i >= p->sensor_count || s->gray_missing[i] != 0u) {
            value[i] = 0.0f;
            exist[i] = 0u;
            valid[i] = 0u;
            continue;
        }

        float y_i = p->sensor_position_m[i];
        float sx = s->x + fwd_x * p->sensor_to_axle_m + left_x * y_i;
        float sy = s->y + fwd_y * p->sensor_to_axle_m + left_y * y_i;

        float d = sy - nncar_sim_track_y(sx, s);
        float t = fabsf(d) / (0.5f * s->line_width_m);
        float g;

        if (t < 1.0f) {
            g = 1.0f;
        } else {
            g = fmaxf(-1.0f, 1.0f - 2.0f * (t - 1.0f));
        }

        g = g * s->gray_gain[i] + s->gray_offset;
        if (s->gray_noise_std > 0.0f) {
            g += s->gray_noise_std * (2.0f * sim_rand(s) - 1.0f);
        }

        value[i] = nncar_clampf(g, -1.0f, 1.0f);
        exist[i] = 1u;
        valid[i] = (s->gray_valid_prob >= 1.0f ||
                    sim_rand(s) < s->gray_valid_prob) ? 1u : 0u;
    }
}

static float wheel_speed_step(float w, float u, const nncar_sim_t *s,
                              const nncar_profile_t *p, float dt_s)
{
    /* Actuator deadband: the plant sees no torque until |pwm| exceeds db. */
    float db = nncar_clampf(p->motor_deadband, 0.0f, 0.5f);
    float mag = fabsf(u);
    float drive = 0.0f;

    if (mag > db) {
        drive = (mag - db) / (1.0f - db);
        drive = (u > 0.0f) ? drive : -drive;
    }

    float u_eff = drive * s->v_rail / p->rail_nominal_v;
    float torque = s->torque_gain_nm * u_eff - s->damping_nms * w;

    if (s->motor_stiction_nm > 0.0f) {
        if (w == 0.0f) {
            /* Breakaway: the wheel stays put until the drive beats stiction. */
            if (fabsf(torque) <= s->motor_stiction_nm) {
                return 0.0f;
            }
            torque -= (torque > 0.0f) ? s->motor_stiction_nm : -s->motor_stiction_nm;
        } else {
            torque -= (w > 0.0f) ? s->motor_stiction_nm : -s->motor_stiction_nm;
        }
    }

    return w + torque / s->inertia_kgm2 * dt_s;
}

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s)
{
    s->w_left = wheel_speed_step(s->w_left, pwm.left, s, p, dt_s);
    s->w_right = wheel_speed_step(s->w_right, pwm.right, s, p, dt_s);

    float v_left = s->w_left * s->wheel_radius_m;
    float v_right = s->w_right * s->wheel_radius_m;

    float v = 0.5f * (v_left + v_right);
    float omega = (v_right - v_left) / fmaxf(s->track_width_m, 1.0e-3f);

    s->x += v * cosf(s->theta) * dt_s;
    s->y += v * sinf(s->theta) * dt_s;
    s->theta += omega * dt_s;
}
