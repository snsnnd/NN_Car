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
    s->plant_motor_deadband = p->motor_deadband;
    s->body_mass_kg = 0.50f;
    s->yaw_inertia_kgm2 = 1.0e-3f;
    s->tire_mu = 0.9f;
    s->tire_stiffness_n_per_mps = 30.0f;
    s->longitudinal_drag_n_per_mps = 0.10f;
    s->yaw_drag_nm_per_radps = 2.0e-3f;

    s->rng = 0x9e3779b9u;
    s->gray_valid_prob = 1.0f;
    s->gray_width_scale = 1.0f;
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        s->gray_gain[i] = 1.0f;
    }
}

void nncar_sim_seed(nncar_sim_t *s, uint32_t seed)
{
    s->rng = (seed != 0u) ? seed : 0x9e3779b9u;
}

void nncar_sim_set_map(nncar_sim_t *s, const nncar_map_t *map)
{
    s->map = map;
    if (map != NULL && map->line_width_m > 0.0f) {
        s->line_width_m = map->line_width_m;
        s->track_amp_m = 0.0f;
        s->track_extra_amp_m[0] = 0.0f;
        s->track_extra_amp_m[1] = 0.0f;
    }
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

static float sim_normal(nncar_sim_t *s)
{
    float u1 = fmaxf(sim_rand(s), 1.0e-7f);
    float u2 = sim_rand(s);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * NNCAR_PI * u2);
}

void nncar_sim_track_geometry(float x, const nncar_sim_t *s,
                              float *y, float *dy_dx, float *d2y_dx2)
{
    float y_value = 0.0f;
    float slope = 0.0f;
    float curvature = 0.0f;
    if (s->track_amp_m != 0.0f) {
        float phase = 2.0f * NNCAR_PI * x / s->track_wavelength_m;
        float k = 2.0f * NNCAR_PI / s->track_wavelength_m;
        y_value += s->track_amp_m * sinf(phase);
        slope += s->track_amp_m * k * cosf(phase);
        curvature -= s->track_amp_m * k * k * sinf(phase);
    }
    for (int i = 0; i < 2; ++i) {
        if (s->track_extra_amp_m[i] == 0.0f) {
            continue;
        }
        float k = 2.0f * NNCAR_PI /
                  fmaxf(s->track_extra_wavelength_m[i], 1.0e-3f);
        float phase = k * x + s->track_extra_phase_rad[i];
        y_value += s->track_extra_amp_m[i] * sinf(phase);
        slope += s->track_extra_amp_m[i] * k * cosf(phase);
        curvature -= s->track_extra_amp_m[i] * k * k * sinf(phase);
    }
    if (y != NULL) {
        *y = y_value;
    }
    if (dy_dx != NULL) {
        *dy_dx = slope;
    }
    if (d2y_dx2 != NULL) {
        *d2y_dx2 = curvature;
    }
}

float nncar_sim_track_y(float x, const nncar_sim_t *s)
{
    float y = 0.0f;
    nncar_sim_track_geometry(x, s, &y, NULL, NULL);
    return y;
}

float nncar_sim_lateral_error(const nncar_sim_t *s)
{
    if (s->map != NULL) {
        float lateral = 0.0f;
        (void)nncar_map_query(s->map, s->x, s->y, &lateral, NULL);
        return lateral;
    }
    return s->y - nncar_sim_track_y(s->x, s);
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

        float d = s->map != NULL ?
            nncar_map_query(s->map, sx, sy, NULL, NULL) :
            sy - nncar_sim_track_y(sx, s);
        float sensor_width = 0.5f * s->line_width_m;
        if (s->gray_width_scale > 0.0f && s->gray_width_scale != 1.0f) {
            sensor_width *= s->gray_width_scale;
        }
        float t = fabsf(d) / sensor_width;
        float g;

        if (s->realistic_dynamics) {
            /* Gaussian point-spread response over a white (-1) background. */
            g = 2.0f * expf(-0.5f * t * t) - 1.0f;
        } else if (t < 1.0f) {
            g = 1.0f;
        } else {
            g = fmaxf(-1.0f, 1.0f - 2.0f * (t - 1.0f));
        }

        g = g * s->gray_gain[i] + s->gray_offset;
        if (s->gray_noise_std > 0.0f) {
            g += s->gray_noise_std * (s->realistic_dynamics ? sim_normal(s)
                                                            : (2.0f * sim_rand(s) - 1.0f));
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

static float wheel_speed_step_realistic(float w, float u, const nncar_sim_t *s,
                                        const nncar_profile_t *p, float body_v,
                                        float dt_s, float *contact_force)
{
    float db = nncar_clampf(s->plant_motor_deadband, 0.0f, 0.5f);
    float mag = fabsf(u);
    float drive = 0.0f;
    if (mag > db) {
        drive = (mag - db) / (1.0f - db);
        drive = (u > 0.0f) ? drive : -drive;
    }

    float u_eff = drive * s->v_rail / p->rail_nominal_v;
    float motor_torque = s->torque_gain_nm * u_eff - s->damping_nms * w;
    float tire_limit = fmaxf(s->tire_mu, 0.0f) * fmaxf(s->body_mass_kg, 0.01f) *
                        9.80665f * 0.5f;
    float force = nncar_clampf((w * s->wheel_radius_m - body_v) *
                                   fmaxf(s->tire_stiffness_n_per_mps, 0.0f),
                               -tire_limit, tire_limit);
    float net_torque = motor_torque - force * s->wheel_radius_m;

    if (s->motor_stiction_nm > 0.0f) {
        if (fabsf(w) < 1.0e-5f && fabsf(net_torque) <= s->motor_stiction_nm) {
            net_torque = 0.0f;
        } else {
            net_torque -= (net_torque > 0.0f) ? s->motor_stiction_nm
                                               : -s->motor_stiction_nm;
        }
    }

    float next = w + net_torque / fmaxf(s->inertia_kgm2, 1.0e-8f) * dt_s;
    if (contact_force != NULL) {
        *contact_force = force;
    }
    return next;
}

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s)
{
    if (s->realistic_dynamics) {
        /* Tire slip is a stiff spring against the small wheel inertia. Use
         * substeps so the host's 2 ms motor tick does not numerically chatter. */
        const int substeps = 4;
        const float h = dt_s / (float)substeps;
        const float mass = fmaxf(s->body_mass_kg, 0.01f);
        const float yaw_inertia = fmaxf(s->yaw_inertia_kgm2, 1.0e-6f);
        for (int i = 0; i < substeps; ++i) {
            float force_left = 0.0f;
            float force_right = 0.0f;
            s->w_left = wheel_speed_step_realistic(s->w_left, pwm.left, s, p,
                                                   s->body_v_mps, h, &force_left);
            s->w_right = wheel_speed_step_realistic(s->w_right, pwm.right, s, p,
                                                    s->body_v_mps, h, &force_right);

            float accel = (force_left + force_right -
                           s->longitudinal_drag_n_per_mps * s->body_v_mps) / mass;
            float yaw_torque = (force_right - force_left) * 0.5f * s->track_width_m;
            float yaw_accel = (yaw_torque - s->yaw_drag_nm_per_radps *
                                              s->body_omega_radps) / yaw_inertia;
            s->body_v_mps += accel * h;
            s->body_omega_radps += yaw_accel * h;

            s->x += s->body_v_mps * cosf(s->theta) * h;
            s->y += s->body_v_mps * sinf(s->theta) * h;
            s->theta += s->body_omega_radps * h;
        }
        return;
    }

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
