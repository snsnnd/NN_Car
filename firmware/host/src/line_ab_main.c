#include "nncar/line.h"
#include "nncar/profile.h"
#include "nncar/sim.h"
#include "nncar/stack.h"
#include "nncar/velocity.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    SRC_PID = 0,
    SRC_MLP = 1,
    SRC_ORACLE = 2,
    SRC_RESIDUAL = 3,
    SRC_INT8 = 4,
    SRC_R4 = 5
} src_t;

typedef struct {
    const char *name;
    float amp_m;
    float wavelength_m;
    float offset_m;
    float heading_deg;
    float v_desired;
    float rail_v;
    float noise_std;
    float gain_bias;
    float offset_bias;
    int missing_a;
    int missing_b;
    float valid_prob;
    float sensor_width_scale;
    float inertia_scale;
    float plant_deadband;
    float body_mass_scale;
    float yaw_inertia_scale;
    float grip_scale;
    bool random_track;
    uint32_t track_seed;
} scenario_t;

typedef struct {
    float lat_sq;
    int lat_n;
    float max_lat;
    float omega_sq;
    float domega_sq;
    float prev_omega;
    float speed_sum;
    int recovery_hits;
    float recovery_s;
    int out_n;
    int tick_n;
    uint32_t faults;
} metrics_t;

/* Pre-declared A/B suite: clean, curves, speed, and sensor perturbations. */
static const scenario_t k_suite[] = {
    {"straight", 0.00f, 1.60f, 0.020f, 3.0f, 0.90f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"gentle_s", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"sharp_s", 0.12f, 0.90f, 0.000f, 0.0f, 0.60f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"fast_gentle", 0.06f, 1.60f, 0.010f, 1.5f, 1.20f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"noise", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.08f, 0.12f, 0.05f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"dropout", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.02f, 0.00f, 0.00f, 2, 5, 0.90f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"rail_sag", 0.10f, 1.20f, 0.000f, 0.0f, 0.75f, 11.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
    {"disturbance", 0.00f, 1.60f, 0.030f, 5.0f, 1.20f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f, 1.00f, 1.00f, -1.0f, 1.0f, 1.0f, 1.0f, false, 0u},
};
#define SUITE_COUNT ((int)(sizeof(k_suite) / sizeof(k_suite[0])))

/* Start-up transient skipped by every metric. */
#define SETTLE_S 0.5f
#define DURATION_S 6.0f

static void metrics_init(metrics_t *m)
{
    memset(m, 0, sizeof(*m));
}

static void metrics_add(metrics_t *m, const nncar_sim_t *s, float omega, float v_cmd,
                        bool line_tick, float t_s, float dt)
{
    float lat = nncar_sim_lateral_error(s);

    if (t_s < SETTLE_S) {
        return;
    }

    m->tick_n += 1;
    if (fabsf(lat) > m->max_lat) {
        m->max_lat = fabsf(lat);
    }
    if (fabsf(lat) > 0.05f) {
        m->out_n += 1;
    }

    if (line_tick) {
        m->lat_sq += lat * lat;
        m->lat_n += 1;
        m->omega_sq += omega * omega;
        if (m->lat_n > 1) {
            float d = omega - m->prev_omega;
            m->domega_sq += d * d;
        }
        m->prev_omega = omega;
        m->speed_sum += v_cmd;
    }

    (void)dt;
}

static float metric_rms(const metrics_t *m)
{
    return (m->lat_n > 0) ? sqrtf(m->lat_sq / (float)m->lat_n) : 0.0f;
}

static float metric_domega(const metrics_t *m)
{
    return (m->lat_n > 1) ? sqrtf(m->domega_sq / (float)(m->lat_n - 1)) : 0.0f;
}

static float metric_speed(const metrics_t *m)
{
    return (m->lat_n > 0) ? m->speed_sum / (float)m->lat_n : 0.0f;
}

static float metric_out(const metrics_t *m)
{
    return (m->tick_n > 0) ? (float)m->out_n / (float)m->tick_n : 0.0f;
}

static void start_pose(const scenario_t *sc, float *x0, float *y0)
{
    /* Start on a crest: the line sits under the array with zero local slope. */
    *x0 = 0.25f * sc->wavelength_m;
    *y0 = sc->amp_m + sc->offset_m;
}

static void apply_scenario(nncar_sim_t *s, const scenario_t *sc, uint32_t seed)
{
    nncar_sim_seed(s, seed);

    s->track_amp_m = sc->amp_m;
    s->track_wavelength_m = sc->wavelength_m;
    s->track_extra_amp_m[0] = 0.0f;
    s->track_extra_amp_m[1] = 0.0f;
    s->realistic_dynamics = true;
    s->v_rail = sc->rail_v;

    s->gray_noise_std = sc->noise_std;
    s->gray_width_scale = sc->sensor_width_scale;
    if (sc->plant_deadband >= 0.0f) {
        s->plant_motor_deadband = sc->plant_deadband;
    }
    if (sc->inertia_scale > 0.0f) {
        s->inertia_kgm2 *= sc->inertia_scale;
    }
    if (sc->body_mass_scale > 0.0f) {
        s->body_mass_kg *= sc->body_mass_scale;
    }
    if (sc->yaw_inertia_scale > 0.0f) {
        s->yaw_inertia_kgm2 *= sc->yaw_inertia_scale;
    }
    if (sc->grip_scale > 0.0f) {
        s->tire_mu *= sc->grip_scale;
    }
    s->gray_offset = sc->offset_bias;
    s->gray_valid_prob = sc->valid_prob;
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        s->gray_gain[i] = 1.0f + ((i % 2 == 0) ? sc->gain_bias : -sc->gain_bias);
        s->gray_missing[i] = (uint8_t)((i == sc->missing_a || i == sc->missing_b) ? 1u : 0u);
    }

    if (sc->random_track) {
        uint32_t state = (sc->track_seed != 0u) ? sc->track_seed : seed;
        for (int i = 0; i < 2; ++i) {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            float u0 = (float)(state >> 8) * (1.0f / 16777216.0f);
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            float u1 = (float)(state >> 8) * (1.0f / 16777216.0f);
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            float u2 = (float)(state >> 8) * (1.0f / 16777216.0f);

            s->track_extra_amp_m[i] = 0.006f + 0.015f * u0;
            s->track_extra_wavelength_m[i] = 0.8f + 1.2f * u1;
            s->track_extra_phase_rad[i] = 2.0f * NNCAR_PI * u2;
        }
        float y = 0.0f;
        float slope = 0.0f;
        nncar_sim_track_geometry(s->x, s, &y, &slope, NULL);
        s->y = y + sc->offset_m;
        s->theta = atanf(slope) + sc->heading_deg * NNCAR_PI / 180.0f;
    }
}

/*
 * Host-side oracle: Stanley-style path tracking on the ground-truth track,
 * with a curvature feedforward and a curvature-limited speed policy. It sees
 * the true pose, so it is only a teacher / upper bound, never a shipped
 * controller. The same kinematic envelope as the shipped path is applied.
 */
/* Gains tuned against the opt-in coupled body/tire host plant. */
static float k_oracle_ke = 400.0f;    /* 1/m^2, lateral error gain */
static float k_oracle_ktheta = 120.0f; /* 1/m, heading error gain */
static float k_oracle_alat = 0.85f;   /* fraction of a_lat_max for the speed cap */
static float k_r4_alpha_ratio = 0.0f;  /* 0 uses the profile's R4 residual scale */
static bool k_trace = false;
static int k_trace_index = -1;

static void oracle_cmd(const nncar_profile_t *p, const nncar_sim_t *s,
                       float v_desired, float v_max_eff, nncar_twist_t *out,
                       float *kappa_out)
{
    float y_p = 0.0f;
    float yp = 0.0f;
    float ypp = 0.0f;
    nncar_sim_track_geometry(s->x, s, &y_p, &yp, &ypp);

    const float theta_path = atanf(yp);
    const float e_lat = s->y - y_p;   /* + = vehicle left of the line */

    float dtheta = theta_path - s->theta;
    while (dtheta > NNCAR_PI) {
        dtheta -= 2.0f * NNCAR_PI;
    }
    while (dtheta < -NNCAR_PI) {
        dtheta += 2.0f * NNCAR_PI;
    }

    const float kappa_path = ypp / powf(1.0f + yp * yp, 1.5f);
    const float kappa = kappa_path - k_oracle_ke * e_lat + k_oracle_ktheta * dtheta;

    /* Speed policy on the *commanded* curvature, so it also slows for large
     * corrective steering. A student can learn this from the line offset. */
    float v = fminf(v_desired, v_max_eff);
    const float kabs = fmaxf(fabsf(kappa), 1.0e-4f);
    v = fminf(v, sqrtf(k_oracle_alat * p->a_lat_max_mps2 / kabs));
    v = fmaxf(v, fminf(v_desired, 0.30f));

    nncar_twist_t twist;
    twist.v = v;
    twist.omega = kappa * v;

    *out = nncar_twist_envelope(p, twist, v_max_eff);
    *kappa_out = kappa;
}

static void encoder_update(const nncar_profile_t *p, nncar_velocity_t *vel_l,
                           nncar_velocity_t *vel_r, float *acc_l, float *acc_r,
                           float *v_l, float *v_r, const nncar_sim_t *s, float dt,
                           bool *edges)
{
    const float cpm = nncar_counts_per_meter(p);

    *acc_l += s->w_left * p->wheel_radius_m * dt * cpm;
    *acc_r += s->w_right * p->wheel_radius_m * dt * cpm;

    int32_t dl = (int32_t)floorf(*acc_l);
    int32_t dr = (int32_t)floorf(*acc_r);
    *acc_l -= (float)dl;
    *acc_r -= (float)dr;

    *v_l = nncar_velocity_step(vel_l, dl, dt);
    *v_r = nncar_velocity_step(vel_r, dr, dt);
    *edges = (dl != 0) || (dr != 0);
}

/* One binary row: 68 float32 inputs, s_v, kappa_norm, kappa_pid_norm. */
#define DUMP_ROW 71

static void dump_pair(FILE *dump, const nncar_profile_t *p,
                      const nncar_gray_frame_t *frame, const nncar_line_estimate_t *est,
                      float v_desired, float v_cmd, float kappa, float kappa_pid)
{
    float row[DUMP_ROW];

    nncar_line_nn_input(p, frame, est, v_desired, row);

    float s_v = (v_desired > 1.0e-3f) ? v_cmd / v_desired : 0.0f;
    row[NNCAR_LINE_INPUT_DIM] = nncar_clampf(s_v, 0.0f, 1.0f);
    row[NNCAR_LINE_INPUT_DIM + 1] = nncar_clampf(kappa / p->kappa_ref, -1.0f, 1.0f);
    row[NNCAR_LINE_INPUT_DIM + 2] = nncar_clampf(kappa_pid, -1.0f, 1.0f);

    if (fwrite(row, sizeof(row), 1, dump) != 1) {
        /* Write errors are surfaced by the caller through ferror(). */
    }
}

/* R4 row: 7 inputs, 2 control targets, 2 privileged labels, 1 PD baseline. */
#define DUMP_R4_ROW 12

static void dump_r4_pair(FILE *dump, const nncar_profile_t *p,
                         const nncar_gray_frame_t *frame,
                         const nncar_line_estimate_t *est,
                         const nncar_line_state_t *line_state,
                         nncar_line_r4_state_t *r4_state,
                         const nncar_sim_t *sim,
                         float v_current,
                         float v_desired,
                         float v_max_eff,
                         nncar_twist_t teacher,
                         float current_omega)
{
    float row[DUMP_R4_ROW];
    float x[NNCAR_LINE_R4_INPUT_DIM];
    nncar_line_r4_input(p, frame, r4_state, v_current, p->line_dt_s, x);
    memcpy(row, x, sizeof(x));

    float base_omega = 0.0f;
    if (est->confidence >= p->conf_nn_min && est->valid_count >= p->conf_valid_min) {
        base_omega = p->pid_kp * est->centroid_m + p->pid_kd * line_state->d_filtered;
    }
    float alpha_ratio = (k_r4_alpha_ratio > 0.0f) ? k_r4_alpha_ratio
                                                   : p->line_kappa_alpha;
    float alpha_omega = fmaxf(alpha_ratio * p->omega_ref_radps, 1.0e-3f);
    row[7] = nncar_clampf((teacher.omega - base_omega) / alpha_omega, -0.96f, 0.96f);
    row[11] = nncar_clampf(base_omega / fmaxf(p->omega_ref_radps, 1.0e-3f), -1.0f, 1.0f);

    float v_hi = fminf(fmaxf(v_desired, 0.0f), fmaxf(v_max_eff, 0.0f));
    float v_lo = fminf(fmaxf(p->v_min_kappa_mps, 0.0f), v_hi);
    float gamma = (v_hi > v_lo + 1.0e-6f)
                      ? nncar_clampf((teacher.v - v_lo) / (v_hi - v_lo), 0.0f, 1.0f)
                      : 0.5f;
    row[8] = 2.0f * gamma - 1.0f;

    row[9] = nncar_clampf(nncar_sim_lateral_error(sim) /
                              fmaxf(p->position_ref_m, 1.0e-3f),
                          -1.0f, 1.0f);
    float yp = 0.0f;
    float ypp = 0.0f;
    nncar_sim_track_geometry(sim->x, sim, NULL, &yp, &ypp);
    float kappa_true = ypp / powf(1.0f + yp * yp, 1.5f);
    row[10] = nncar_clampf(kappa_true / fmaxf(p->kappa_ref, 1.0e-3f), -1.0f, 1.0f);

    if (fwrite(row, sizeof(row), 1, dump) != 1) {
        /* Write errors are surfaced by the caller through ferror(). */
    }
    r4_state->prev_omega = current_omega;
}

/*
 * Shipped-controller run through the real stack (line -> mixer -> safety ->
 * motors). With dump != NULL the visited states are labelled by the oracle,
 * which is how off-policy DAgger-style data is produced.
 */
static void run_stack(const nncar_profile_t *p, const scenario_t *sc, src_t src,
                      const nncar_line_nn_t *nn,
                      const nncar_line_r4_nn_t *r4_nn,
                      uint32_t seed, metrics_t *m, FILE *dump, FILE *r4_dump)
{
    nncar_sim_t sim;
    float x0 = 0.0f;
    float y0 = 0.0f;
    start_pose(sc, &x0, &y0);
    nncar_sim_init(&sim, p, x0, y0, sc->heading_deg * NNCAR_PI / 180.0f, sc->rail_v);
    apply_scenario(&sim, sc, seed);

    nncar_stack_t stack;
    nncar_stack_init(&stack, p);
    nncar_line_ctrl_t ctrl = NNCAR_LINE_CTRL_PID;
    if (src == SRC_MLP) {
        ctrl = NNCAR_LINE_CTRL_MLP;
    } else if (src == SRC_RESIDUAL) {
        ctrl = NNCAR_LINE_CTRL_MLP_RESIDUAL;
    } else if (src == SRC_INT8) {
        ctrl = NNCAR_LINE_CTRL_MLP_INT8;
    }
    if (src == SRC_R4) {
        nncar_stack_set_line_r4_ctrl(&stack, r4_nn);
    } else {
        nncar_stack_set_line_ctrl(&stack, ctrl, (src == SRC_PID) ? NULL : nn);
    }

    nncar_velocity_t vel_l;
    nncar_velocity_t vel_r;
    nncar_velocity_init(&vel_l, p);
    nncar_velocity_init(&vel_r, p);

    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);
    const int ticks = (int)(DURATION_S / dt + 0.5f);

    float acc_l = 0.0f;
    float acc_r = 0.0f;
    float v_l = 0.0f;
    float v_r = 0.0f;
    uint64_t now_us = 0;
    bool edges = false;
    nncar_line_r4_state_t r4_state = {0.0f, 0.0f, 0.0f, false};

    for (int k = 0; k < ticks; ++k) {
        float gray_value[NNCAR_MAX_SENSORS];
        uint8_t gray_exist[NNCAR_MAX_SENSORS];
        uint8_t gray_valid[NNCAR_MAX_SENSORS];
        nncar_sim_read_gray(&sim, p, gray_value, gray_exist, gray_valid);

        nncar_stack_input_t in;
        memset(&in, 0, sizeof(in));
        in.v_desired = sc->v_desired;
        in.v_rail = sim.v_rail;
        in.v_rail_valid = true;
        in.v_rail_exist = true;
        in.now_us = now_us;
        in.actual_left_mps = v_l;
        in.actual_right_mps = v_r;
        in.actual_valid = true;
        in.encoder_activity = edges;
        in.comm_ok = true;

        nncar_stack_output_t out = nncar_stack_step(&stack, p, &in, gray_value,
                                                    gray_exist, gray_valid, dt);
        nncar_sim_step(&sim, p, out.pwm, dt);
        encoder_update(p, &vel_l, &vel_r, &acc_l, &acc_r, &v_l, &v_r, &sim, dt, &edges);

        const bool line_tick = (stack.line_accum_s < 1.0e-6f);
        const float t_s = (float)now_us * 1.0e-6f;

        metrics_add(m, &sim, out.twist.omega, out.twist.v, line_tick, t_s, dt);
        m->faults |= out.safety.active_faults;
        m->recovery_s += (out.mode == NNCAR_LINE_RECOVERY) ? dt : 0.0f;
        m->recovery_hits = (int)stack.line.transitions;

        if (k_trace && k_trace_index >= 0 && sc == &k_suite[k_trace_index] && line_tick &&
            k < 900) {
            fprintf(stderr,
                    "SRC=%d t=%.2f lat=%.4f v=%.3f w=%.3f mode=%d conf=%.2f faults=0x%x\n",
                    (int)src, (double)((float)now_us * 1.0e-6f),
                    (double)(sim.y - nncar_sim_track_y(sim.x, &sim)), (double)out.twist.v,
                    (double)out.twist.omega, (int)out.mode, (double)out.line.confidence,
                    out.safety.active_faults);
        }

        if (dump != NULL && line_tick) {
            const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
            nncar_twist_t label = {0.0f, 0.0f};
            float kappa = 0.0f;
            const float v_max_eff = nncar_v_max_eff(p, sim.v_rail);
            oracle_cmd(p, &sim, sc->v_desired, v_max_eff, &label, &kappa);
            /* The residual baseline is the PID evaluated at this same state. */
            const float kappa_pid = nncar_line_pid_kappa(p, &stack.line, &out.line,
                                                         sc->v_desired, v_max_eff);
            dump_pair(dump, p, &frame, &out.line, sc->v_desired, label.v, kappa,
                      kappa_pid);
        }

        if (r4_dump != NULL && line_tick) {
            const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
            nncar_twist_t label = {0.0f, 0.0f};
            float kappa = 0.0f;
            float v_max_eff = nncar_v_max_eff(p, sim.v_rail);
            oracle_cmd(p, &sim, sc->v_desired, v_max_eff, &label, &kappa);
            (void)kappa;
            dump_r4_pair(r4_dump, p, &frame, &out.line, &stack.line, &r4_state,
                         &sim, 0.5f * (v_l + v_r), sc->v_desired, v_max_eff,
                         label, out.twist.omega);
        }

        now_us += dt_us;
    }
}

/* Oracle run: the same execution stage, with a ground-truth twist source. */
static void run_oracle(const nncar_profile_t *p, const scenario_t *sc, uint32_t seed,
                       metrics_t *m, FILE *dump, FILE *r4_dump)
{
    nncar_sim_t sim;
    float x0 = 0.0f;
    float y0 = 0.0f;
    start_pose(sc, &x0, &y0);
    nncar_sim_init(&sim, p, x0, y0, sc->heading_deg * NNCAR_PI / 180.0f, sc->rail_v);
    apply_scenario(&sim, sc, seed);

    nncar_motor_state_t motor_l;
    nncar_motor_state_t motor_r;
    nncar_motor_reset(&motor_l);
    nncar_motor_reset(&motor_r);

    nncar_motor_cfg_t cfg;
    cfg.mode = NNCAR_MOTOR_MODE_PI_FF;
    cfg.nn = NULL;

    nncar_safety_state_t safety;
    nncar_safety_reset(&safety);

    nncar_line_estimate_t est;
    memset(&est, 0, sizeof(est));

    /* Shadow state so the dumped rows carry the same PID baseline a residual
     * student would see; it only tracks the shared derivative filter. */
    nncar_line_state_t shadow;
    nncar_line_reset(&shadow);

    nncar_velocity_t vel_l;
    nncar_velocity_t vel_r;
    nncar_velocity_init(&vel_l, p);
    nncar_velocity_init(&vel_r, p);

    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);
    const int ticks = (int)(DURATION_S / dt + 0.5f);
    const int line_every = (int)(p->line_dt_s / dt + 0.5f);

    nncar_twist_t twist = {0.0f, 0.0f};
    nncar_wheel_speed_t wheels = {0.0f, 0.0f};

    float acc_l = 0.0f;
    float acc_r = 0.0f;
    float v_l = 0.0f;
    float v_r = 0.0f;
    uint64_t now_us = 0;
    bool edges = false;
    float twist_age = 0.0f;
    float last_kappa = 0.0f;
    nncar_line_r4_state_t r4_state = {0.0f, 0.0f, 0.0f, false};

    for (int k = 0; k < ticks; ++k) {
        float gray_value[NNCAR_MAX_SENSORS];
        uint8_t gray_exist[NNCAR_MAX_SENSORS];
        uint8_t gray_valid[NNCAR_MAX_SENSORS];
        nncar_sim_read_gray(&sim, p, gray_value, gray_exist, gray_valid);

        const float v_max_eff = nncar_v_max_eff(p, sim.v_rail);
        twist_age += dt;
        const bool line_tick = (k % line_every) == 0;

        if (line_tick) {
            nncar_gray_process(p, gray_value, gray_exist, gray_valid, &est);

            float kappa = 0.0f;
            oracle_cmd(p, &sim, sc->v_desired, v_max_eff, &twist, &kappa);
            last_kappa = kappa;
            wheels = nncar_mixer(p, twist, v_max_eff).wheels;
            twist_age = 0.0f;

            if (dump != NULL || r4_dump != NULL) {
                const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
                nncar_line_history_update(p, &shadow, &est, p->line_dt_s);
                if (dump != NULL) {
                    const float kappa_pid = nncar_line_pid_kappa(p, &shadow, &est,
                                                                 sc->v_desired, v_max_eff);
                    dump_pair(dump, p, &frame, &est, sc->v_desired, twist.v, kappa,
                              kappa_pid);
                }
                if (r4_dump != NULL) {
                    dump_r4_pair(r4_dump, p, &frame, &est, &shadow, &r4_state,
                                 &sim, 0.5f * (v_l + v_r), sc->v_desired,
                                 v_max_eff, twist, twist.omega);
                }
            }
        }

        nncar_safety_input_t sin;
        memset(&sin, 0, sizeof(sin));
        sin.v_max_eff = v_max_eff;
        sin.wheel_target = wheels;
        sin.twist_age_s = twist_age;
        sin.wheel_target_age_s = twist_age;
        sin.actual_left.value = v_l;
        sin.actual_left.exist = true;
        sin.actual_left.valid = true;
        sin.actual_left.timestamp_us = now_us;
        sin.actual_right.value = v_r;
        sin.actual_right.exist = true;
        sin.actual_right.valid = true;
        sin.actual_right.timestamp_us = now_us;
        sin.v_rail.value = sim.v_rail;
        sin.v_rail.exist = true;
        sin.v_rail.valid = true;
        sin.v_rail.timestamp_us = now_us;
        sin.now_us = now_us;
        sin.encoder_activity = edges;
        sin.comm_ok = true;
        sin.applied_pwm_abs = fmaxf(fabsf(motor_l.u), fabsf(motor_r.u));

        nncar_safety_output_t saf = nncar_safety_step(p, &safety, &sin, dt);

        nncar_motor_input_t min;
        memset(&min, 0, sizeof(min));
        min.v_max_eff = v_max_eff;
        min.rail_ratio = sim.v_rail / p->rail_nominal_v;
        min.rail_exist = true;
        min.rail_valid = true;
        min.enable = !saf.force_zero_pwm;

        min.target_mps = saf.wheel_out.left;
        min.actual_mps = v_l;
        nncar_motor_out_t ml = nncar_motor_step(p, &motor_l, &cfg, &min, dt);

        min.target_mps = saf.wheel_out.right;
        min.actual_mps = v_r;
        nncar_motor_out_t mr = nncar_motor_step(p, &motor_r, &cfg, &min, dt);

        nncar_pwm_t pwm;
        pwm.left = ml.pwm;
        pwm.right = mr.pwm;
        nncar_sim_step(&sim, p, pwm, dt);
        encoder_update(p, &vel_l, &vel_r, &acc_l, &acc_r, &v_l, &v_r, &sim, dt, &edges);

        metrics_add(m, &sim, twist.omega, twist.v, line_tick,
                    (float)now_us * 1.0e-6f, dt);
        m->faults |= saf.active_faults;

        if (k_trace && line_tick && k < 900) {
            fprintf(stderr,
                    "t=%.3f lat=%.4f kap=%.3f w=%.3f wL=%.3f wR=%.3f tgt=(%.3f,%.3f) "
                    "pwm=(%.2f,%.2f) act=0x%x latch=0x%x ramp=%.2f\n",
                    (double)((float)now_us * 1.0e-6f),
                    (double)(sim.y - nncar_sim_track_y(sim.x, &sim)), (double)last_kappa,
                    (double)twist.omega,
                    (double)(sim.w_left * p->wheel_radius_m),
                    (double)(sim.w_right * p->wheel_radius_m),
                    (double)saf.wheel_out.left, (double)saf.wheel_out.right,
                    (double)ml.pwm, (double)mr.pwm, saf.active_faults, saf.latched_faults,
                    (double)saf.ramp);
        }

        now_us += dt_us;
    }
}

static void run_one(const nncar_profile_t *p, const scenario_t *sc, src_t src,
                    const nncar_line_nn_t *nn,
                    const nncar_line_r4_nn_t *r4_nn,
                    uint32_t seed, metrics_t *m, FILE *dump, FILE *r4_dump)
{
    metrics_init(m);
    if (src == SRC_ORACLE) {
        run_oracle(p, sc, seed, m, dump, r4_dump);
    } else {
        run_stack(p, sc, src, nn, r4_nn, seed, m, dump, r4_dump);
    }
}

static float aggregate(const metrics_t *m)
{
    float sum_sq = 0.0f;
    for (int i = 0; i < SUITE_COUNT; ++i) {
        float rms = metric_rms(&m[i]);
        sum_sq += rms * rms;
    }
    return sqrtf(sum_sq / (float)SUITE_COUNT);
}

static float eval_random_unit(uint32_t *state)
{
    uint32_t x = (*state != 0u) ? *state : 0x9e3779b9u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

static void make_random_eval_case(const nncar_profile_t *base, uint32_t seed,
                                  int episode, nncar_profile_t *profile,
                                  scenario_t *sc)
{
    uint32_t rng = seed ^ ((uint32_t)(episode + 1) * 0x85ebca6bu);
    float u[18];
    for (int i = 0; i < 18; ++i) {
        u[i] = eval_random_unit(&rng);
    }

    *profile = *base;
    static const int counts[4] = {5, 6, 7, 8};
    profile->sensor_count = counts[(episode + (int)(u[0] * 4.0f)) % 4];
    float span = fabsf(base->sensor_position_m[0]) * (0.6f + 0.9f * u[1]);
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        profile->sensor_position_m[i] = 0.0f;
    }
    for (int i = 0; i < profile->sensor_count; ++i) {
        float q = (float)i / (float)(profile->sensor_count - 1);
        profile->sensor_position_m[i] = span * (1.0f - 2.0f * q);
    }

    memset(sc, 0, sizeof(*sc));
    sc->name = "heldout_random";
    sc->amp_m = 0.02f + 0.10f * u[2];
    sc->wavelength_m = 0.9f + 1.8f * u[3];
    sc->offset_m = -0.025f + 0.05f * u[4];
    sc->heading_deg = -8.0f + 16.0f * u[5];
    sc->v_desired = 0.5f + 0.75f * u[6];
    sc->rail_v = 10.5f + 1.8f * u[7];
    sc->noise_std = 0.08f;
    sc->gain_bias = 0.08f;
    sc->offset_bias = -0.08f + 0.16f * u[8];
    sc->valid_prob = 0.94f + 0.06f * u[9];
    sc->sensor_width_scale = 0.6f + 0.9f * u[10];
    sc->inertia_scale = 0.5f + 1.7f * u[11];
    sc->plant_deadband = 0.05f + 0.23f * u[12];
    sc->body_mass_scale = 0.5f + 1.7f * u[13];
    sc->yaw_inertia_scale = 0.5f + 1.7f * u[14];
    sc->grip_scale = 0.65f + 0.5f * u[15];
    sc->missing_a = (u[16] > 0.75f) ? (int)(u[17] * profile->sensor_count) : -1;
    sc->missing_b = -1;
    sc->random_track = true;
    sc->track_seed = seed ^ ((uint32_t)(episode + 1) * 0x9e3779b9u);
}

static void metrics_merge(metrics_t *dst, const metrics_t *src)
{
    dst->lat_sq += src->lat_sq;
    dst->lat_n += src->lat_n;
    dst->max_lat = fmaxf(dst->max_lat, src->max_lat);
    dst->omega_sq += src->omega_sq;
    dst->domega_sq += src->domega_sq;
    dst->speed_sum += src->speed_sum;
    dst->recovery_s += src->recovery_s;
    dst->out_n += src->out_n;
    dst->tick_n += src->tick_n;
    dst->faults |= src->faults;
}

static void random_r4_check(const nncar_profile_t *p,
                            const nncar_line_nn_t *r3_nn,
                            const nncar_line_r4_nn_t *r4_nn,
                            int episodes, uint32_t seed)
{
    metrics_t pid_total = {0};
    metrics_t r3_total = {0};
    metrics_t r4_total = {0};
    for (int ep = 0; ep < episodes; ++ep) {
        nncar_profile_t instance;
        scenario_t sc;
        make_random_eval_case(p, seed, ep, &instance, &sc);
        metrics_t one;
        const uint32_t episode_seed = seed + (uint32_t)ep * 7919u + 17u;
        run_one(&instance, &sc, SRC_PID, NULL, NULL, episode_seed, &one, NULL, NULL);
        metrics_merge(&pid_total, &one);
        run_one(&instance, &sc, SRC_MLP, r3_nn, NULL, episode_seed, &one, NULL, NULL);
        metrics_merge(&r3_total, &one);
        run_one(&instance, &sc, SRC_R4, NULL, r4_nn, episode_seed, &one, NULL, NULL);
        metrics_merge(&r4_total, &one);
    }

    printf("\nHeld-out randomized episodes: %d (same per-episode plant/noise for all controllers)\n",
           episodes);
    printf("  PID RMS %.5f m; R3 MLP %.5f m; R4 %.5f m; R4 vs PID %+.1f%%\n",
           (double)metric_rms(&pid_total), (double)metric_rms(&r3_total),
           (double)metric_rms(&r4_total),
           (double)(100.0f * (1.0f - metric_rms(&r4_total) /
                                       fmaxf(metric_rms(&pid_total), 1.0e-6f))));
    printf("  max |e|: PID %.4f m; R3 %.4f m; R4 %.4f m; fault unions 0x%x/0x%x/0x%x%s\n",
           (double)pid_total.max_lat, (double)r3_total.max_lat, (double)r4_total.max_lat,
           pid_total.faults, r3_total.faults, r4_total.faults,
           (r4_nn != NULL) ? "" : " [R4 weights unavailable; PID fallback]");
}

/* Randomised episodes for teacher data: on-policy oracle plus off-policy PID. */
static void dump_teacher(const nncar_profile_t *p, const char *path,
                         const char *r4_path, int episodes, uint32_t seed)
{
    FILE *fp = (path != NULL) ? fopen(path, "wb") : NULL;
    FILE *r4_fp = (r4_path != NULL) ? fopen(r4_path, "wb") : NULL;
    if ((path != NULL && fp == NULL) || (r4_path != NULL && r4_fp == NULL)) {
        fprintf(stderr, "cannot open teacher output\n");
        if (fp != NULL) {
            fclose(fp);
        }
        if (r4_fp != NULL) {
            fclose(r4_fp);
        }
        return;
    }

    /* Header: magic, episodes, line ticks per run, runs per episode. */
    const int32_t rows_per_run = (int32_t)(DURATION_S / p->line_dt_s + 0.5f);
    const int32_t runs_per_ep = 3;   /* oracle, PID, current MLP */
    const int32_t header[4] = {0x4C4E4E31, episodes, rows_per_run, runs_per_ep};
    const int32_t r4_header[4] = {0x4C4E5231, episodes, rows_per_run, runs_per_ep};
    if ((fp != NULL && fwrite(header, sizeof(header), 1, fp) != 1) ||
        (r4_fp != NULL && fwrite(r4_header, sizeof(r4_header), 1, r4_fp) != 1)) {
        fprintf(stderr, "write error on teacher header\n");
        if (fp != NULL) {
            fclose(fp);
        }
        if (r4_fp != NULL) {
            fclose(r4_fp);
        }
        return;
    }

    uint32_t rng = seed | 1u;
    for (int ep = 0; ep < episodes; ++ep) {
        float u[14];
        for (int j = 0; j < 14; ++j) {
            rng = rng * 1664525u + 1013904223u;
            u[j] = (float)(rng >> 8) * (1.0f / 16777216.0f);
        }

        nncar_profile_t randomized = *p;
        scenario_t sc;
        memset(&sc, 0, sizeof(sc));
        sc.name = "random";
        sc.amp_m = 0.02f + 0.10f * u[0];
        sc.wavelength_m = 1.10f + 1.30f * u[1];
        sc.offset_m = -0.025f + 0.05f * u[2];
        sc.heading_deg = -6.0f + 12.0f * u[3];
        sc.v_desired = 0.55f + 0.65f * ((float)((ep * 7) % 11) / 10.0f);
        sc.rail_v = (r4_fp != NULL) ? (10.5f + 1.8f * u[13])
                                    : (11.2f + 1.1f * ((float)((ep * 3) % 9) / 8.0f));
        sc.noise_std = (r4_fp != NULL) ? 0.08f : ((ep % 4 == 3) ? 0.06f : 0.0f);
        sc.gain_bias = (r4_fp != NULL) ? 0.08f : ((ep % 5 == 4) ? 0.10f : 0.0f);
        sc.offset_bias = (r4_fp != NULL) ? (-0.08f + 0.16f * u[5])
                                          : ((ep % 6 == 5) ? 0.04f : 0.0f);
        sc.sensor_width_scale = (r4_fp != NULL) ? (0.6f + 0.9f * u[6]) : 1.0f;
        sc.inertia_scale = (r4_fp != NULL) ? (0.5f + 1.7f * u[7]) : 1.0f;
        sc.plant_deadband = (r4_fp != NULL) ? (0.05f + 0.23f * u[8]) : -1.0f;
        sc.body_mass_scale = (r4_fp != NULL) ? (0.5f + 1.7f * u[10]) : 1.0f;
        sc.yaw_inertia_scale = (r4_fp != NULL) ? (0.5f + 1.7f * u[11]) : 1.0f;
        sc.grip_scale = (r4_fp != NULL) ? (0.65f + 0.5f * u[12]) : 1.0f;
        sc.random_track = (r4_fp != NULL);
        sc.track_seed = seed ^ ((uint32_t)ep * 0x9e3779b9u) ^ 0x72a59b4du;
        if (r4_fp != NULL) {
            static const int sensor_counts[4] = {5, 6, 7, 8};
            randomized.sensor_count = sensor_counts[ep % 4];
            float span = fabsf(p->sensor_position_m[0]) * (0.6f + 0.9f * u[4]);
            for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
                randomized.sensor_position_m[i] = 0.0f;
            }
            for (int i = 0; i < randomized.sensor_count; ++i) {
                float fraction = (float)i / (float)(randomized.sensor_count - 1);
                randomized.sensor_position_m[i] = span * (1.0f - 2.0f * fraction);
            }
        }
        /* Missing channels are drawn from the active array, not fixed slots. */
        sc.missing_a = (ep % 7 == 6) ? (ep * 3 + 1) % randomized.sensor_count : -1;
        sc.missing_b = (ep % 7 == 6) ? (ep * 5 + 4) % randomized.sensor_count : -1;
        sc.valid_prob = (r4_fp != NULL) ? (0.92f + 0.08f * u[9])
                                        : ((ep % 7 == 6) ? 0.92f : 1.0f);

        metrics_t m;
        const nncar_profile_t *episode_profile = (r4_fp != NULL) ? &randomized : p;
        const uint32_t episode_seed = seed + (uint32_t)ep * 7919u + 1u;
        run_one(episode_profile, &sc, SRC_ORACLE, NULL, NULL,
                episode_seed, &m, fp, r4_fp);
        run_one(episode_profile, &sc, SRC_PID, NULL, NULL,
                episode_seed, &m, fp, r4_fp);
        /* Student-in-the-loop states, relabelled by the oracle (DAgger). */
        if (r4_fp != NULL) {
            run_one(episode_profile, &sc, SRC_R4, NULL, nncar_line_r4_nn_trained(),
                    episode_seed, &m, fp, r4_fp);
        } else {
            run_one(episode_profile, &sc, SRC_MLP, nncar_line_nn_trained(), NULL,
                    episode_seed, &m, fp, r4_fp);
        }
    }

    if ((fp != NULL && ferror(fp)) || (r4_fp != NULL && ferror(r4_fp))) {
        fprintf(stderr, "write error on teacher output\n");
    }
    if (fp != NULL) {
        fclose(fp);
        fprintf(stderr, "teacher: %d episodes -> %s\n", episodes, path);
    }
    if (r4_fp != NULL) {
        fclose(r4_fp);
        fprintf(stderr, "R4 teacher: %d episodes, %d rows x %d float32 -> %s\n",
                episodes, episodes * runs_per_ep * rows_per_run, DUMP_R4_ROW, r4_path);
    }
}

static void print_header(void)
{
    printf("\n%-12s %8s %8s %8s %8s %8s %8s %8s %8s\n", "scenario", "pid_rms",
           "mlp_rms", "res_rms", "i8_rms", "orc_rms", "pid_max", "mlp_max",
           "pid_dw");
}

int main(int argc, char **argv)
{
    const char *dump_path = NULL;
    const char *r4_dump_path = NULL;
    int episodes = 120;
    int random_check_episodes = 0;
    uint32_t seed = 1u;
    bool check = false;
    bool r4_check = false;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--check") == 0) {
            check = true;
        } else if (strcmp(a, "--r4-check") == 0) {
            r4_check = true;
        } else if (strcmp(a, "--r4-random-check") == 0 && i + 1 < argc) {
            random_check_episodes = atoi(argv[++i]);
        } else if (strcmp(a, "--dump-teacher") == 0 && i + 1 < argc) {
            dump_path = argv[++i];
        } else if (strcmp(a, "--dump-r4-teacher") == 0 && i + 1 < argc) {
            r4_dump_path = argv[++i];
        } else if (strcmp(a, "--episodes") == 0 && i + 1 < argc) {
            episodes = atoi(argv[++i]);
        } else if (strcmp(a, "--seed") == 0 && i + 1 < argc) {
            seed = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(a, "--ke") == 0 && i + 1 < argc) {
            k_oracle_ke = (float)atof(argv[++i]);
        } else if (strcmp(a, "--ktheta") == 0 && i + 1 < argc) {
            k_oracle_ktheta = (float)atof(argv[++i]);
        } else if (strcmp(a, "--alat") == 0 && i + 1 < argc) {
            k_oracle_alat = (float)atof(argv[++i]);
        } else if (strcmp(a, "--r4-alpha-ratio") == 0 && i + 1 < argc) {
            k_r4_alpha_ratio = (float)atof(argv[++i]);
        } else if (strcmp(a, "--trace") == 0) {
            k_trace = true;
            if (i + 1 < argc) {
                k_trace_index = atoi(argv[++i]);
            }
        } else {
            fprintf(stderr,
                    "usage: %s [--check] [--dump-teacher out.csv] "
                    "[--dump-r4-teacher out.bin] [--r4-check] "
                    "[--r4-random-check N] "
                    "[--episodes N] [--seed S]\n"
                    "          [--ke K] [--ktheta K] [--alat F]  (oracle tuning)\n"
                    "          [--r4-alpha-ratio R]  (R4 label/output scale)\n"
                    "          [--trace N]  (print scenario N per line tick)\n",
                    argv[0]);
            return 2;
        }
    }

    const nncar_profile_t *p = nncar_profile_default();

    if (dump_path != NULL || r4_dump_path != NULL) {
        dump_teacher(p, dump_path, r4_dump_path, episodes, seed);
        return 0;
    }

    const nncar_line_nn_t *trained = nncar_line_nn_trained();
    const nncar_line_nn_t *residual_nn = nncar_line_nn_residual_trained();
    const nncar_line_r4_nn_t *r4_nn = nncar_line_r4_nn_trained();

    metrics_t pid[SUITE_COUNT];
    metrics_t mlp[SUITE_COUNT];
    metrics_t res[SUITE_COUNT];
    metrics_t i8[SUITE_COUNT];
    metrics_t orc[SUITE_COUNT];
    metrics_t r4[SUITE_COUNT];

    printf("line A/B (profile %s, line_dt %.3f s, %d scenarios)\n", nncar_profile_name(),
           (double)p->line_dt_s, SUITE_COUNT);
    print_header();

    for (int i = 0; i < SUITE_COUNT; ++i) {
        const uint32_t s = (uint32_t)(i + 1);
        run_one(p, &k_suite[i], SRC_PID, NULL, NULL, s, &pid[i], NULL, NULL);
        run_one(p, &k_suite[i], SRC_MLP, trained, NULL, s, &mlp[i], NULL, NULL);
        run_one(p, &k_suite[i], SRC_RESIDUAL, residual_nn, NULL, s, &res[i], NULL, NULL);
        run_one(p, &k_suite[i], SRC_INT8, NULL, NULL, s, &i8[i], NULL, NULL);
        run_one(p, &k_suite[i], SRC_ORACLE, NULL, NULL, s, &orc[i], NULL, NULL);
        if (r4_check) {
            run_one(p, &k_suite[i], SRC_R4, NULL, r4_nn, s, &r4[i], NULL, NULL);
        }

        printf("%-12s %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
               k_suite[i].name, (double)metric_rms(&pid[i]),
               (double)metric_rms(&mlp[i]), (double)metric_rms(&res[i]),
               (double)metric_rms(&i8[i]), (double)metric_rms(&orc[i]),
               (double)pid[i].max_lat, (double)mlp[i].max_lat,
               (double)metric_domega(&pid[i]));
    }

    const float pid_agg = aggregate(pid);
    const float mlp_agg = aggregate(mlp);
    const float res_agg = aggregate(res);
    const float i8_agg = aggregate(i8);
    const float orc_agg = aggregate(orc);
    const float r4_agg = r4_check ? aggregate(r4) : 0.0f;

    printf("%-12s %8.4f %8.4f %8.4f %8.4f %8.4f\n", "aggregate", (double)pid_agg,
           (double)mlp_agg, (double)res_agg, (double)i8_agg, (double)orc_agg);
    if (r4_check) {
        printf("R4 experimental aggregate RMS: %.5f (%+.1f%% vs PID)%s\n",
               (double)r4_agg,
               (double)(100.0f * (1.0f - r4_agg / fmaxf(pid_agg, 1.0e-6f))),
               (r4_nn != NULL) ? "" : " [weights unavailable; PID fallback]");
        printf("%-12s %8s %8s %8s %8s\n", "scenario", "r4_rms", "r4_max",
               "r4_speed", "faults");
        for (int i = 0; i < SUITE_COUNT; ++i) {
            printf("%-12s %8.4f %8.4f %8.3f 0x%x\n", k_suite[i].name,
                   (double)metric_rms(&r4[i]), (double)r4[i].max_lat,
                   (double)metric_speed(&r4[i]), r4[i].faults);
        }
    }

    printf("\n%-12s %8s %8s %8s %8s %8s\n", "scenario", "out%_pid", "out%_mlp",
           "spd_pid", "spd_mlp", "recov_s");
    for (int i = 0; i < SUITE_COUNT; ++i) {
        printf("%-12s %8.1f %8.1f %8.3f %8.3f %8.3f\n", k_suite[i].name,
               (double)(metric_out(&pid[i]) * 100.0f),
               (double)(metric_out(&mlp[i]) * 100.0f), (double)metric_speed(&pid[i]),
               (double)metric_speed(&mlp[i]), (double)pid[i].recovery_s);
    }

    if (random_check_episodes > 0) {
        random_r4_check(p, trained, r4_nn, random_check_episodes, seed);
    }

    if (!check) {
        return 0;
    }

    int failures = 0;

    /* Pre-declared gate: the MLP must cut aggregate lateral RMS by >= 10% and
     * must not be worse than PID by more than 10% on any single scenario. */
    printf("\nA/B: PID %.5f -> MLP %.5f (%+.1f%%), oracle %.5f\n", (double)pid_agg,
           (double)mlp_agg, (double)(100.0f * (1.0f - mlp_agg / pid_agg)),
           (double)orc_agg);
    printf("     int8     %.5f (%+.1f%% vs MLP fp32)\n", (double)i8_agg,
           (double)(100.0f * (1.0f - i8_agg / mlp_agg)));
    printf("     residual %.5f (%+.1f%% vs PID, %+.1f%% vs MLP)\n", (double)res_agg,
           (double)(100.0f * (1.0f - res_agg / pid_agg)),
           (double)(100.0f * (1.0f - res_agg / mlp_agg)));

    if (!(mlp_agg <= 0.90f * pid_agg)) {
        printf("FAIL: aggregate MLP rms is not 10%% below PID\n");
        failures += 1;
    }

    /* The residual variant must at least not be worse than the PID it is
     * anchored to; anything else means the export or the label is wrong. */
    if (res_agg > pid_agg) {
        printf("FAIL: residual %.5f is worse than PID %.5f\n", (double)res_agg,
               (double)pid_agg);
        failures += 1;
    }

    for (int i = 0; i < SUITE_COUNT; ++i) {
        float a = metric_rms(&pid[i]);
        float b = metric_rms(&mlp[i]);
        if (b > 1.10f * a) {
            printf("FAIL: %s MLP %.4f is >10%% worse than PID %.4f\n", k_suite[i].name,
                   (double)b, (double)a);
            failures += 1;
        }
    }

    if (failures != 0) {
        fprintf(stderr, "line A/B: FAILED (%d)\n", failures);
        return 1;
    }

    printf("line A/B: ok\n");
    return 0;
}
