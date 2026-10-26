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
    SRC_ORACLE = 2
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
    {"straight", 0.00f, 1.60f, 0.020f, 3.0f, 0.90f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
    {"gentle_s", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
    {"sharp_s", 0.12f, 0.90f, 0.000f, 0.0f, 0.60f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
    {"fast_gentle", 0.06f, 1.60f, 0.010f, 1.5f, 1.20f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
    {"noise", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.08f, 0.12f, 0.05f, -1, -1, 1.00f},
    {"dropout", 0.08f, 1.60f, 0.010f, 0.0f, 0.90f, 12.0f, 0.02f, 0.00f, 0.00f, 2, 5, 0.90f},
    {"rail_sag", 0.10f, 1.20f, 0.000f, 0.0f, 0.75f, 11.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
    {"disturbance", 0.00f, 1.60f, 0.030f, 5.0f, 1.20f, 12.0f, 0.00f, 0.00f, 0.00f, -1, -1, 1.00f},
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
    float lat = s->y - nncar_sim_track_y(s->x, s);

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
    s->v_rail = sc->rail_v;

    s->gray_noise_std = sc->noise_std;
    s->gray_offset = sc->offset_bias;
    s->gray_valid_prob = sc->valid_prob;
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        s->gray_gain[i] = 1.0f + ((i % 2 == 0) ? sc->gain_bias : -sc->gain_bias);
        s->gray_missing[i] = (uint8_t)((i == sc->missing_a || i == sc->missing_b) ? 1u : 0u);
    }
}

/*
 * Host-side oracle: Stanley-style path tracking on the ground-truth track,
 * with a curvature feedforward and a curvature-limited speed policy. It sees
 * the true pose, so it is only a teacher / upper bound, never a shipped
 * controller. The same kinematic envelope as the shipped path is applied.
 */
static float k_oracle_ke = 80.0f;     /* 1/m^2, lateral error gain */
static float k_oracle_ktheta = 10.0f; /* 1/m, heading error gain */
static float k_oracle_alat = 0.85f;   /* fraction of a_lat_max for the speed cap */
static bool k_trace = false;
static int k_trace_index = -1;

static void oracle_cmd(const nncar_profile_t *p, const nncar_sim_t *s,
                       float v_desired, float v_max_eff, nncar_twist_t *out,
                       float *kappa_out)
{
    const float amp = s->track_amp_m;
    const float w = 2.0f * NNCAR_PI / s->track_wavelength_m;

    const float y_p = amp * sinf(w * s->x);
    const float yp = amp * w * cosf(w * s->x);
    const float ypp = -amp * w * w * sinf(w * s->x);

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

/* One binary row: 68 float32 inputs followed by s_v and kappa_norm. */
#define DUMP_ROW 70

static void dump_pair(FILE *dump, const nncar_profile_t *p,
                      const nncar_gray_frame_t *frame, const nncar_line_estimate_t *est,
                      float v_desired, float v_cmd, float kappa)
{
    float row[DUMP_ROW];

    nncar_line_nn_input(p, frame, est, v_desired, row);

    float s_v = (v_desired > 1.0e-3f) ? v_cmd / v_desired : 0.0f;
    row[NNCAR_LINE_INPUT_DIM] = nncar_clampf(s_v, 0.0f, 1.0f);
    row[NNCAR_LINE_INPUT_DIM + 1] = nncar_clampf(kappa / p->kappa_ref, -1.0f, 1.0f);

    if (fwrite(row, sizeof(row), 1, dump) != 1) {
        /* Write errors are surfaced by the caller through ferror(). */
    }
}

/*
 * Shipped-controller run through the real stack (line -> mixer -> safety ->
 * motors). With dump != NULL the visited states are labelled by the oracle,
 * which is how off-policy DAgger-style data is produced.
 */
static void run_stack(const nncar_profile_t *p, const scenario_t *sc, src_t src,
                      const nncar_line_nn_t *nn, uint32_t seed, metrics_t *m,
                      FILE *dump)
{
    nncar_sim_t sim;
    float x0 = 0.0f;
    float y0 = 0.0f;
    start_pose(sc, &x0, &y0);
    nncar_sim_init(&sim, p, x0, y0, sc->heading_deg * NNCAR_PI / 180.0f, sc->rail_v);
    apply_scenario(&sim, sc, seed);

    nncar_stack_t stack;
    nncar_stack_init(&stack, p);
    nncar_stack_set_line_ctrl(&stack, (src == SRC_MLP) ? NNCAR_LINE_CTRL_MLP
                                                       : NNCAR_LINE_CTRL_PID,
                              (src == SRC_MLP) ? nn : NULL);

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
            oracle_cmd(p, &sim, sc->v_desired, nncar_v_max_eff(p, sim.v_rail), &label,
                       &kappa);
            dump_pair(dump, p, &frame, &out.line, sc->v_desired, label.v, kappa);
        }

        now_us += dt_us;
    }
}

/* Oracle run: the same execution stage, with a ground-truth twist source. */
static void run_oracle(const nncar_profile_t *p, const scenario_t *sc, uint32_t seed,
                       metrics_t *m, FILE *dump)
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

            if (dump != NULL) {
                const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
                dump_pair(dump, p, &frame, &est, sc->v_desired, twist.v, kappa);
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
                    const nncar_line_nn_t *nn, uint32_t seed, metrics_t *m, FILE *dump)
{
    metrics_init(m);
    if (src == SRC_ORACLE) {
        run_oracle(p, sc, seed, m, dump);
    } else {
        run_stack(p, sc, src, nn, seed, m, dump);
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

/* Randomised episodes for teacher data: on-policy oracle plus off-policy PID. */
static void dump_teacher(const nncar_profile_t *p, const char *path, int episodes,
                         uint32_t seed)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        fprintf(stderr, "cannot open %s\n", path);
        return;
    }

    /* Header: magic, episodes, line ticks per run, runs per episode. */
    const int32_t rows_per_run = (int32_t)(DURATION_S / p->line_dt_s + 0.5f);
    const int32_t runs_per_ep = 3;   /* oracle, PID, current MLP */
    const int32_t header[4] = {0x4C4E4E31, episodes, rows_per_run, runs_per_ep};
    if (fwrite(header, sizeof(header), 1, fp) != 1) {
        fprintf(stderr, "write error on %s\n", path);
        fclose(fp);
        return;
    }

    uint32_t rng = seed | 1u;
    for (int ep = 0; ep < episodes; ++ep) {
        float u[4];
        for (int j = 0; j < 4; ++j) {
            rng = rng * 1664525u + 1013904223u;
            u[j] = (float)(rng >> 8) * (1.0f / 16777216.0f);
        }

        scenario_t sc;
        memset(&sc, 0, sizeof(sc));
        sc.name = "random";
        sc.amp_m = 0.02f + 0.10f * u[0];
        sc.wavelength_m = 1.10f + 1.30f * u[1];
        sc.offset_m = -0.025f + 0.05f * u[2];
        sc.heading_deg = -6.0f + 12.0f * u[3];
        sc.v_desired = 0.55f + 0.65f * ((float)((ep * 7) % 11) / 10.0f);
        sc.rail_v = 11.2f + 1.1f * ((float)((ep * 3) % 9) / 8.0f);
        sc.noise_std = (ep % 4 == 3) ? 0.06f : 0.0f;
        sc.gain_bias = (ep % 5 == 4) ? 0.10f : 0.0f;
        sc.offset_bias = (ep % 6 == 5) ? 0.04f : 0.0f;
        /* Missing channels are drawn from the whole array, not fixed slots. */
        sc.missing_a = (ep % 7 == 6) ? (ep * 3 + 1) % 8 : -1;
        sc.missing_b = (ep % 7 == 6) ? (ep * 5 + 4) % 8 : -1;
        sc.valid_prob = (ep % 7 == 6) ? 0.92f : 1.0f;

        metrics_t m;
        run_one(p, &sc, SRC_ORACLE, NULL, seed + (uint32_t)ep * 3u + 1u, &m, fp);
        run_one(p, &sc, SRC_PID, NULL, seed + (uint32_t)ep * 3u + 2u, &m, fp);
        /* Student-in-the-loop states, relabelled by the oracle (DAgger). */
        run_one(p, &sc, SRC_MLP, nncar_line_nn_trained(), seed + (uint32_t)ep * 3u + 3u,
                &m, fp);
    }

    if (ferror(fp)) {
        fprintf(stderr, "write error on %s\n", path);
    }
    fclose(fp);
    fprintf(stderr, "teacher: %d episodes, %d rows x %d float32 -> %s\n", episodes,
            episodes * runs_per_ep * rows_per_run, DUMP_ROW, path);
}

static void print_header(void)
{
    printf("\n%-12s %8s %8s %8s %8s %8s %8s %8s\n", "scenario", "pid_rms", "mlp_rms",
           "orc_rms", "pid_max", "mlp_max", "pid_dw", "mlp_dw");
}

int main(int argc, char **argv)
{
    const char *dump_path = NULL;
    int episodes = 120;
    uint32_t seed = 1u;
    bool check = false;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--check") == 0) {
            check = true;
        } else if (strcmp(a, "--dump-teacher") == 0 && i + 1 < argc) {
            dump_path = argv[++i];
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
        } else if (strcmp(a, "--trace") == 0) {
            k_trace = true;
            if (i + 1 < argc) {
                k_trace_index = atoi(argv[++i]);
            }
        } else {
            fprintf(stderr,
                    "usage: %s [--check] [--dump-teacher out.csv] "
                    "[--episodes N] [--seed S]\n"
                    "          [--ke K] [--ktheta K] [--alat F]  (oracle tuning)\n"
                    "          [--trace N]  (print scenario N per line tick)\n",
                    argv[0]);
            return 2;
        }
    }

    const nncar_profile_t *p = nncar_profile_default();

    if (dump_path != NULL) {
        dump_teacher(p, dump_path, episodes, seed);
        return 0;
    }

    const nncar_line_nn_t *trained = nncar_line_nn_trained();

    metrics_t pid[SUITE_COUNT];
    metrics_t mlp[SUITE_COUNT];
    metrics_t orc[SUITE_COUNT];

    printf("line A/B (profile %s, line_dt %.3f s, %d scenarios)\n", nncar_profile_name(),
           (double)p->line_dt_s, SUITE_COUNT);
    print_header();

    for (int i = 0; i < SUITE_COUNT; ++i) {
        const uint32_t s = (uint32_t)(i + 1);
        run_one(p, &k_suite[i], SRC_PID, NULL, s, &pid[i], NULL);
        run_one(p, &k_suite[i], SRC_MLP, trained, s, &mlp[i], NULL);
        run_one(p, &k_suite[i], SRC_ORACLE, NULL, s, &orc[i], NULL);

        printf("%-12s %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n", k_suite[i].name,
               (double)metric_rms(&pid[i]), (double)metric_rms(&mlp[i]),
               (double)metric_rms(&orc[i]), (double)pid[i].max_lat,
               (double)mlp[i].max_lat, (double)metric_domega(&pid[i]),
               (double)metric_domega(&mlp[i]));
    }

    const float pid_agg = aggregate(pid);
    const float mlp_agg = aggregate(mlp);
    const float orc_agg = aggregate(orc);

    printf("%-12s %8.4f %8.4f %8.4f\n", "aggregate", (double)pid_agg, (double)mlp_agg,
           (double)orc_agg);

    printf("\n%-12s %8s %8s %8s %8s %8s\n", "scenario", "out%_pid", "out%_mlp",
           "spd_pid", "spd_mlp", "recov_s");
    for (int i = 0; i < SUITE_COUNT; ++i) {
        printf("%-12s %8.1f %8.1f %8.3f %8.3f %8.3f\n", k_suite[i].name,
               (double)(metric_out(&pid[i]) * 100.0f),
               (double)(metric_out(&mlp[i]) * 100.0f), (double)metric_speed(&pid[i]),
               (double)metric_speed(&mlp[i]), (double)pid[i].recovery_s);
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

    if (!(mlp_agg <= 0.90f * pid_agg)) {
        printf("FAIL: aggregate MLP rms is not 10%% below PID\n");
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
