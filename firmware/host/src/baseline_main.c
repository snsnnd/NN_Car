#include "nncar/motor.h"
#include "nncar/profile.h"
#include "nncar/sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_STEPS 2048
#define MAX_STAGES 4

typedef struct {
    float t_start;
    float t_end;
    float target;
} stage_t;

typedef struct {
    float rms_err;
    float overshoot_pct;
    float rise_s;
    float settle_s;
    float final_err;
    float sat_frac;
} stage_metrics_t;

typedef struct {
    int n;
    float t[MAX_STEPS];
    float target[MAX_STEPS];
    float y[MAX_STEPS];
    float u[MAX_STEPS];
    float un[MAX_STEPS];
    float pwm[MAX_STEPS];
    float energy;
    float unn_abs_max;
} trace_t;

static const char *const k_mode_name[] = {"PI", "PI+FF", "PI+residual"};

/* Two-step schedule: standstill -> 0.8 -> 0.4 m/s. */
static const stage_t k_nominal[] = {
    {0.00f, 0.20f, 0.0f},
    {0.20f, 1.60f, 0.8f},
    {1.60f, 3.00f, 0.4f},
};

static float target_at(const stage_t *stage, int count, float t)
{
    for (int i = 0; i < count; ++i) {
        if (t < stage[i].t_end) {
            return stage[i].target;
        }
    }
    return stage[count - 1].target;
}

static float rail_at(float t, bool sag)
{
    if (!sag) {
        return 12.0f;
    }
    return (t < 0.60f) ? 12.0f : 10.8f;
}

static void run_mode(const nncar_profile_t *p, nncar_motor_mode_t mode,
                     const nncar_motor_nn_t *nn, const stage_t *stage, int nstage,
                     float duration, bool sag, trace_t *tr)
{
    const float dt = p->motor_dt_s;
    const int nsteps = (int)(duration / dt + 0.5f);

    nncar_sim_t s;
    nncar_sim_init(&s, p, 0.0f, 0.0f, 0.0f, rail_at(0.0f, sag));

    nncar_motor_state_t st;
    nncar_motor_reset(&st);

    nncar_motor_cfg_t cfg;
    cfg.mode = mode;
    cfg.nn = nn;

    memset(tr, 0, sizeof(*tr));
    tr->n = (nsteps < MAX_STEPS) ? nsteps : MAX_STEPS;

    for (int i = 0; i < tr->n; ++i) {
        float t = (float)i * dt;
        float rail = rail_at(t, sag);
        s.v_rail = rail;

        nncar_motor_input_t in;
        memset(&in, 0, sizeof(in));
        in.target_mps = target_at(stage, nstage, t);
        in.actual_mps = s.w_left * p->wheel_radius_m;
        in.v_max_eff = nncar_v_max_eff(p, rail);
        in.rail_ratio = rail / p->rail_nominal_v;
        in.rail_exist = true;
        in.rail_valid = true;
        in.enable = true;

        nncar_motor_out_t out = nncar_motor_step(p, &st, &cfg, &in, dt);

        nncar_pwm_t pwm;
        pwm.left = out.pwm;
        pwm.right = out.pwm;
        nncar_sim_step(&s, p, pwm, dt);

        tr->t[i] = t;
        tr->target[i] = in.target_mps;
        tr->y[i] = s.w_left * p->wheel_radius_m;
        tr->u[i] = out.u_cmd;
        tr->un[i] = out.u_nn;
        tr->pwm[i] = out.pwm;

        tr->energy += fabsf(out.pwm) * dt;
        tr->unn_abs_max = fmaxf(tr->unn_abs_max, fabsf(out.u_nn));
    }
}

static stage_metrics_t measure(const trace_t *tr, const stage_t *stage, int index)
{
    stage_metrics_t m;
    memset(&m, 0, sizeof(m));

    const float t0 = stage[index].t_start;
    const float t1 = stage[index].t_end;
    const float target = stage[index].target;
    const float prev = (index > 0) ? stage[index - 1].target : 0.0f;
    const float span = target - prev;

    float sum_sq = 0.0f;
    float peak = -1.0e9f;
    float last_out = 0.0f;
    float tail_sum = 0.0f;
    int count = 0;
    int tail_count = 0;
    float t10 = -1.0f;
    float t90 = -1.0f;

    for (int i = 0; i < tr->n; ++i) {
        float t = tr->t[i];
        if (t < t0 || t >= t1) {
            continue;
        }

        float y = tr->y[i];
        float err = y - target;

        sum_sq += err * err;

        if (span > 0.0f) {
            if (peak < y) {
                peak = y;
            }
        } else if (peak < -y) {
            peak = -y;
        }

        float progress = (fabsf(span) > 1.0e-6f) ? (y - prev) / span : 1.0f;
        if (t10 < 0.0f && progress >= 0.10f) {
            t10 = t;
        }
        if (t90 < 0.0f && progress >= 0.90f) {
            t90 = t;
        }

        if (fabsf(err) > 0.02f * fmaxf(fabsf(target), 0.05f)) {
            last_out = t;
        }

        if (t >= t1 - 0.10f) {
            tail_sum += y;
            tail_count += 1;
        }

        if (fabsf(tr->u[i]) >= 0.999f) {
            m.sat_frac += 1.0f;
        }
        count += 1;
    }

    if (count == 0) {
        return m;
    }

    m.rms_err = sqrtf(sum_sq / (float)count);
    m.sat_frac /= (float)count;
    m.final_err = (tail_count > 0) ? (tail_sum / (float)tail_count) - target : 0.0f;
    m.settle_s = (last_out > 0.0f) ? (last_out - t0) : 0.0f;

    if (t90 > 0.0f) {
        m.rise_s = (t10 > 0.0f) ? (t90 - t10) : (t90 - t0);
    } else {
        m.rise_s = -1.0f;
    }

    float peak_signed = (span >= 0.0f) ? peak : -peak;
    float direction = (span >= 0.0f) ? 1.0f : -1.0f;
    m.overshoot_pct = (fabsf(span) > 1.0e-6f)
                          ? fmaxf(0.0f, (peak_signed - target) * direction /
                                            fabsf(span) * 100.0f)
                          : 0.0f;

    return m;
}

static void print_scenario(const char *label, bool sag,
                           const trace_t *tr, const stage_t *stage, int nstage)
{
    printf("\n%s (rail %s)\n", label, sag ? "12.0 -> 10.8 V" : "12.0 V");
    printf("  %-12s %8s %8s %8s %8s %8s %8s\n",
           "mode", "rms_err", "overshoot", "rise_s", "settle_s", "ss_err", "sat%");

    for (int mode = 0; mode < 3; ++mode) {
        float rms = 0.0f;
        float overshoot = 0.0f;
        float rise = 0.0f;
        float settle = 0.0f;
        float ss = 0.0f;
        float sat = 0.0f;
        int measured = 0;

        for (int k = 1; k < nstage; ++k) {
            stage_metrics_t m = measure(&tr[mode], stage, k);
            rms += m.rms_err;
            overshoot = fmaxf(overshoot, m.overshoot_pct);
            rise = fmaxf(rise, m.rise_s);
            settle = fmaxf(settle, m.settle_s);
            ss = fmaxf(ss, fabsf(m.final_err));
            sat = fmaxf(sat, m.sat_frac);
            measured += 1;
        }

        if (measured > 0) {
            rms /= (float)measured;
        }

        printf("  %-12s %8.4f %7.1f%% %8.4f %8.4f %8.4f %7.1f%%\n",
               k_mode_name[mode], (double)rms, (double)overshoot, (double)rise,
               (double)settle, (double)ss, (double)(sat * 100.0f));
    }

    for (int mode = 0; mode < 3; ++mode) {
        printf("  %-12s energy=%.4f  max|u_nn|=%.4f\n", k_mode_name[mode],
               (double)tr[mode].energy, (double)tr[mode].unn_abs_max);
    }
}

int main(int argc, char **argv)
{
    bool check = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else {
            fprintf(stderr, "usage: %s [--check]\n", argv[0]);
            return 2;
        }
    }

    const nncar_profile_t *p = nncar_profile_default();
    const float duration = 3.00f;
    const int nstage = (int)(sizeof(k_nominal) / sizeof(k_nominal[0]));

    static trace_t tr[2][3];
    const bool sag_list[2] = {false, true};

    for (int s = 0; s < 2; ++s) {
        for (int mode = 0; mode < 3; ++mode) {
            run_mode(p, (nncar_motor_mode_t)mode, NULL, k_nominal, nstage,
                     duration, sag_list[s], &tr[s][mode]);
        }
        print_scenario((s == 0) ? "nominal" : "rail sag", sag_list[s],
                       tr[s], k_nominal, nstage);
    }

    if (!check) {
        return 0;
    }

    int failures = 0;

    /* Invariant 1: an untrained residual must reproduce PI+FF bit-for-bit. */
    for (int i = 0; i < tr[0][1].n; ++i) {
        if (tr[0][1].u[i] != tr[0][2].u[i] ||
            tr[0][1].pwm[i] != tr[0][2].pwm[i]) {
            printf("FAIL: zero residual diverges from PI+FF at t=%.4f\n",
                   (double)tr[0][1].t[i]);
            failures += 1;
            break;
        }
    }

    /* Invariant 2: every baseline settles inside 2% of the last target. */
    for (int mode = 0; mode < 3; ++mode) {
        stage_metrics_t m = measure(&tr[0][mode], k_nominal, nstage - 1);
        if (fabsf(m.final_err) > 0.02f * 0.4f) {
            printf("FAIL: %s steady-state error %.4f\n",
                   k_mode_name[mode], (double)m.final_err);
            failures += 1;
        }
    }

    /* Invariant 3: PI_FF rises faster than bare PI on the first step. */
    for (int s = 0; s < 1; ++s) {
        stage_metrics_t pi = measure(&tr[s][0], k_nominal, 1);
        stage_metrics_t ff = measure(&tr[s][1], k_nominal, 1);
        if (!(ff.rise_s > 0.0f && pi.rise_s > 0.0f && ff.rise_s < pi.rise_s)) {
            printf("FAIL: PI+FF rise %.4f not faster than PI %.4f\n",
                   (double)ff.rise_s, (double)pi.rise_s);
            failures += 1;
        }
    }

    /* Invariant 4: bare PI never applies deadband compensation. */
    for (int i = 0; i < tr[0][0].n; ++i) {
        if (tr[0][0].pwm[i] != tr[0][0].u[i]) {
            printf("FAIL: PI mode compensated the deadband at t=%.4f\n",
                   (double)tr[0][0].t[i]);
            failures += 1;
            break;
        }
    }

    if (failures != 0) {
        fprintf(stderr, "baseline: FAILED (%d)\n", failures);
        return 1;
    }

    printf("\nbaseline: ok\n");
    return 0;
}
