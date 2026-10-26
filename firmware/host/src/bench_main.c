#include "nncar/profile.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Bench identification excitation + acquisition.
 *
 * The same CSV contract is used by real hardware logs: the firmware applies
 * the segment command, samples wheel speed at 500 Hz and exports the rows.
 * No deadband compensation is applied, otherwise the deadband is invisible.
 *
 * Excitation order:
 *   phase 1  positive deadband staircase  +0.01 .. +0.12
 *   phase 2  positive speed steps         +0.20 .. +0.90
 *   phase 3  rest (0), negative staircase -0.01 .. -0.12, reverse steps
 *   phase 4  rail sensitivity              pwm +0.5 at 11.0 / 10.5 / 12.0 V
 *
 * A rest segment is emitted before the reverse sweep so that coast-down from
 * the positive sweep cannot be mistaken for motion above the negative
 * deadband.
 */

#define MAX_SEGMENTS 48

typedef struct {
    float inertia_kgm2;
    float damping_nms;
    float torque_gain_nm;
    float deadband;
    float wheel_radius_m;
    float gain_scale;
    float damping_scale;
    float omega;
} bench_wheel_t;

typedef struct {
    int id;
    int phase;
    float pwm;
    float rail_v;
    float duration_s;
} segment_t;

static float wheel_step(bench_wheel_t *w, float pwm, float rail_v, float rail_nom, float dt_s)
{
    float mag = fabsf(pwm);
    float drive = 0.0f;

    if (mag > w->deadband) {
        drive = (mag - w->deadband) / (1.0f - w->deadband);
    }
    if (pwm < 0.0f) {
        drive = -drive;
    }

    float u_eff = drive * rail_v / rail_nom;
    float torque = w->torque_gain_nm * w->gain_scale * u_eff -
                   w->damping_nms * w->damping_scale * w->omega;

    w->omega += torque / w->inertia_kgm2 * dt_s;
    return w->omega * w->wheel_radius_m;
}

static float wheel_gain(const bench_wheel_t *w)
{
    return w->torque_gain_nm * w->gain_scale * w->wheel_radius_m /
           (w->damping_nms * w->damping_scale);
}

static float wheel_tau(const bench_wheel_t *w)
{
    return w->inertia_kgm2 / (w->damping_nms * w->damping_scale);
}

static bool push_segment(segment_t *segs, int *count, int max, int phase,
                         float pwm, float rail_v, float duration_s)
{
    if (*count >= max) {
        return false;
    }

    int i = *count;
    segs[i].id = i + 1;
    segs[i].phase = phase;
    segs[i].pwm = pwm;
    segs[i].rail_v = rail_v;
    segs[i].duration_s = duration_s;
    *count = i + 1;
    return true;
}

static int build_segments(segment_t *segs, int max)
{
    static const float up_levels[] = {0.2f, 0.35f, 0.5f, 0.7f, 0.9f};
    static const float reverse_levels[] = {-0.35f, -0.7f};
    static const float rail_levels[] = {11.0f, 10.5f, 12.0f};

    int n = 0;

    /* Phase 1: positive deadband staircase, 0.01 .. 0.12. */
    for (int i = 1; i <= 12; ++i) {
        if (!push_segment(segs, &n, max, 1, 0.01f * (float)i, 12.0f, 0.25f)) {
            return n;
        }
    }

    /* Phase 2: positive speed steps. */
    for (unsigned i = 0; i < sizeof(up_levels) / sizeof(up_levels[0]); ++i) {
        if (!push_segment(segs, &n, max, 2, up_levels[i], 12.0f, 0.40f)) {
            return n;
        }
    }

    /* Phase 3: rest, negative staircase, reverse steps. */
    if (!push_segment(segs, &n, max, 3, 0.0f, 12.0f, 0.50f)) {
        return n;
    }
    for (int i = 1; i <= 12; ++i) {
        if (!push_segment(segs, &n, max, 3, -0.01f * (float)i, 12.0f, 0.25f)) {
            return n;
        }
    }
    for (unsigned i = 0; i < sizeof(reverse_levels) / sizeof(reverse_levels[0]); ++i) {
        if (!push_segment(segs, &n, max, 3, reverse_levels[i], 12.0f, 0.40f)) {
            return n;
        }
    }

    /* Phase 4: rail sensitivity at a fixed command. */
    for (unsigned i = 0; i < sizeof(rail_levels) / sizeof(rail_levels[0]); ++i) {
        if (!push_segment(segs, &n, max, 4, 0.5f, rail_levels[i], 0.40f)) {
            return n;
        }
    }

    return n;
}

static void write_truth(const char *path, const bench_wheel_t *left,
                        const bench_wheel_t *right, float rail_nom)
{
    FILE *fp = fopen(path, "w");

    if (fp == NULL) {
        return;
    }

    fprintf(fp,
            "{\n"
            "  \"model\": \"first_order_dc_motor_with_deadband\",\n"
            "  \"rail_nominal_v\": %.6f,\n"
            "  \"deadband\": %.6f,\n"
            "  \"left\": {\n"
            "    \"static_gain_mps\": %.6f,\n"
            "    \"tau_s\": %.6f,\n"
            "    \"inertia_kgm2\": %.6g,\n"
            "    \"damping_nms\": %.6g,\n"
            "    \"torque_gain_nm\": %.6g\n"
            "  },\n"
            "  \"right\": {\n"
            "    \"static_gain_mps\": %.6f,\n"
            "    \"tau_s\": %.6f,\n"
            "    \"inertia_kgm2\": %.6g,\n"
            "    \"damping_nms\": %.6g,\n"
            "    \"torque_gain_nm\": %.6g\n"
            "  }\n"
            "}\n",
            (double)rail_nom, (double)left->deadband,
            (double)wheel_gain(left), (double)wheel_tau(left),
            (double)left->inertia_kgm2, (double)left->damping_nms,
            (double)left->torque_gain_nm,
            (double)wheel_gain(right), (double)wheel_tau(right),
            (double)right->inertia_kgm2, (double)right->damping_nms,
            (double)right->torque_gain_nm);

    fclose(fp);
}

int main(int argc, char **argv)
{
    const char *out_path = "bench.csv";
    const char *truth_path = NULL;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--truth") == 0 && i + 1 < argc) {
            truth_path = argv[++i];
        } else {
            fprintf(stderr, "usage: %s [--out bench.csv] [--truth truth.json]\n",
                    argv[0]);
            return 2;
        }
    }

    const nncar_profile_t *p = nncar_profile_default();
    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);

    /* Identifiable truth for the simulated bench: the right wheel is weaker
     * and slightly more damped, which the identification script must recover. */
    bench_wheel_t left;
    bench_wheel_t right;

    left.inertia_kgm2 = 2.0e-5f;
    left.damping_nms = 4.0e-4f;
    left.torque_gain_nm = left.damping_nms * (p->v_max_nom_mps / p->wheel_radius_m);
    left.deadband = p->motor_deadband;
    left.wheel_radius_m = p->wheel_radius_m;
    left.gain_scale = 1.0f;
    left.damping_scale = 1.0f;
    left.omega = 0.0f;

    right = left;
    right.gain_scale = 0.93f;
    right.damping_scale = 1.08f;

    segment_t segs[MAX_SEGMENTS];
    int seg_count = build_segments(segs, MAX_SEGMENTS);

    FILE *fp = fopen(out_path, "w");
    if (fp == NULL) {
        fprintf(stderr, "cannot open %s\n", out_path);
        return 1;
    }

    fputs("t_us,segment_id,phase,pwm,v_rail,v_left,v_right,w_left,w_right,load_nm\n", fp);

    uint64_t t_us = 0;
    int rows = 0;

    for (int s = 0; s < seg_count; ++s) {
        int ticks = (int)(segs[s].duration_s / dt + 0.5f);

        for (int k = 0; k < ticks; ++k) {
            float vl = wheel_step(&left, segs[s].pwm, segs[s].rail_v,
                                  p->rail_nominal_v, dt);
            float vr = wheel_step(&right, segs[s].pwm, segs[s].rail_v,
                                  p->rail_nominal_v, dt);

            fprintf(fp, "%llu,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    (unsigned long long)t_us, segs[s].id, segs[s].phase,
                    (double)segs[s].pwm, (double)segs[s].rail_v, (double)vl,
                    (double)vr, (double)left.omega, (double)right.omega, 0.0);

            t_us += dt_us;
            rows += 1;
        }
    }

    fclose(fp);

    if (truth_path != NULL) {
        write_truth(truth_path, &left, &right, p->rail_nominal_v);
    }

    fprintf(stderr,
            "bench: %d segments, %d rows, %.2fs\n"
            "  truth left  gain=%.4f tau=%.4f deadband=%.3f\n"
            "  truth right gain=%.4f tau=%.4f deadband=%.3f\n",
            seg_count, rows, (double)(t_us * 1.0e-6f),
            (double)wheel_gain(&left), (double)wheel_tau(&left),
            (double)left.deadband, (double)wheel_gain(&right),
            (double)wheel_tau(&right), (double)right.deadband);

    return 0;
}
