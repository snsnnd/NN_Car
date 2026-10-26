#include "nncar/calib.h"
#include "nncar/motor.h"
#include "nncar/profile.h"
#include "nncar/sim.h"
#include "nncar/velocity.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*
 * Closed-loop validation of the on-board identification, against the same
 * plant the A/B harness uses.
 *
 * The rig is deliberately dumb: it applies a command, integrates the wheel
 * into a quantized encoder position, and reports the speed the firmware would
 * measure. Everything that decides *how* to excite the motor lives in the
 * calibration mode, not here, so this bench can validate the estimator without
 * sharing any of its assumptions.
 */

#define PWM_STEP 0.01f
#define RAMP_LEVELS 20
#define RAMP_HOLD_S 0.10f
#define RAMP_WINDOW_S 0.04f
#define KFF_HOLD_S 0.30f   /* >= 6 tau, so the transient bias stays under 0.3% */
#define KFF_WINDOW_S 0.05f
#define MOTION_MPS 0.006f
#define V_RAIL 12.0f
#define MAX_ERR_DEADBAND 0.008f
#define MAX_ERR_KFF 0.05f

typedef struct {
    float truth_db;
    float stiction_nm;
} calib_case_t;

static const calib_case_t k_cases[] = {
    {0.05f, 0.0f},
    {0.07f, 1.0e-4f},
    {0.10f, 0.0f},
};

typedef struct {
    nncar_sim_t sim;
    nncar_profile_t truth;
    float cpm;
    float dt;
    float radius;
    float pos_m;
} rig_t;

static int32_t rig_counts(const rig_t *r)
{
    return (int32_t)floorf(r->pos_m * r->cpm);
}

static void rig_init(rig_t *r, float truth_db, float stiction_nm)
{
    memset(r, 0, sizeof(*r));
    r->truth = *nncar_profile_default();
    r->truth.motor_deadband = truth_db;

    nncar_sim_init(&r->sim, &r->truth, 0.0f, 0.0f, 0.0f, V_RAIL);
    nncar_sim_seed(&r->sim, 20261026u);
    r->sim.motor_stiction_nm = stiction_nm;

    r->cpm = nncar_counts_per_meter(&r->truth);
    r->dt = r->truth.motor_dt_s;
    r->radius = r->truth.wheel_radius_m;
}

/* Apply `pwm` for hold_s, then report the speed over the trailing window. */
static float rig_run(rig_t *r, float pwm, float hold_s, float window_s)
{
    const int ticks = (int)(hold_s / r->dt + 0.5f);
    const int win = (int)(window_s / r->dt + 0.5f);
    const int start = ticks - win;
    const nncar_pwm_t cmd = {pwm, pwm};
    int32_t begin = 0;

    for (int k = 0; k < ticks; ++k) {
        /* Counts must be latched before stepping, so the window spans exactly
         * `win` intervals; latching after the step loses one and biases low. */
        if (k == start) {
            begin = rig_counts(r);
        }
        nncar_sim_step(&r->sim, &r->truth, cmd, r->dt);
        r->pos_m += r->sim.w_left * r->radius * r->dt;
    }

    const int32_t end = rig_counts(r);

    return (float)(end - begin) / (r->cpm * window_s);
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

    const nncar_profile_t *base = nncar_profile_default();
    const float v_max_eff = nncar_v_max_eff(base, V_RAIL);
    int failures = 0;

    printf("%-8s %-9s %8s %8s %8s %7s %6s %6s\n", "case", "stiction", "db_true",
           "db_est", "db_err", "kff_est", "q_db", "q_kff");

    for (unsigned c = 0; c < sizeof(k_cases) / sizeof(k_cases[0]); ++c) {
        rig_t rig;
        rig_init(&rig, k_cases[c].truth_db, k_cases[c].stiction_nm);

        /* Stage 1: monotone |pwm| ramp, no deadband compensation. */
        nncar_calib_deadband_t db_st;
        nncar_calib_deadband_begin(&db_st, PWM_STEP, MOTION_MPS);
        for (int level = 1; level <= RAMP_LEVELS; ++level) {
            const float pwm = PWM_STEP * (float)level;
            nncar_calib_deadband_push(&db_st, pwm,
                                      rig_run(&rig, pwm, RAMP_HOLD_S, RAMP_WINDOW_S));
        }

        float db_est = 0.0f;
        float q_db = 0.0f;
        const bool db_ok = nncar_calib_deadband_result(&db_st, &db_est, &q_db);

        /* Stage 2: steady points with the actuator compensation the firmware
         * would apply, so the fit sees the plant gain rather than the deadband. */
        nncar_profile_t compensated = rig.truth;
        compensated.motor_deadband = db_est;

        nncar_calib_kff_t kff_st;
        nncar_calib_kff_begin(&kff_st);
        for (int i = 0; i < 3; ++i) {
            const float cmd = 0.4f + 0.2f * (float)i;
            const float sent = nncar_deadband_compensate(&compensated, cmd);
            nncar_calib_kff_push(&kff_st, rig_run(&rig, sent, KFF_HOLD_S, KFF_WINDOW_S),
                                 cmd, v_max_eff);
        }

        float kff_est = 0.0f;
        float q_kff = 0.0f;
        const bool kff_ok = nncar_calib_kff_result(&kff_st, &kff_est, &q_kff);

        /* Apparent deadband: stiction raises the breakaway knee. */
        const float torque_gain = rig.sim.torque_gain_nm;
        const float apparent =
            k_cases[c].truth_db +
            (1.0f - k_cases[c].truth_db) * k_cases[c].stiction_nm / torque_gain;
        const float db_err = fabsf(db_est - apparent);
        const float kff_err = fabsf(kff_est - 1.0f);

        printf("%-8u %-9.4f %8.4f %8.4f %8.4f %7.4f %6.3f %6.3f\n", c,
               (double)k_cases[c].stiction_nm, (double)k_cases[c].truth_db,
               (double)db_est, (double)db_err, (double)kff_est, (double)q_db,
               (double)q_kff);

        if (!db_ok || db_err > MAX_ERR_DEADBAND || !kff_ok || kff_err > MAX_ERR_KFF) {
            ++failures;
            if (!db_ok) {
                printf("  FAIL: deadband ramp never showed motion\n");
            } else if (db_err > MAX_ERR_DEADBAND) {
                printf("  FAIL: deadband error %.4f > %.4f (apparent %.4f)\n",
                       (double)db_err, (double)MAX_ERR_DEADBAND, (double)apparent);
            }
            if (!kff_ok) {
                printf("  FAIL: kff fit rejected\n");
            } else if (kff_err > MAX_ERR_KFF) {
                printf("  FAIL: kff %.4f is %.1f%% off unity\n", (double)kff_est,
                       (double)(kff_err * 100.0f));
            }
        }
    }

    const int total = (int)(sizeof(k_cases) / sizeof(k_cases[0]));
    printf("\ncalib bench: %d/%d cases within tolerance (db <= %.3f, kff <= %.1f%%)\n",
           total - failures, total, (double)MAX_ERR_DEADBAND,
           (double)(MAX_ERR_KFF * 100.0f));

    if (check && failures > 0) {
        printf("calib bench: FAILED (%d)\n", failures);
        return 1;
    }

    printf("calib bench: ok\n");
    return 0;
}
