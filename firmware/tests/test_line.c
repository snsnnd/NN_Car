#include "nncar/line.h"

#include "test_util.h"

static void test_mlp_contract(const nncar_profile_t *p, float v_max_eff,
                              const nncar_gray_frame_t *clean,
                              const nncar_line_estimate_t *est)
{
    /* 68-D input contract: masks first, then values, then geometry. */
    float value[NNCAR_MAX_SENSORS];
    uint8_t exist[NNCAR_MAX_SENSORS];
    uint8_t valid[NNCAR_MAX_SENSORS];
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        value[i] = 0.0f;
        exist[i] = clean->exist[i];
        valid[i] = clean->valid[i];
    }
    value[0] = 0.5f;    /* usable */
    value[1] = -0.9f;   /* valid=0 -> must be zeroed in x */
    value[2] = 2.5f;    /* out of range -> clamped */
    exist[1] = 1u;
    valid[1] = 0u;

    nncar_gray_frame_t frame = {value, exist, valid};
    float x[NNCAR_LINE_INPUT_DIM];
    nncar_line_nn_input(p, &frame, est, 0.75f, x);

    CHECK_NEAR(x[0], 0.5f, 1.0e-6f);
    CHECK_NEAR(x[1], 0.0f, 1.0e-9f);
    CHECK_NEAR(x[2], 1.0f, 1.0e-6f);
    CHECK_NEAR(x[16], 1.0f, 1.0e-9f);
    CHECK_NEAR(x[17], 1.0f, 1.0e-9f);
    CHECK_NEAR(x[32], 1.0f, 1.0e-9f);
    CHECK_NEAR(x[33], 0.0f, 1.0e-9f);
    CHECK_NEAR(x[48], 0.0525f / p->position_ref_m, 1.0e-6f);
    CHECK_NEAR(x[64], 0.09f / p->axle_ref_m, 1.0e-6f);
    CHECK_NEAR(x[65], 0.75f / p->speed_ref_mps, 1.0e-6f);
    CHECK_NEAR(x[66], est->confidence, 1.0e-9f);
    CHECK_NEAR(x[67], est->valid_count_norm, 1.0e-9f);

    /* Null network or PID mode must fall back to the deterministic PID. */
    nncar_line_state_t pid;
    nncar_line_reset(&pid);
    nncar_line_state_t null_mlp;
    nncar_line_reset(&null_mlp);
    nncar_line_set_ctrl(&null_mlp, NNCAR_LINE_CTRL_MLP, NULL);
    CHECK_NEAR(nncar_line_step(p, &pid, clean, est, 0.9f, v_max_eff, 0.0f, p->line_dt_s).v,
               nncar_line_step(p, &null_mlp, clean, est, 0.9f, v_max_eff, 0.0f,
                               p->line_dt_s).v,
               1.0e-9f);

    /* A non-finite network falls back instead of commanding NaN. */
    static nncar_line_nn_t bad;
    nncar_line_nn_zero(&bad);
    bad.b1[0] = NAN;
    nncar_line_state_t nan_mlp;
    nncar_line_reset(&nan_mlp);
    nncar_line_set_ctrl(&nan_mlp, NNCAR_LINE_CTRL_MLP, &bad);
    nncar_twist_t nan_twist = nncar_line_step(p, &nan_mlp, clean, est, 0.9f, v_max_eff,
                                              0.0f, p->line_dt_s);
    CHECK(nncar_finite(nan_twist.v) && nncar_finite(nan_twist.omega));

    /* Zero network: s_v = 0.5, kappa = 0 -> halfway speed, no yaw. */
    static nncar_line_nn_t zero_nn;
    nncar_line_nn_zero(&zero_nn);
    float y[2];
    nncar_line_nn_eval(&zero_nn, x, y);
    CHECK_NEAR(y[0], 0.5f, 1.0e-6f);
    CHECK_NEAR(y[1], 0.0f, 1.0e-9f);

    nncar_line_state_t zero_state;
    nncar_line_reset(&zero_state);
    nncar_line_set_ctrl(&zero_state, NNCAR_LINE_CTRL_MLP, &zero_nn);
    /* The post-recovery ramp lasts recovery_ramp_s; step it out first. */
    nncar_twist_t ztw = {0.0f, 0.0f};
    for (int i = 0; i < 30; ++i) {
        ztw = nncar_line_step(p, &zero_state, clean, est, 0.9f, v_max_eff,
                              (float)i * p->line_dt_s, p->line_dt_s);
    }
    CHECK_NEAR(ztw.v, 0.45f, 1.0e-6f);
    CHECK_NEAR(ztw.omega, 0.0f, 1.0e-9f);

    /* Hand-set head: s_v = sigmoid(2) and kappa_norm = tanh(1). */
    static nncar_line_nn_t head;
    nncar_line_nn_zero(&head);
    head.b4[0] = 2.0f;
    head.b4[1] = 1.0f;
    nncar_line_nn_eval(&head, x, y);
    CHECK_NEAR(y[0], 1.0f / (1.0f + expf(-2.0f)), 1.0e-6f);
    CHECK_NEAR(y[1], tanhf(1.0f), 1.0e-6f);
}

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    const float v_max_eff = 1.5f;

    /* Envelope: kinematic limit binds before the lateral-acceleration limit. */
    nncar_twist_t raw = {1.4f, 100.0f};
    nncar_twist_t env = nncar_twist_envelope(p, raw, v_max_eff);
    float kin = 2.0f * (v_max_eff - 1.4f) / p->track_width_m;
    float lat = p->a_lat_max_mps2 / 1.4f;
    CHECK_NEAR(env.omega, fminf(kin, lat), 1.0e-5f);
    CHECK(env.omega < 100.0f);

    /* Zero speed forces zero yaw rate (no pivot turns). */
    nncar_twist_t stopped = nncar_twist_envelope(p, (nncar_twist_t){0.0f, 2.0f}, v_max_eff);
    CHECK_NEAR(stopped.omega, 0.0f, 1.0e-9f);

    /* Network contract: [s_v, kappa_norm] -> twist, with envelope applied. */
    nncar_twist_t nn_twist = nncar_twist_from_kappa(p, 1.0f, 1.0f, 1.4f, v_max_eff);
    CHECK_NEAR(nn_twist.v, 1.4f, 1.0e-6f);
    CHECK_NEAR(nn_twist.omega, fminf(kin, lat), 1.0e-5f);

    nncar_twist_t slow = nncar_twist_from_kappa(p, 0.5f, 0.0f, 1.4f, v_max_eff);
    CHECK_NEAR(slow.v, 0.7f, 1.0e-6f);
    CHECK_NEAR(slow.omega, 0.0f, 1.0e-9f);

    /* Recovery state machine entry. */
    nncar_line_state_t st;
    nncar_line_reset(&st);

    float gray_value[NNCAR_MAX_SENSORS] = {0};
    uint8_t gray_exist[NNCAR_MAX_SENSORS];
    uint8_t gray_valid[NNCAR_MAX_SENSORS];
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        gray_exist[i] = (i < p->sensor_count) ? 1u : 0u;
        gray_valid[i] = gray_exist[i];
    }
    nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};

    nncar_line_estimate_t est;
    est.centroid_m = 0.01f;
    est.confidence = 1.0f;
    est.valid_count = 8;
    est.valid_count_norm = 0.5f;
    est.line_found = true;

    float t = 0.0f;
    float dt = p->line_dt_s;
    nncar_twist_t tw = nncar_line_step(p, &st, &frame, &est, 0.9f, v_max_eff, t, dt);
    CHECK(st.mode == NNCAR_LINE_TRACKING);
    CHECK(tw.v > 0.0f);

    est.confidence = 0.0f;
    est.line_found = false;
    for (int i = 0; i < 4; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &frame, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK(st.mode == NNCAR_LINE_RECOVERY);
    CHECK_NEAR(tw.v, p->recovery_speed_mps, 1.0e-4f);
    CHECK(tw.omega > 0.0f); /* last valid line was to the left */
    CHECK(st.transitions == 1);

    /* Sustained loss times out and stops. */
    for (int i = 0; i < 60; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &frame, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK_NEAR(tw.v, 0.0f, 1.0e-9f);
    CHECK_NEAR(tw.omega, 0.0f, 1.0e-9f);

    /* Reacquisition returns to tracking with a ramp. */
    est.confidence = 1.0f;
    est.line_found = true;
    est.centroid_m = 0.0f;
    for (int i = 0; i < 6; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &frame, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK(st.mode == NNCAR_LINE_TRACKING);
    CHECK(st.transitions == 2);

    /* Zero desired speed outputs zero twist without consulting the line. */
    nncar_twist_t zero = nncar_line_step(p, &st, &frame, &est, 0.0f, v_max_eff, t, dt);
    CHECK_NEAR(zero.v, 0.0f, 1.0e-9f);
    CHECK_NEAR(zero.omega, 0.0f, 1.0e-9f);

    test_mlp_contract(p, v_max_eff, &frame, &est);

    TEST_END();
}
