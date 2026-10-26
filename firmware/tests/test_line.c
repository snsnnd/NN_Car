#include "nncar/line.h"

#include "test_util.h"

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

    nncar_line_estimate_t est;
    est.centroid_m = 0.01f;
    est.confidence = 1.0f;
    est.valid_count = 8;
    est.valid_count_norm = 0.5f;
    est.line_found = true;

    float t = 0.0f;
    float dt = p->line_dt_s;
    nncar_twist_t tw = nncar_line_step(p, &st, &est, 0.9f, v_max_eff, t, dt);
    CHECK(st.mode == NNCAR_LINE_TRACKING);
    CHECK(tw.v > 0.0f);

    est.confidence = 0.0f;
    est.line_found = false;
    for (int i = 0; i < 4; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK(st.mode == NNCAR_LINE_RECOVERY);
    CHECK_NEAR(tw.v, p->recovery_speed_mps, 1.0e-4f);
    CHECK(tw.omega > 0.0f); /* last valid line was to the left */
    CHECK(st.transitions == 1);

    /* Sustained loss times out and stops. */
    for (int i = 0; i < 60; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK_NEAR(tw.v, 0.0f, 1.0e-9f);
    CHECK_NEAR(tw.omega, 0.0f, 1.0e-9f);

    /* Reacquisition returns to tracking with a ramp. */
    est.confidence = 1.0f;
    est.line_found = true;
    est.centroid_m = 0.0f;
    for (int i = 0; i < 6; ++i) {
        t += dt;
        tw = nncar_line_step(p, &st, &est, 0.9f, v_max_eff, t, dt);
    }
    CHECK(st.mode == NNCAR_LINE_TRACKING);
    CHECK(st.transitions == 2);

    /* Zero desired speed outputs zero twist without consulting the line. */
    nncar_twist_t zero = nncar_line_step(p, &st, &est, 0.0f, v_max_eff, t, dt);
    CHECK_NEAR(zero.v, 0.0f, 1.0e-9f);
    CHECK_NEAR(zero.omega, 0.0f, 1.0e-9f);

    TEST_END();
}
