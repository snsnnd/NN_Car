#include "nncar/motor.h"

#include "test_util.h"

#include <string.h>

static nncar_motor_input_t make_input(float target, float actual, float v_max_eff)
{
    nncar_motor_input_t in;

    memset(&in, 0, sizeof(in));
    in.target_mps = target;
    in.actual_mps = actual;
    in.v_max_eff = v_max_eff;
    in.rail_ratio = 1.0f;
    in.rail_exist = true;
    in.rail_valid = true;
    in.enable = true;
    return in;
}

static nncar_motor_cfg_t make_cfg(nncar_motor_mode_t mode, const nncar_motor_nn_t *nn)
{
    nncar_motor_cfg_t cfg;

    cfg.mode = mode;
    cfg.nn = nn;
    return cfg;
}

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    const float v_max_eff = 1.5f;
    const float dt = p->motor_dt_s;

    /* ---- PI: no feedforward, no deadband compensation ------------------ */
    {
        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI, NULL);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.4f, 0.0f, v_max_eff);
        nncar_motor_out_t out = nncar_motor_step(p, &st, &cfg, &in, dt);

        float expected = p->motor_kp * 0.4f + p->motor_ki * 0.4f * dt;
        CHECK_NEAR(out.u_pi, expected, 1.0e-6f);
        CHECK_NEAR(out.u_nn, 0.0f, 1.0e-9f);
        CHECK_NEAR(out.u_cmd, out.u_pi, 1.0e-9f);
        CHECK_NEAR(out.pwm, out.u_cmd, 1.0e-9f);
        CHECK(out.pwm == out.u_cmd);
        CHECK_NEAR(st.applied_pwm, out.pwm, 1.0e-9f);
    }

    /* ---- PI_FF: feedforward plus deterministic deadband compensation --- */
    {
        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI_FF, NULL);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.4f, 0.0f, v_max_eff);
        nncar_motor_out_t out = nncar_motor_step(p, &st, &cfg, &in, dt);

        float u_ff = p->motor_kff * 0.4f / v_max_eff;
        float expected = u_ff + p->motor_kp * 0.4f + p->motor_ki * 0.4f * dt;

        CHECK_NEAR(out.u_pi, expected, 1.0e-6f);
        CHECK(out.u_pi > 0.4f);
        CHECK_NEAR(out.pwm, nncar_deadband_compensate(p, out.u_cmd), 1.0e-9f);
        CHECK(out.pwm > out.u_cmd);
    }

    /* ---- anti-windup: saturation must not accumulate the integral ------ */
    {
        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI_FF, NULL);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.9f, 0.0f, v_max_eff);
        nncar_motor_out_t out = {0};

        for (int i = 0; i < 1000; ++i) {
            out = nncar_motor_step(p, &st, &cfg, &in, dt);
        }
        CHECK_NEAR(out.u_pi, 1.0f, 1.0e-6f);
        CHECK(fabsf(st.integral) <= p->motor_i_limit + 1.0e-6f);

        /* Releasing the error must unwrap the output immediately. */
        in.actual_mps = 0.9f;
        out = nncar_motor_step(p, &st, &cfg, &in, dt);
        CHECK(out.u_cmd < 0.999f);
    }

    /* ---- PI_RESIDUAL with a zero network is exactly PI_FF ------------- */
    {
        nncar_motor_nn_t nn;
        nncar_motor_nn_zero(&nn);

        float zero_x[NNCAR_MOTOR_INPUT_DIM] = {0.1f, 0.2f, 0.3f, 0.4f, 1.0f, 1.0f, 1.0f};
        CHECK_NEAR(nncar_motor_nn_eval(&nn, zero_x), 0.0f, 0.0f);

        nncar_motor_cfg_t ref_cfg = make_cfg(NNCAR_MOTOR_MODE_PI_FF, NULL);
        nncar_motor_cfg_t nn_cfg = make_cfg(NNCAR_MOTOR_MODE_PI_RESIDUAL, &nn);

        nncar_motor_state_t st_ref;
        nncar_motor_state_t st_nn;
        nncar_motor_reset(&st_ref);
        nncar_motor_reset(&st_nn);

        for (int i = 0; i < 200; ++i) {
            nncar_motor_input_t in = make_input(0.8f, 0.0005f * (float)i, v_max_eff);
            nncar_motor_out_t a = nncar_motor_step(p, &st_ref, &ref_cfg, &in, dt);
            nncar_motor_out_t b = nncar_motor_step(p, &st_nn, &nn_cfg, &in, dt);

            CHECK(a.u_pi == b.u_pi);
            CHECK(a.u_cmd == b.u_cmd);
            CHECK(a.pwm == b.pwm);
            CHECK_NEAR(b.u_nn, 0.0f, 0.0f);
        }

        /* A NULL network is the same safe degradation. */
        nncar_motor_cfg_t null_cfg = make_cfg(NNCAR_MOTOR_MODE_PI_RESIDUAL, NULL);
        nncar_motor_state_t st_null;
        nncar_motor_reset(&st_null);
        nncar_motor_input_t in = make_input(0.8f, 0.2f, v_max_eff);
        nncar_motor_out_t out = nncar_motor_step(p, &st_null, &null_cfg, &in, dt);
        CHECK_NEAR(out.u_nn, 0.0f, 0.0f);
    }

    /* ---- residual term is bounded by alpha and respects the sign ------- */
    {
        nncar_motor_nn_t nn;
        nncar_motor_nn_zero(&nn);
        nn.b4[0] = 1.0f; /* tanh(1) = 0.7616 */

        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI_RESIDUAL, &nn);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.2f, 0.2f, v_max_eff);
        nncar_motor_out_t out = nncar_motor_step(p, &st, &cfg, &in, dt);

        CHECK(out.u_nn > 0.0f);
        CHECK(out.u_nn <= p->motor_alpha + 1.0e-6f);
        CHECK_NEAR(out.u_nn, p->motor_alpha * tanhf(1.0f), 1.0e-6f);
    }

    /* ---- frozen 7-D input contract ------------------------------------ */
    {
        nncar_motor_input_t in = make_input(0.75f, -0.3f, v_max_eff);
        in.rail_ratio = 0.9f;
        in.rail_exist = true;
        in.rail_valid = false;

        float x[NNCAR_MOTOR_INPUT_DIM];
        nncar_motor_nn_input(p, &in, 0.42f, -0.11f, 0.3f, x);

        const float v_ref = p->v_max_nom_mps;
        CHECK_NEAR(x[0], 0.75f / v_ref, 1.0e-6f);
        CHECK_NEAR(x[1], -0.3f / v_ref, 1.0e-6f);
        CHECK_NEAR(x[2], -0.11f, 1.0e-6f);
        CHECK_NEAR(x[3], 0.42f, 1.0e-6f);
        CHECK_NEAR(x[4], 0.9f, 1.0e-6f);
        CHECK_NEAR(x[5], 1.0f, 1.0e-6f);
        CHECK_NEAR(x[6], 0.0f, 1.0e-6f);
        CHECK_NEAR(x[7], 0.3f / p->motor_i_limit, 1.0e-6f);

        /* Out-of-range inputs are clipped, not wrapped. */
        nncar_motor_input_t big = make_input(10.0f, 10.0f, v_max_eff);
        big.rail_ratio = 10.0f;
        nncar_motor_nn_input(p, &big, 5.0f, 5.0f, 1.0e9f, x);
        CHECK_NEAR(x[0], 2.0f, 1.0e-6f);
        CHECK_NEAR(x[1], 2.0f, 1.0e-6f);
        CHECK_NEAR(x[2], 1.0f, 1.0e-6f);
        CHECK_NEAR(x[3], 1.0f, 1.0e-6f);
        CHECK_NEAR(x[4], p->rail_scale_max, 1.0e-6f);
        CHECK_NEAR(x[7], 1.0f, 1.0e-6f);
    }

    /* ---- temporal feature: clipped error sum, maintained in every mode --- */
    {
        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI_FF, NULL);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.8f, 0.3f, v_max_eff);
        nncar_motor_step(p, &st, &cfg, &in, dt);

        float expect = nncar_clampf((0.8f - 0.3f) * dt, -p->motor_i_limit,
                                    p->motor_i_limit);
        CHECK_NEAR(st.e_acc, expect, 1.0e-7f);

        in.actual_mps = 0.1f;
        nncar_motor_step(p, &st, &cfg, &in, dt);
        expect = nncar_clampf(expect + (0.8f - 0.1f) * dt, -p->motor_i_limit,
                              p->motor_i_limit);
        CHECK_NEAR(st.e_acc, expect, 1.0e-7f);

        /* Disabling the loop clears the history, not just the integral. */
        in.enable = false;
        nncar_motor_step(p, &st, &cfg, &in, dt);
        CHECK_NEAR(st.e_acc, 0.0f, 1.0e-9f);
    }

    /* ---- disabled path zeroes everything ------------------------------ */
    {
        nncar_motor_cfg_t cfg = make_cfg(NNCAR_MOTOR_MODE_PI_FF, NULL);
        nncar_motor_state_t st;
        nncar_motor_reset(&st);

        nncar_motor_input_t in = make_input(0.9f, 0.0f, v_max_eff);
        in.enable = false;

        nncar_motor_out_t out = nncar_motor_step(p, &st, &cfg, &in, dt);
        CHECK_NEAR(out.u_cmd, 0.0f, 0.0f);
        CHECK_NEAR(out.pwm, 0.0f, 0.0f);
        CHECK_NEAR(st.integral, 0.0f, 0.0f);
        CHECK_NEAR(st.applied_pwm, 0.0f, 0.0f);
    }

    /* ---- deadband compensation mapping -------------------------------- */
    CHECK_NEAR(nncar_deadband_compensate(p, 0.0f), 0.0f, 1.0e-9f);
    CHECK(nncar_deadband_compensate(p, 0.5f) > 0.5f);
    CHECK(nncar_deadband_compensate(p, -0.5f) < -0.5f);
    CHECK_NEAR(nncar_deadband_compensate(p, 1.0f), 1.0f, 1.0e-9f);

    TEST_END();
}
