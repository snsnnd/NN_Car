#include "nncar/mixer.h"

#include "test_util.h"

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();

    nncar_twist_t twist;
    twist.v = 1.4f;
    twist.omega = 4.0f;

    float v_max_eff = 1.5f;
    nncar_mix_result_t mix = nncar_mixer(p, twist, v_max_eff);

    float vl = twist.v - 0.5f * p->track_width_m * twist.omega;
    float vr = twist.v + 0.5f * p->track_width_m * twist.omega;
    CHECK_NEAR(vl, 1.1f, 1.0e-5f);
    CHECK_NEAR(vr, 1.7f, 1.0e-5f);

    float expected_lambda = v_max_eff / vr;
    CHECK_NEAR(mix.lambda, expected_lambda, 1.0e-5f);
    CHECK_NEAR(mix.wheels.right, v_max_eff, 1.0e-5f);
    CHECK_NEAR(mix.wheels.left, expected_lambda * vl, 1.0e-5f);

    /* Proportional desaturation preserves the curvature ratio. */
    float ratio_before = (vr - vl) / (vr + vl);
    float ratio_after = (mix.wheels.right - mix.wheels.left) /
                        (mix.wheels.right + mix.wheels.left);
    CHECK_NEAR(ratio_after, ratio_before, 1.0e-5f);

    CHECK_NEAR(mix.kappa_target, twist.omega / twist.v, 1.0e-5f);
    CHECK_NEAR(mix.v_target, twist.v, 1.0e-6f);

    /* Straight line is never desaturated. */
    nncar_twist_t straight = {0.8f, 0.0f};
    nncar_mix_result_t s = nncar_mixer(p, straight, v_max_eff);
    CHECK_NEAR(s.lambda, 1.0f, 1.0e-6f);
    CHECK_NEAR(s.wheels.left, 0.8f, 1.0e-6f);
    CHECK_NEAR(s.wheels.right, 0.8f, 1.0e-6f);

    /* Achieved curvature from measured wheel speeds. */
    nncar_wheel_speed_t actual = {0.6f, 0.9f};
    float v_actual = 0.75f;
    float kappa = nncar_kappa_achieved(p, actual);
    CHECK_NEAR(kappa, (0.9f - 0.6f) / (p->track_width_m * fmaxf(v_actual, 0.05f)), 1.0e-5f);

    TEST_END();
}
