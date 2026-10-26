#include "nncar/motor.h"

#include "test_util.h"

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    const float v_max_eff = 1.5f;
    const float dt = p->motor_dt_s;

    nncar_motor_state_t st;
    nncar_motor_reset(&st);

    /* Positive error produces positive duty. */
    float u = nncar_pi_step(p, &st, 0.9f, 0.0f, dt, v_max_eff, true);
    CHECK(u > 0.0f);
    CHECK(u <= 1.0f);

    /* Sustained saturation must not wind the integrator past its limit. */
    for (int i = 0; i < 1000; ++i) {
        u = nncar_pi_step(p, &st, 0.9f, 0.0f, dt, v_max_eff, true);
    }
    CHECK_NEAR(u, 1.0f, 1.0e-6f);
    CHECK(fabsf(st.integral) <= p->motor_i_limit + 1.0e-6f);

    /* Back-calculation must release the output once the error disappears. */
    u = nncar_pi_step(p, &st, 0.9f, 0.9f, dt, v_max_eff, true);
    CHECK(u < 0.999f);

    /* Disabled path zeroes output and integrator. */
    u = nncar_pi_step(p, &st, 0.9f, 0.0f, dt, v_max_eff, false);
    CHECK_NEAR(u, 0.0f, 1.0e-9f);
    CHECK_NEAR(st.integral, 0.0f, 1.0e-9f);

    /* Deadband compensation mapping. */
    CHECK_NEAR(nncar_deadband_compensate(p, 0.0f), 0.0f, 1.0e-9f);
    CHECK(nncar_deadband_compensate(p, 0.5f) > 0.5f);
    CHECK(nncar_deadband_compensate(p, -0.5f) < -0.5f);
    CHECK(nncar_deadband_compensate(p, 1.0f) <= 1.0f + 1.0e-6f);

    TEST_END();
}
