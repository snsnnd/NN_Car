#include "nncar/safety.h"

#include <string.h>

#include "test_util.h"

static nncar_safety_input_t healthy_input(const nncar_profile_t *p, uint64_t now_us)
{
    nncar_safety_input_t in;
    memset(&in, 0, sizeof(in));

    in.v_max_eff = p->v_max_nom_mps;
    in.wheel_target.left = 0.9f;
    in.wheel_target.right = 0.9f;
    in.twist_age_s = 0.0f;
    in.wheel_target_age_s = 0.0f;
    in.motor_cmd_age_s = 0.0f;

    in.actual_left.value = 0.9f;
    in.actual_left.exist = true;
    in.actual_left.valid = true;
    in.actual_left.timestamp_us = now_us;

    in.actual_right.value = 0.9f;
    in.actual_right.exist = true;
    in.actual_right.valid = true;
    in.actual_right.timestamp_us = now_us;

    in.v_rail.value = 12.0f;
    in.v_rail.exist = true;
    in.v_rail.valid = true;
    in.v_rail.timestamp_us = now_us;
    in.rail_age_s = 0.0f;
    in.now_us = now_us;

    in.encoder_activity = true;
    in.comm_ok = true;

    return in;
}

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    const float dt = p->motor_dt_s;
    const uint64_t now = 1000000u;

    nncar_safety_state_t st;
    nncar_safety_reset(&st);

    nncar_safety_input_t in = healthy_input(p, now);
    nncar_safety_output_t out = nncar_safety_step(p, &st, &in, dt);

    CHECK(out.active_faults == 0u);
    CHECK(out.latched_faults == 0u);
    CHECK_NEAR(out.ramp, 1.0f, 1.0e-6f);
    CHECK_NEAR(out.derate, 1.0f, 1.0e-6f);
    CHECK(!out.stop);
    CHECK(!out.force_zero_pwm);
    CHECK_NEAR(out.wheel_out.left, 0.9f, 1.0e-6f);

    /* Stale twist ramps down but is not an immediate cut. */
    nncar_safety_input_t stale = healthy_input(p, now);
    stale.twist_age_s = p->twist_ttl_s + 0.005f;
    out = nncar_safety_step(p, &st, &stale, dt);
    CHECK((out.active_faults & NNCAR_FAULT_STALE_TWIST) != 0u);
    CHECK(!out.force_zero_pwm);

    /* Advisory overspeed does not stop the car. */
    nncar_safety_reset(&st);
    nncar_safety_input_t fast = healthy_input(p, now);
    fast.actual_left.value = p->overspeed_factor * p->v_max_nom_mps + 0.05f;
    out = nncar_safety_step(p, &st, &fast, dt);
    CHECK((out.active_faults & NNCAR_FAULT_OVERSPEED) != 0u);
    CHECK(!out.stop);
    CHECK(out.latched_faults == 0u);

    /* Undervoltage below the stop threshold is immediate and latched. */
    nncar_safety_reset(&st);
    nncar_safety_input_t low = healthy_input(p, now);
    low.v_rail.value = p->rail_stop_v - 0.5f;
    for (int i = 0; i < 20; ++i) {
        out = nncar_safety_step(p, &st, &low, dt);
    }
    CHECK((out.active_faults & NNCAR_FAULT_UNDERVOLT) != 0u);
    CHECK(out.force_zero_pwm);
    CHECK(out.latched_faults != 0u);

    /* Fault stays latched until explicitly cleared. */
    nncar_safety_input_t ok = healthy_input(p, now);
    out = nncar_safety_step(p, &st, &ok, dt);
    CHECK(out.latched_faults != 0u);
    CHECK(out.ramp < 1.0f);
    CHECK(out.wheel_out.left < ok.wheel_target.left);

    nncar_safety_clear(&st, NNCAR_FAULT_UNDERVOLT);
    for (int i = 0; i < 200; ++i) {
        out = nncar_safety_step(p, &st, &ok, dt);
    }
    CHECK_NEAR(out.ramp, 1.0f, 1.0e-6f);
    CHECK(!out.stop);

    /* Temperature warning derates; stop threshold trips. */
    nncar_safety_reset(&st);
    nncar_safety_input_t warm = healthy_input(p, now);
    warm.temp_valid = true;
    warm.max_temp_c = p->temp_warn_c + 1.0f;
    out = nncar_safety_step(p, &st, &warm, dt);
    CHECK((out.active_faults & NNCAR_FAULT_TEMP_WARN) != 0u);
    CHECK_NEAR(out.derate, p->derate_temp_warn, 1.0e-6f);
    CHECK(!out.stop);

    nncar_safety_input_t hot = healthy_input(p, now);
    hot.temp_valid = true;
    hot.max_temp_c = p->temp_stop_c + 1.0f;
    out = nncar_safety_step(p, &st, &hot, dt);
    CHECK((out.active_faults & NNCAR_FAULT_TEMP_STOP) != 0u);
    CHECK(out.force_zero_pwm);

    /* E-stop is immediate. */
    nncar_safety_reset(&st);
    nncar_safety_input_t stop = healthy_input(p, now);
    stop.estop = true;
    out = nncar_safety_step(p, &st, &stop, dt);
    CHECK(out.force_zero_pwm);
    CHECK_NEAR(out.wheel_out.left, 0.0f, 1.0e-6f);
    CHECK((out.latched_faults & NNCAR_FAULT_ESTOP) != 0u);

    TEST_END();
}
