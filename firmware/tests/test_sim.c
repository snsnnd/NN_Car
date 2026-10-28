#include "nncar/safety.h"
#include "nncar/sim.h"
#include "nncar/sim_runner.h"
#include "nncar/stack.h"

#include "test_util.h"

static nncar_sim_run_result_t run_case(const nncar_profile_t *p,
                                       float offset_m, float heading_deg,
                                       float amp_m, float wavelength_m,
                                       float v_desired, float duration_s)
{
    nncar_sim_t sim;
    nncar_sim_init(&sim, p, 0.0f, offset_m, heading_deg * NNCAR_PI / 180.0f, 12.0f);
    sim.track_amp_m = amp_m;
    sim.track_wavelength_m = wavelength_m;

    nncar_sim_run_cfg_t cfg;
    cfg.duration_s = duration_s;
    cfg.v_desired = v_desired;

    return nncar_sim_run(p, &sim, cfg, NULL, NULL);
}

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();

    /* Straight line with lateral offset and heading error must converge. */
    nncar_sim_run_result_t res = run_case(p, 0.02f, 3.0f, 0.0f, 1.0f, 0.9f, 6.0f);
    printf("  straight: final=%.5f max=%.5f omega=%.3f faults=0x%x\n",
           (double)res.final_lat_m, (double)res.max_lat_m, (double)res.max_omega,
           res.fault_union);
    CHECK(res.final_lat_m < 0.006f);
    CHECK(res.fault_union == 0u);
    CHECK(res.max_omega <= 2.0f);

    /* Slalom at reduced speed must stay within the line corridor. */
    nncar_sim_run_result_t slalom = run_case(p, 0.0f, 0.0f, 0.02f, 0.6f, 0.6f, 8.0f);
    printf("  slalom:   final=%.5f max=%.5f omega=%.3f faults=0x%x\n",
           (double)slalom.final_lat_m, (double)slalom.max_lat_m,
           (double)slalom.max_omega, slalom.fault_union);
    CHECK(slalom.max_lat_m < 0.045f);
    CHECK(slalom.fault_union == 0u);

    /* Batch and incremental session APIs must produce the same C truth. */
    nncar_sim_t batch_sim;
    nncar_sim_t stepped_sim;
    nncar_sim_init(&batch_sim, p, 0.0f, 0.01f, 0.0f, 12.0f);
    nncar_sim_init(&stepped_sim, p, 0.0f, 0.01f, 0.0f, 12.0f);
    batch_sim.track_amp_m = 0.02f;
    stepped_sim.track_amp_m = 0.02f;
    batch_sim.track_wavelength_m = 0.6f;
    stepped_sim.track_wavelength_m = 0.6f;
    nncar_sim_run_cfg_t incremental_cfg = {0.12f, 0.6f};
    nncar_sim_run_result_t batch_result = nncar_sim_run(
        p, &batch_sim, incremental_cfg, NULL, NULL);
    nncar_sim_session_t session;
    nncar_sim_session_init(&session, p, &stepped_sim, incremental_cfg);
    nncar_log_row_t last_row;
    int step_count = 0;
    while (nncar_sim_session_step(&session, &last_row)) {
        ++step_count;
    }
    CHECK(step_count == (int)batch_result.ticks);
    CHECK(stepped_sim.x == batch_sim.x);
    CHECK(stepped_sim.y == batch_sim.y);
    CHECK(stepped_sim.theta == batch_sim.theta);
    CHECK(stepped_sim.w_left == batch_sim.w_left);
    CHECK(stepped_sim.w_right == batch_sim.w_right);
    CHECK(session.result.final_lat_m == batch_result.final_lat_m);
    CHECK(session.result.max_lat_m == batch_result.max_lat_m);
    CHECK(session.result.fault_union == batch_result.fault_union);

    /* Lower rail voltage must still track, at reduced top speed. */
    nncar_sim_t sim;
    nncar_sim_init(&sim, p, 0.0f, 0.01f, 0.0f, 11.0f);
    nncar_sim_run_cfg_t cfg;
    cfg.duration_s = 5.0f;
    cfg.v_desired = 0.8f;
    nncar_sim_run_result_t lowrail = nncar_sim_run(p, &sim, cfg, NULL, NULL);
    printf("  lowrail:  final=%.5f max=%.5f faults=0x%x\n",
           (double)lowrail.final_lat_m, (double)lowrail.max_lat_m,
           lowrail.fault_union);
    CHECK(lowrail.final_lat_m < 0.01f);
    CHECK((lowrail.fault_union & NNCAR_FAULT_UNDERVOLT) == 0u);

    /* Sensor fault/noise draws are reproducible from an explicit seed. */
    nncar_sim_t noisy_a;
    nncar_sim_t noisy_b;
    nncar_sim_init(&noisy_a, p, 0.0f, 0.0f, 0.0f, 12.0f);
    nncar_sim_init(&noisy_b, p, 0.0f, 0.0f, 0.0f, 12.0f);
    noisy_a.gray_noise_std = 0.08f;
    noisy_b.gray_noise_std = 0.08f;
    noisy_a.gray_offset = -0.05f;
    noisy_b.gray_offset = -0.05f;
    noisy_a.gray_valid_prob = 0.0f;
    noisy_b.gray_valid_prob = 0.0f;
    noisy_a.gray_gain[0] = 0.9f;
    noisy_b.gray_gain[0] = 0.9f;
    noisy_a.gray_missing[2] = 1u;
    noisy_b.gray_missing[2] = 1u;
    nncar_sim_seed(&noisy_a, 42u);
    nncar_sim_seed(&noisy_b, 42u);

    float gray_a[NNCAR_MAX_SENSORS];
    float gray_b[NNCAR_MAX_SENSORS];
    uint8_t exist_a[NNCAR_MAX_SENSORS];
    uint8_t exist_b[NNCAR_MAX_SENSORS];
    uint8_t valid_a[NNCAR_MAX_SENSORS];
    uint8_t valid_b[NNCAR_MAX_SENSORS];
    nncar_sim_read_gray(&noisy_a, p, gray_a, exist_a, valid_a);
    nncar_sim_read_gray(&noisy_b, p, gray_b, exist_b, valid_b);
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        CHECK(gray_a[i] == gray_b[i]);
        CHECK(exist_a[i] == exist_b[i]);
        CHECK(valid_a[i] == valid_b[i]);
        CHECK(valid_a[i] == 0u);
        if (i >= p->sensor_count || i == 2) {
            CHECK(exist_a[i] == 0u);
            CHECK(gray_a[i] == 0.0f);
        } else {
            CHECK(exist_a[i] == 1u);
        }
    }

    /* RL policy hook replaces line output while retaining the C stack contract. */
    nncar_stack_t policy_stack;
    nncar_stack_init(&policy_stack, p);
    nncar_stack_input_t policy_input = {0};
    policy_input.v_desired = 0.9f;
    policy_input.v_rail = p->rail_nominal_v;
    policy_input.v_rail_valid = true;
    policy_input.v_rail_exist = true;
    policy_input.actual_valid = true;
    policy_input.comm_ok = true;
    float policy_action[2] = {1.0f, 0.2f};
    nncar_stack_output_t policy_out = nncar_stack_step_action(
        &policy_stack, p, &policy_input, gray_a, exist_a, valid_a,
        policy_action, p->line_dt_s);
    CHECK_NEAR(policy_out.twist.v, 0.9f, 1.0e-5f);
    CHECK_NEAR(policy_out.twist.omega, 1.08f, 1.0e-4f);

    TEST_END();
}
