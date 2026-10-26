#include "nncar/safety.h"
#include "nncar/sim.h"
#include "nncar/sim_runner.h"

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

    TEST_END();
}
