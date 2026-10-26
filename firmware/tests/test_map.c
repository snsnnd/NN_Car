#include "nncar/map.h"
#include "nncar/sim.h"
#include "nncar/sim_runner.h"

#include "test_util.h"

#include <string.h>

int main(void)
{
    const nncar_map_t *map = nncar_map_find("erth_grid");
    CHECK(map != NULL);
    CHECK(nncar_map_count() >= 1);
    CHECK(nncar_map_at(0) != NULL);
    CHECK(nncar_map_find("missing") == NULL);

    if (map == NULL) {
        TEST_END();
    }

    printf("  map: %s v%s segments=%d\n", map->name, map->version,
           map->segment_count);
    CHECK_NEAR(map->segment_count, 105, 0);
    CHECK_NEAR(map->x_min, 0.5f, 1e-4);
    CHECK_NEAR(map->x_max, 9.3f, 1e-4);
    CHECK_NEAR(map->y_min, 0.6f, 1e-4);
    CHECK_NEAR(map->y_max, 8.0f, 1e-4);
    CHECK_NEAR(map->start_x, 9.3f, 1e-4);
    CHECK_NEAR(map->start_y, 3.9f, 1e-4);
    CHECK_NEAR(map->start_theta, NNCAR_PI, 1e-4);

    /* ERTH edge 2-3 runs from (0.5, 8.0) to (2.1, 8.0). */
    float lat = 0.0f;
    int seg = -1;
    float d = nncar_map_query(map, 1.3f, 8.0f, &lat, &seg);
    printf("  on-line:  d=%.6f lat=%.6f seg=%d\n", (double)d, (double)lat, seg);
    CHECK_NEAR(d, 0.0f, 1e-5);
    CHECK_NEAR(lat, 0.0f, 1e-5);
    CHECK(seg >= 0);

    d = nncar_map_query(map, 1.3f, 8.01f, &lat, &seg);
    printf("  left:     d=%.6f lat=%.6f\n", (double)d, (double)lat);
    CHECK_NEAR(d, 0.01f, 1e-5);
    CHECK_NEAR(lat, 0.01f, 1e-5);

    d = nncar_map_query(map, 1.3f, 7.99f, &lat, &seg);
    printf("  right:    d=%.6f lat=%.6f\n", (double)d, (double)lat);
    CHECK_NEAR(d, 0.01f, 1e-5);
    CHECK_NEAR(lat, -0.01f, 1e-5);

    d = nncar_map_query(map, 1.3f, 9.0f, NULL, NULL);
    printf("  off-line: d=%.6f\n", (double)d);
    CHECK_NEAR(d, 1.0f, 1e-4);

    const nncar_profile_t *p = nncar_profile_default();

    /* Grey array on the line: inner sensors saturate, outer ones see floor. */
    nncar_sim_t sim;
    nncar_sim_init(&sim, p, 0.5f, 8.0f, 0.0f, 12.0f);
    nncar_sim_set_map(&sim, map);
    CHECK(sim.map == map);
    CHECK_NEAR(sim.line_width_m, 0.02f, 1e-6);

    float g[NNCAR_MAX_SENSORS];
    uint8_t exist[NNCAR_MAX_SENSORS];
    uint8_t valid[NNCAR_MAX_SENSORS];

    nncar_sim_read_gray(&sim, p, g, exist, valid);
    for (int i = 0; i < p->sensor_count; ++i) {
        CHECK(exist[i] == 1u);
        CHECK(valid[i] == 1u);
    }
    printf("  gray on:  %.2f %.2f %.2f %.2f | %.2f %.2f %.2f %.2f\n",
           (double)g[0], (double)g[1], (double)g[2], (double)g[3],
           (double)g[4], (double)g[5], (double)g[6], (double)g[7]);
    CHECK(g[3] > 0.99f);
    CHECK(g[4] > 0.99f);
    CHECK(g[0] < -0.99f);
    CHECK(g[7] < -0.99f);

    /* 8 cm off the line: even the innermost sensors are on the floor. */
    sim.y = 8.08f;
    nncar_sim_read_gray(&sim, p, g, exist, valid);
    for (int i = 0; i < p->sensor_count; ++i) {
        CHECK(g[i] < -0.99f);
    }

    /* Short straight run at node 2 must neither fault nor leave the line. */
    nncar_sim_init(&sim, p, 0.5f, 8.01f, 0.0f, 12.0f);
    nncar_sim_set_map(&sim, map);
    nncar_sim_run_cfg_t cfg;
    cfg.duration_s = 2.0f;
    cfg.v_desired = 0.6f;
    nncar_sim_run_result_t res = nncar_sim_run(p, &sim, cfg, NULL, NULL);
    printf("  run: final=%.5f max=%.5f omega=%.3f faults=0x%x\n",
           (double)res.final_lat_m, (double)res.max_lat_m,
           (double)res.max_omega, res.fault_union);
    CHECK(res.final_lat_m < 0.01f);
    CHECK(res.max_lat_m < 0.03f);
    CHECK(res.fault_union == 0u);

    TEST_END();
}
