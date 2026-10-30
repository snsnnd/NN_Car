#include "nncar/map_file.h"
#include "nncar/sim.h"
#include "nncar/sim_runner.h"

#include "test_util.h"

#include <math.h>
#include <string.h>

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (argc != 2) TEST_END();

    nncar_map_t map;
    char error[160];
    CHECK(nncar_map_load_json(argv[1], &map, error, sizeof(error)) == 0);
    if (map.segments == NULL) TEST_END();

    CHECK(strcmp(map.name, "oval") == 0);
    CHECK(strcmp(map.version, "1.0.0") == 0);
    CHECK(map.segment_count == 64u);
    CHECK_NEAR(map.line_width_m, 0.02f, 1e-6);
    CHECK_NEAR(map.x_min, 0.7f, 1e-5);
    CHECK_NEAR(map.x_max, 4.3f, 1e-5);
    CHECK_NEAR(map.y_min, -1.0f, 1e-5);
    CHECK_NEAR(map.y_max, 1.0f, 1e-5);
    CHECK_NEAR(map.start_theta, 1.65899f, 1e-4);

    float lateral = 0.0f;
    size_t segment = 0u;
    const float distance = nncar_map_query(&map, map.start_x, map.start_y,
                                           &lateral, &segment);
    CHECK_NEAR(distance, 0.0f, 1e-5);
    CHECK_NEAR(lateral, 0.0f, 1e-5);
    CHECK(segment < map.segment_count);

    const float left_x = map.start_x - sinf(map.start_theta) * 0.04f;
    const float left_y = map.start_y + cosf(map.start_theta) * 0.04f;
    (void)nncar_map_query(&map, left_x, left_y, &lateral, NULL);
    CHECK_NEAR(lateral, 0.04f, 0.003);

    const nncar_profile_t *profile = nncar_profile_default();
    nncar_sim_t sim;
    nncar_sim_init(&sim, profile, map.start_x, map.start_y,
                   map.start_theta, 12.0f);
    nncar_sim_set_map(&sim, &map);
    CHECK_NEAR(nncar_sim_lateral_error(&sim), 0.0f, 1e-5);
    CHECK_NEAR(sim.line_width_m, map.line_width_m, 1e-6);

    nncar_sim_run_cfg_t cfg = {24.0f, 0.4f};
    nncar_sim_run_result_t result = nncar_sim_run(profile, &sim, cfg, NULL, NULL);
    CHECK(result.ticks == 12000u);
    CHECK(result.max_lat_m < 0.01f);
    CHECK(result.fault_union == 0u);

    nncar_map_free(&map);
    TEST_END();
}
