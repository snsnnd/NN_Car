#include "nncar/gray.h"

#include <string.h>

#include "test_util.h"

int main(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    float value[NNCAR_MAX_SENSORS];
    uint8_t exist[NNCAR_MAX_SENSORS];
    uint8_t valid[NNCAR_MAX_SENSORS];
    nncar_line_estimate_t est;

    memset(value, 0, sizeof(value));
    memset(exist, 0, sizeof(exist));
    memset(valid, 0, sizeof(valid));

    /* Two adjacent sensors on the line -> confidence saturates at 1. */
    value[2] = 1.0f;
    exist[2] = 1u;
    valid[2] = 1u;
    value[3] = 1.0f;
    exist[3] = 1u;
    valid[3] = 1u;

    nncar_gray_process(p, value, exist, valid, &est);
    CHECK_NEAR(est.centroid_m, 0.5f * (p->sensor_position_m[2] + p->sensor_position_m[3]),
               1.0e-5f);
    CHECK_NEAR(est.confidence, 1.0f, 1.0e-5f);
    CHECK(est.valid_count == 2);
    CHECK(est.line_found);
    CHECK_NEAR(est.valid_count_norm, 2.0f / 16.0f, 1.0e-6f);

    /* All background -> no confidence, no line. */
    for (int i = 0; i < p->sensor_count; ++i) {
        value[i] = -1.0f;
        exist[i] = 1u;
        valid[i] = 1u;
    }
    nncar_gray_process(p, value, exist, valid, &est);
    CHECK_NEAR(est.confidence, 0.0f, 1.0e-6f);
    CHECK_NEAR(est.centroid_m, 0.0f, 1.0e-6f);
    CHECK(!est.line_found);
    CHECK(est.valid_count == (uint16_t)p->sensor_count);

    /* Missing hardware must be excluded and never produce NaN. */
    exist[0] = 0u;
    valid[0] = 1u;
    value[0] = NAN;
    nncar_gray_process(p, value, exist, valid, &est);
    CHECK(nncar_finite(est.centroid_m));
    CHECK(est.valid_count == (uint16_t)(p->sensor_count - 1));

    /* exist=1, valid=0 (stale) also excluded. */
    exist[0] = 1u;
    valid[0] = 0u;
    nncar_gray_process(p, value, exist, valid, &est);
    CHECK(est.valid_count == (uint16_t)(p->sensor_count - 1));

    TEST_END();
}
