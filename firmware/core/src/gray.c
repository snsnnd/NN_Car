#include "nncar/gray.h"

#include <string.h>

void nncar_gray_process(const nncar_profile_t *p,
                        const float value[NNCAR_MAX_SENSORS],
                        const uint8_t exist[NNCAR_MAX_SENSORS],
                        const uint8_t valid[NNCAR_MAX_SENSORS],
                        nncar_line_estimate_t *out)
{
    float sum_w = 0.0f;
    float sum_wx = 0.0f;
    float sum_q = 0.0f;
    int valid_count = 0;

    memset(out, 0, sizeof(*out));

    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        bool usable = (exist[i] != 0u) && (valid[i] != 0u);
        float g = usable ? value[i] : 0.0f;

        if (usable && !nncar_finite(g)) {
            usable = false;
            g = 0.0f;
        }

        float q = usable ? nncar_clampf((g + 1.0f) * 0.5f, 0.0f, 1.0f) : 0.0f;
        if (usable) {
            valid_count += 1;
        }

        sum_q += q;
        sum_w += q;
        sum_wx += q * p->sensor_position_m[i];
    }

    out->valid_count = (uint16_t)valid_count;
    out->valid_count_norm = (float)valid_count / (float)NNCAR_MAX_SENSORS;
    out->confidence = nncar_clampf(sum_q / 2.0f, 0.0f, 1.0f);
    out->centroid_m = (sum_w > 1.0e-6f) ? (sum_wx / sum_w) : 0.0f;
    out->line_found = (out->confidence >= p->conf_nn_min) &&
                      (valid_count >= p->conf_valid_min);
}
