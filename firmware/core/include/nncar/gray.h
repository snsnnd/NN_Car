#ifndef NNCAR_GRAY_H
#define NNCAR_GRAY_H

#include "nncar/profile.h"

/*
 * Gray array contract:
 *   - index 0..sensor_count-1 follow the physical order left -> right;
 *   - sensor_position_m[i] is positive toward the vehicle's LEFT;
 *   - value is calibrated to [-1, 1] with +1 = line, -1 = background.
 */
typedef struct {
    float centroid_m;      /* + = line to the left of the axle */
    float confidence;      /* 0..1, frozen R3 definition */
    float valid_count_norm;
    uint16_t valid_count;
    bool line_found;
} nncar_line_estimate_t;

void nncar_gray_process(const nncar_profile_t *p,
                        const float value[NNCAR_MAX_SENSORS],
                        const uint8_t exist[NNCAR_MAX_SENSORS],
                        const uint8_t valid[NNCAR_MAX_SENSORS],
                        nncar_line_estimate_t *out);

#endif
