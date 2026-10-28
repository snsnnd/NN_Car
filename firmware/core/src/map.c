#include "nncar/map.h"

#include <float.h>
#include <math.h>

float nncar_map_query(const nncar_map_t *map, float x, float y,
                      float *lat_signed, size_t *seg_index)
{
    float best_d2 = FLT_MAX;
    float best_lat = 0.0f;
    size_t best_i = 0u;

    if (map == NULL || map->segments == NULL || map->segment_count == 0u) {
        return FLT_MAX;
    }

    for (size_t i = 0u; i < map->segment_count; ++i) {
        const nncar_map_seg_t *seg = &map->segments[i];
        const float dx = seg->x1 - seg->x0;
        const float dy = seg->y1 - seg->y0;
        const float len2 = dx * dx + dy * dy;
        if (len2 <= 1.0e-12f) {
            continue;
        }

        float t = ((x - seg->x0) * dx + (y - seg->y0) * dy) / len2;
        t = fmaxf(0.0f, fminf(t, 1.0f));
        const float cx = seg->x0 + t * dx;
        const float cy = seg->y0 + t * dy;
        const float ex = x - cx;
        const float ey = y - cy;
        const float d2 = ex * ex + ey * ey;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_i = i;
            best_lat = (dx * ey - dy * ex) / sqrtf(len2);
        }
    }

    if (best_d2 == FLT_MAX) {
        return FLT_MAX;
    }
    if (lat_signed != NULL) {
        *lat_signed = best_lat;
    }
    if (seg_index != NULL) {
        *seg_index = best_i;
    }
    return sqrtf(best_d2);
}
