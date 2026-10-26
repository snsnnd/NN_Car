#include "nncar/map.h"

#include "nncar/types.h"

#include <float.h>
#include <string.h>

/* Generated table lives in nncar_maps_generated.c (see tools/python/gen_maps.py). */
extern const nncar_map_t *const nncar_map_table[];
extern const int nncar_map_table_count;

int nncar_map_count(void)
{
    return nncar_map_table_count;
}

const nncar_map_t *nncar_map_at(int index)
{
    if (index < 0 || index >= nncar_map_table_count) {
        return NULL;
    }
    return nncar_map_table[index];
}

const nncar_map_t *nncar_map_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (int i = 0; i < nncar_map_table_count; ++i) {
        if (strcmp(nncar_map_table[i]->name, name) == 0) {
            return nncar_map_table[i];
        }
    }
    return NULL;
}

float nncar_map_query(const nncar_map_t *map, float x, float y,
                      float *lat_signed, int *seg_index)
{
    float best_d2 = FLT_MAX;
    float best_lat = 0.0f;
    int best_i = -1;

    if (map == NULL || map->segments == NULL || map->segment_count <= 0) {
        return FLT_MAX;
    }

    for (int i = 0; i < map->segment_count; ++i) {
        const nncar_map_seg_t *seg = &map->segments[i];
        const float dx = seg->x1 - seg->x0;
        const float dy = seg->y1 - seg->y0;
        const float len2 = dx * dx + dy * dy;

        float t = 0.0f;
        if (len2 > 1.0e-12f) {
            t = ((x - seg->x0) * dx + (y - seg->y0) * dy) / len2;
            t = nncar_clampf(t, 0.0f, 1.0f);
        }

        const float cx = seg->x0 + t * dx;
        const float cy = seg->y0 + t * dy;
        const float ex = x - cx;
        const float ey = y - cy;
        const float d2 = ex * ex + ey * ey;

        if (d2 < best_d2) {
            best_d2 = d2;
            best_i = i;
            if (len2 > 1.0e-12f) {
                best_lat = (dx * ey - dy * ex) / sqrtf(len2);
            } else {
                best_lat = 0.0f;
            }
        }
    }

    if (lat_signed != NULL) {
        *lat_signed = best_lat;
    }
    if (seg_index != NULL) {
        *seg_index = best_i;
    }
    return sqrtf(best_d2);
}
