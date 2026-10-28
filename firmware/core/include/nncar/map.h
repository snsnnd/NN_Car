#ifndef NNCAR_MAP_H
#define NNCAR_MAP_H

#include <stddef.h>

typedef struct {
    float x0;
    float y0;
    float x1;
    float y1;
} nncar_map_seg_t;

typedef struct {
    char name[64];
    char version[16];
    nncar_map_seg_t *segments;
    size_t segment_count;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float line_width_m;
    float start_x;
    float start_y;
    float start_theta;
} nncar_map_t;

/* Nearest-segment distance and signed lateral error (positive is left). */
float nncar_map_query(const nncar_map_t *map, float x, float y,
                      float *lat_signed, size_t *seg_index);

#endif
