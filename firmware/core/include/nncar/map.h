#ifndef NNCAR_MAP_H
#define NNCAR_MAP_H

#include <stdbool.h>

/*
 * Static track maps for the host-side plant. A map is a set of straight line
 * segments in world metres; the grey array sees the union of those segments.
 * Maps are generated from JSON files under configs/maps by
 * tools/python/gen_maps.py, so adding another concrete field never touches C
 * code.
 */
typedef struct {
    float x0;
    float y0;
    float x1;
    float y1;
} nncar_map_seg_t;

typedef struct {
    const char *name;
    const char *version;
    const char *source;
    const nncar_map_seg_t *segments;
    int segment_count;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float line_width_m;
    /* Default spawn: position of start node, heading along start->toward. */
    float start_x;
    float start_y;
    float start_theta;
} nncar_map_t;

int nncar_map_count(void);
const nncar_map_t *nncar_map_at(int index);
const nncar_map_t *nncar_map_find(const char *name);

/*
 * Nearest-segment query. Returns the unsigned distance in metres. When
 * lat_signed is non-NULL it receives the cross-track offset measured along the
 * stored segment direction (positive = left of x0->x1); seg_index likewise.
 */
float nncar_map_query(const nncar_map_t *map, float x, float y,
                      float *lat_signed, int *seg_index);

#endif
