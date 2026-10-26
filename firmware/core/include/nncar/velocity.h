#ifndef NNCAR_VELOCITY_H
#define NNCAR_VELOCITY_H

#include "nncar/profile.h"

typedef struct {
    float counts_per_meter;
    float alpha;
    float speed_mps;
    float raw_mps;
} nncar_velocity_t;

void nncar_velocity_init(nncar_velocity_t *v, const nncar_profile_t *p);

/* delta_counts is the quadrature-count delta accumulated over dt_s. */
float nncar_velocity_step(nncar_velocity_t *v, int32_t delta_counts, float dt_s);

#endif
