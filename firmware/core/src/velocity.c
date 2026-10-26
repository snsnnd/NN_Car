#include "nncar/velocity.h"

#include <string.h>

void nncar_velocity_init(nncar_velocity_t *v, const nncar_profile_t *p)
{
    memset(v, 0, sizeof(*v));
    v->counts_per_meter = nncar_counts_per_meter(p);
    /* 30 Hz first-order filter at the 500 Hz motor rate. */
    v->alpha = 1.0f - expf(-2.0f * NNCAR_PI * 30.0f * p->motor_dt_s);
    v->alpha = nncar_clampf(v->alpha, 0.0f, 1.0f);
}

float nncar_velocity_step(nncar_velocity_t *v, int32_t delta_counts, float dt_s)
{
    float raw = 0.0f;

    if (dt_s > 1.0e-6f && v->counts_per_meter > 1.0e-3f) {
        raw = (float)delta_counts / (v->counts_per_meter * dt_s);
    }

    v->raw_mps = raw;
    v->speed_mps += v->alpha * (raw - v->speed_mps);
    return v->speed_mps;
}
