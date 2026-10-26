#ifndef NNCAR_MIXER_H
#define NNCAR_MIXER_H

#include "nncar/profile.h"

typedef struct {
    nncar_wheel_speed_t wheels;
    float lambda;       /* 0..1, proportional desaturation factor */
    float kappa_target;
    float v_target;
} nncar_mix_result_t;

nncar_mix_result_t nncar_mixer(const nncar_profile_t *p,
                               nncar_twist_t twist,
                               float v_max_eff);

float nncar_kappa_achieved(const nncar_profile_t *p, nncar_wheel_speed_t actual);

#endif
