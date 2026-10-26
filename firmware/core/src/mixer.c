#include "nncar/mixer.h"

nncar_mix_result_t nncar_mixer(const nncar_profile_t *p,
                               nncar_twist_t twist,
                               float v_max_eff)
{
    nncar_mix_result_t out;

    float half = 0.5f * p->track_width_m;

    float vl = twist.v - half * twist.omega;
    float vr = twist.v + half * twist.omega;

    float peak = fmaxf(fabsf(vl), fabsf(vr));
    float lambda = 1.0f;

    if (peak > 1.0e-6f) {
        lambda = fminf(1.0f, v_max_eff / peak);
    }
    lambda = nncar_clampf(lambda, 0.0f, 1.0f);

    out.wheels.left = lambda * vl;
    out.wheels.right = lambda * vr;
    out.lambda = lambda;
    out.v_target = twist.v;
    out.kappa_target = (twist.v > p->v_min_kappa_mps) ? (twist.omega / twist.v) : 0.0f;

    return out;
}

float nncar_kappa_achieved(const nncar_profile_t *p, nncar_wheel_speed_t actual)
{
    float v = 0.5f * (actual.left + actual.right);
    float denom = p->track_width_m * fmaxf(v, 0.05f);

    if (fabsf(denom) < 1.0e-6f) {
        return 0.0f;
    }

    return (actual.right - actual.left) / denom;
}
