#ifndef NNCAR_SIM_H
#define NNCAR_SIM_H

#include "nncar/profile.h"

/*
 * Host-side plant: two independent DC-motor first-order models, differential
 * kinematics and a synthetic gray array. No RNG, no wall clock.
 */
typedef struct {
    float x;
    float y;
    float theta;
    float w_left;
    float w_right;
    float v_rail;
    float line_width_m;
    float track_amp_m;
    float track_wavelength_m;
    float inertia_kgm2;
    float damping_nms;
    float torque_gain_nm;
    float wheel_radius_m;
    float track_width_m;
} nncar_sim_t;

void nncar_sim_init(nncar_sim_t *s, const nncar_profile_t *p,
                    float x0, float y0, float theta0, float v_rail);

float nncar_sim_track_y(float x, const nncar_sim_t *s);

void nncar_sim_read_gray(const nncar_sim_t *s, const nncar_profile_t *p,
                         float value[NNCAR_MAX_SENSORS],
                         uint8_t exist[NNCAR_MAX_SENSORS],
                         uint8_t valid[NNCAR_MAX_SENSORS]);

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s);

#endif
