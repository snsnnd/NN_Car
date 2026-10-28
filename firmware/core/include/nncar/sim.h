#ifndef NNCAR_SIM_H
#define NNCAR_SIM_H

#include "nncar/map.h"
#include "nncar/profile.h"

/*
 * Host-side plant: differential drive or four-wheel skid-steer, per-wheel
 * first-order motor models and a synthetic gray array. Randomness is seeded.
 */
typedef struct {
    float x;
    float y;
    float theta;
    float w_left;
    float w_right;
    float wheel_omega[NNCAR_WHEEL_COUNT];
    float v_rail;
    float line_width_m;
    float track_amp_m;
    float track_wavelength_m;
    const nncar_map_t *map;
    float inertia_kgm2;
    float damping_nms;
    float torque_gain_nm;
    float wheel_radius_m;
    float track_width_m;
    float wheel_inertia_kgm2[NNCAR_WHEEL_COUNT];
    float wheel_damping_nms[NNCAR_WHEEL_COUNT];
    float wheel_torque_gain_nm[NNCAR_WHEEL_COUNT];
    float wheel_radius[NNCAR_WHEEL_COUNT];
    float wheel_stiction_nm[NNCAR_WHEEL_COUNT];
    float wheel_deadband[NNCAR_WHEEL_COUNT];
    float skid_yaw_gain; /* empirical attenuation for lateral scrub */
    nncar_drive_layout_t drive_layout;

    /*
     * Optional Coulomb friction at the wheel. Zero (the default) reproduces the
     * clean first-order plant bit for bit; a positive value adds a breakaway
     * torque, which is what makes the identified deadband an *apparent* one.
     */
    float motor_stiction_nm;

    /* sensor model perturbations; zero/full defaults reproduce the clean sim */
    uint32_t rng;
    float gray_noise_std;
    float gray_offset;
    float gray_gain[NNCAR_MAX_SENSORS];
    uint8_t gray_missing[NNCAR_MAX_SENSORS];
    float gray_valid_prob;
} nncar_sim_t;

void nncar_sim_init(nncar_sim_t *s, const nncar_profile_t *p,
                    float x0, float y0, float theta0, float v_rail);

/* Seed the perturbation RNG; every tool must call this for reproducibility. */
void nncar_sim_seed(nncar_sim_t *s, uint32_t seed);

float nncar_sim_track_y(float x, const nncar_sim_t *s);
void nncar_sim_set_map(nncar_sim_t *s, const nncar_map_t *map);
void nncar_sim_set_drive_layout(nncar_sim_t *s,
                                nncar_drive_layout_t drive_layout);
float nncar_sim_lateral_error(const nncar_sim_t *s);

void nncar_sim_read_gray(nncar_sim_t *s, const nncar_profile_t *p,
                         float value[NNCAR_MAX_SENSORS],
                         uint8_t exist[NNCAR_MAX_SENSORS],
                         uint8_t valid[NNCAR_MAX_SENSORS]);

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s);
void nncar_sim_step_skid4(nncar_sim_t *s, const nncar_profile_t *p,
                          const float pwm_wheel[NNCAR_WHEEL_COUNT], float dt_s);

#endif
