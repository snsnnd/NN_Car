#ifndef NNCAR_SIM_H
#define NNCAR_SIM_H

#include "nncar/map.h"
#include "nncar/profile.h"

/*
 * Host-side plant. Legacy mode preserves the original independent wheel model;
 * optional realistic_dynamics couples wheel torque, tire force, chassis mass and
 * yaw inertia. All stochastic sensor/track perturbations are explicitly seeded.
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
    float track_extra_amp_m[2];
    float track_extra_wavelength_m[2];
    float track_extra_phase_rad[2];
    const nncar_map_t *map;
    float inertia_kgm2;
    float damping_nms;
    float torque_gain_nm;
    float wheel_radius_m;
    float track_width_m;
    float plant_motor_deadband;

    /* Optional body/tire dynamics. Disabled by default for legacy log replay. */
    bool realistic_dynamics;
    float body_v_mps;
    float body_omega_radps;
    float body_mass_kg;
    float yaw_inertia_kgm2;
    float tire_mu;
    float tire_stiffness_n_per_mps;
    float longitudinal_drag_n_per_mps;
    float yaw_drag_nm_per_radps;

    /*
     * Optional Coulomb friction at the wheel. Zero (the default) reproduces the
     * clean first-order plant bit for bit; a positive value adds a breakaway
     * torque, which is what makes the identified deadband an *apparent* one.
     */
    float motor_stiction_nm;

    /* sensor model perturbations; zero/full defaults reproduce the clean sim */
    uint32_t rng;
    float gray_noise_std;
    float gray_width_scale;
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
void nncar_sim_track_geometry(float x, const nncar_sim_t *s,
                              float *y, float *dy_dx, float *d2y_dx2);
void nncar_sim_set_map(nncar_sim_t *s, const nncar_map_t *map);
float nncar_sim_lateral_error(const nncar_sim_t *s);

void nncar_sim_read_gray(nncar_sim_t *s, const nncar_profile_t *p,
                         float value[NNCAR_MAX_SENSORS],
                         uint8_t exist[NNCAR_MAX_SENSORS],
                         uint8_t valid[NNCAR_MAX_SENSORS]);

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s);

#endif
