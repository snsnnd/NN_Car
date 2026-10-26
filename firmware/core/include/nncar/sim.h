#ifndef NNCAR_SIM_H
#define NNCAR_SIM_H

#include "nncar/map.h"
#include "nncar/profile.h"

/*
 * Host-side plant: two independent DC-motor first-order models, differential
 * kinematics and a synthetic gray array. No wall clock; the only randomness is
 * an explicit, seeded xorshift so every run is reproducible.
 *
 * track_amp_m != 0 selects the legacy analytic sine track; map != NULL selects
 * a concrete segment map and disables the sine track.
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
    const nncar_map_t *map;
    float inertia_kgm2;
    float damping_nms;
    float torque_gain_nm;
    float wheel_radius_m;
    float track_width_m;

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

/* Bind a concrete segment map (NULL restores the analytic sine track). */
void nncar_sim_set_map(nncar_sim_t *s, const nncar_map_t *map);

/* Signed cross-track offset, positive when the vehicle is left of the line. */
float nncar_sim_lateral_error(const nncar_sim_t *s);

/* Legacy analytic track; only valid when map == NULL. */
float nncar_sim_track_y(float x, const nncar_sim_t *s);

void nncar_sim_read_gray(nncar_sim_t *s, const nncar_profile_t *p,
                         float value[NNCAR_MAX_SENSORS],
                         uint8_t exist[NNCAR_MAX_SENSORS],
                         uint8_t valid[NNCAR_MAX_SENSORS]);

void nncar_sim_step(nncar_sim_t *s, const nncar_profile_t *p, nncar_pwm_t pwm, float dt_s);

#endif
