#ifndef NNCAR_LINE_H
#define NNCAR_LINE_H

#include "nncar/gray.h"
#include "nncar/profile.h"

typedef enum {
    NNCAR_LINE_TRACKING = 0,
    NNCAR_LINE_RECOVERY = 1
} nncar_line_mode_t;

typedef struct {
    nncar_line_mode_t mode;
    float low_hold_s;
    float high_hold_s;
    float recovery_time_s;
    float ramp_s;
    float last_centroid_m;
    bool has_last_centroid;
    float prev_centroid_m;
    float prev_t_s;
    float d_filtered;
    bool has_prev;
    nncar_twist_t recovery_cmd;
    uint32_t transitions;
} nncar_line_state_t;

void nncar_line_reset(nncar_line_state_t *st);

/*
 * Deterministic PID teacher plus recovery state machine.
 * Returns the commanded twist already clamped by the kinematic / lateral envelope.
 */
nncar_twist_t nncar_line_step(const nncar_profile_t *p,
                              nncar_line_state_t *st,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff,
                              float t_s,
                              float dt_s);

/* R3 network contract: y = [s_v, kappa_norm] -> TargetTwist. */
nncar_twist_t nncar_twist_from_kappa(const nncar_profile_t *p,
                                     float s_v,
                                     float kappa_norm,
                                     float v_desired,
                                     float v_max_eff);

/* Clamp omega by min(omega_kin, omega_lat) and clamp v by v_max_eff. */
nncar_twist_t nncar_twist_envelope(const nncar_profile_t *p,
                                   nncar_twist_t twist,
                                   float v_max_eff);

#endif
