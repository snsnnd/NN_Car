#ifndef NNCAR_PROFILE_H
#define NNCAR_PROFILE_H

#include "nncar/types.h"

/*
 * Instance layer of the vehicle profile. Values are generated from
 * configs/vehicle_default.json by tools/python/gen_profile.py.
 * Do not hard-code safety thresholds in control code.
 */
typedef struct {
    /* geometry */
    float track_width_m;
    float wheel_radius_m;
    int sensor_count;
    float sensor_position_m[NNCAR_MAX_SENSORS];
    float sensor_to_axle_m;
    float position_ref_m;
    float axle_ref_m;

    /* speed */
    float v_max_nom_mps;
    float speed_ref_mps;
    float v_min_kappa_mps;

    /* line follower */
    float kappa_ref;
    float a_lat_max_mps2;
    float curve_slow_factor;
    float omega_ref_radps;

    /* confidence and handover */
    float conf_enter_low;
    float conf_exit_high;
    float conf_nn_min;
    int conf_valid_min;
    float conf_enter_hold_s;
    float conf_exit_hold_s;

    /* line PID teacher */
    float pid_kp;
    float pid_kd;
    float pid_d_filter_hz;

    /* recovery state machine */
    float recovery_speed_mps;
    float recovery_kappa_scale;
    float recovery_timeout_s;
    float recovery_ramp_s;

    /* motor loop */
    float motor_kff;
    float motor_kp;
    float motor_ki;
    float motor_tt_s;
    float motor_i_limit;
    float motor_deadband;
    float motor_alpha;

    /* supply rail */
    float rail_nominal_v;
    float rail_min_v;
    float rail_max_v;
    float rail_derate_v;
    float rail_stop_v;
    float rail_ovp_v;
    float rail_scale_min;
    float rail_scale_max;

    /* encoder */
    int encoder_ppr;
    float encoder_gear_ratio;
    int encoder_quadrature;
    float encoder_timeout_s;

    /* safety */
    float overspeed_factor;
    float hard_overspeed_factor;
    float hard_overspeed_hold_s;
    float ramp_down_s;
    float cmd_ttl_s;
    float twist_ttl_s;
    float rail_ttl_s;
    float temp_warn_c;
    float temp_stop_c;
    float encoder_min_pwm;
    float encoder_no_edge_s;
    float line_lost_timeout_s;
    float curvature_fault_s;
    float derate_rail_low;
    float derate_temp_warn;

    /* timing */
    float motor_dt_s;
    float line_dt_s;
} nncar_profile_t;

const nncar_profile_t *nncar_profile_default(void);
const char *nncar_profile_name(void);
const char *nncar_profile_version(void);
bool nncar_profile_is_verified(void);

/* v_max_eff = v_max_nom * clip(V_rail / V_rail_nom, scale_min, scale_max) */
static inline float nncar_v_max_eff(const nncar_profile_t *p, float v_rail)
{
    float scale = v_rail / p->rail_nominal_v;
    return p->v_max_nom_mps * nncar_clampf(scale, p->rail_scale_min, p->rail_scale_max);
}

static inline float nncar_counts_per_meter(const nncar_profile_t *p)
{
    float cpr = (float)p->encoder_ppr * p->encoder_gear_ratio * (float)p->encoder_quadrature;
    return cpr / (2.0f * NNCAR_PI * p->wheel_radius_m);
}

#endif
