#ifndef NNCAR_STACK_H
#define NNCAR_STACK_H

#include "nncar/gray.h"
#include "nncar/line.h"
#include "nncar/mixer.h"
#include "nncar/motor.h"
#include "nncar/safety.h"

/*
 * Composition root for the fixed-rate control stack:
 *   Gray -> LineFollower -> Mixer -> Safety -> Motor PI -> PWM
 * The line module runs every profile.line_dt_s; the motor/safety path runs on
 * every call. No allocation, no HAL dependency.
 */
typedef struct {
    float v_desired;
    float v_rail;
    bool v_rail_valid;
    bool v_rail_exist;
    float rail_age_s;
    uint64_t now_us;
    float actual_left_mps;
    float actual_right_mps;
    bool actual_valid;
    bool encoder_activity;
    bool comm_ok;
    bool estop;
    bool nn_fault;
    float max_temp_c;
    bool temp_valid;
} nncar_stack_input_t;

typedef struct {
    nncar_twist_t twist;
    nncar_pwm_t pwm;
    nncar_line_estimate_t line;
    nncar_mix_result_t mix;
    nncar_safety_output_t safety;
    nncar_line_mode_t mode;
    float u_left;
    float u_right;
    bool line_lost;
    float recovery_ms;
} nncar_stack_output_t;

typedef struct {
    nncar_line_state_t line;
    nncar_motor_state_t motor_l;
    nncar_motor_state_t motor_r;
    nncar_safety_state_t safety;
    nncar_line_estimate_t estimate;
    nncar_mix_result_t mix;
    nncar_twist_t twist;
    float line_accum_s;
    float twist_age_s;
    float lambda_low_s;
} nncar_stack_t;

void nncar_stack_init(nncar_stack_t *st, const nncar_profile_t *p);

nncar_stack_output_t nncar_stack_step(nncar_stack_t *st,
                                      const nncar_profile_t *p,
                                      const nncar_stack_input_t *in,
                                      const float gray_value[NNCAR_MAX_SENSORS],
                                      const uint8_t gray_exist[NNCAR_MAX_SENSORS],
                                      const uint8_t gray_valid[NNCAR_MAX_SENSORS],
                                      float dt_s);

#endif
