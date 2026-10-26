#ifndef NNCAR_SAFETY_H
#define NNCAR_SAFETY_H

#include "nncar/profile.h"

#define NNCAR_FAULT_ESTOP            (1u << 0)
#define NNCAR_FAULT_STALE_TWIST      (1u << 1)
#define NNCAR_FAULT_STALE_WHEEL      (1u << 2)
#define NNCAR_FAULT_STALE_MOTOR_CMD  (1u << 3)
#define NNCAR_FAULT_SPEED_INVALID    (1u << 4)
#define NNCAR_FAULT_OVERSPEED        (1u << 5)
#define NNCAR_FAULT_HARD_OVERSPEED   (1u << 6)
#define NNCAR_FAULT_UNDERVOLT        (1u << 7)
#define NNCAR_FAULT_OVERVOLT         (1u << 8)
#define NNCAR_FAULT_RAIL_LOW         (1u << 9)
#define NNCAR_FAULT_TEMP_WARN        (1u << 10)
#define NNCAR_FAULT_TEMP_STOP        (1u << 11)
#define NNCAR_FAULT_NN               (1u << 12)
#define NNCAR_FAULT_ENCODER_NO_EDGE  (1u << 13)
#define NNCAR_FAULT_COMM             (1u << 14)
#define NNCAR_FAULT_LINE_LOST        (1u << 15)
#define NNCAR_FAULT_CURVATURE        (1u << 16)

#define NNCAR_FAULT_IMMEDIATE_MASK                                                  \
    (NNCAR_FAULT_ESTOP | NNCAR_FAULT_HARD_OVERSPEED | NNCAR_FAULT_UNDERVOLT |        \
     NNCAR_FAULT_OVERVOLT | NNCAR_FAULT_TEMP_STOP)

/* Advisory faults derate or degrade the loop but never stop the car. */
#define NNCAR_FAULT_ADVISORY_MASK                                                   \
    (NNCAR_FAULT_OVERSPEED | NNCAR_FAULT_RAIL_LOW | NNCAR_FAULT_TEMP_WARN |          \
     NNCAR_FAULT_NN | NNCAR_FAULT_CURVATURE)

#define NNCAR_FAULT_STOP_MASK (~NNCAR_FAULT_ADVISORY_MASK)

typedef struct {
    float v_max_eff;
    nncar_wheel_speed_t wheel_target;
    float twist_age_s;
    float wheel_target_age_s;
    float motor_cmd_age_s;
    nncar_sensor_t actual_left;
    nncar_sensor_t actual_right;
    nncar_sensor_t v_rail;
    float rail_age_s;
    uint64_t now_us;
    float max_temp_c;
    bool temp_valid;
    bool nn_fault;
    bool encoder_activity;
    bool comm_ok;
    bool estop;
    bool line_lost;
    bool curvature_fault;
    float applied_pwm_abs;
} nncar_safety_input_t;

typedef struct {
    nncar_wheel_speed_t wheel_out;
    float v_max_eff;
    float derate;
    float ramp;
    bool stop;
    bool force_zero_pwm;
    uint32_t active_faults;
    uint32_t latched_faults;
} nncar_safety_output_t;

typedef struct {
    uint32_t latched;
    float ramp;
    float speed_invalid_s;
    float hard_overspeed_s;
    float undervolt_s;
    float rail_low_s;
    float no_edge_s;
} nncar_safety_state_t;

void nncar_safety_reset(nncar_safety_state_t *st);

nncar_safety_output_t nncar_safety_step(const nncar_profile_t *p,
                                        nncar_safety_state_t *st,
                                        const nncar_safety_input_t *in,
                                        float dt_s);

/* Explicit reset is required for latched faults. */
void nncar_safety_clear(nncar_safety_state_t *st, uint32_t faults);

#endif
