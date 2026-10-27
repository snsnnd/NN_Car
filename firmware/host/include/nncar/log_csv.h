#ifndef NNCAR_LOG_CSV_H
#define NNCAR_LOG_CSV_H

#include <stdio.h>
#include <stdint.h>

#include "nncar/types.h"

/* Host-only debugging/logging path. Not used on the MCU. */
typedef struct {
    FILE *fp;
} nncar_logger_t;

typedef struct {
    uint64_t t_us;
    int mode;
    float line_conf;
    float line_centroid_m;
    int line_found;
    int valid_count;
    float v_target;
    float omega_target;
    float v_max_eff;
    float lambda;
    float kappa_target;
    float wheel_v_l_target;
    float wheel_v_r_target;
    float wheel_v_l;
    float wheel_v_r;
    float u_l;
    float u_r;
    float pwm_l;
    float pwm_r;
    float v_rail;
    int v_rail_valid;
    unsigned safety_faults;
    int safety_latched;
    float safety_derate;
    float safety_ramp;
    int safety_stop;
    float x;
    float y;
    float theta;
    float recovery_ms;
    float gray_value[NNCAR_MAX_SENSORS];
    uint8_t gray_exist[NNCAR_MAX_SENSORS];
    uint8_t gray_valid[NNCAR_MAX_SENSORS];
} nncar_log_row_t;

int nncar_log_open(nncar_logger_t *lg, const char *path);
void nncar_log_close(nncar_logger_t *lg);
void nncar_log_write_header(nncar_logger_t *lg);
void nncar_log_write_row(nncar_logger_t *lg, const nncar_log_row_t *row);

#endif
