#include "nncar/log_csv.h"

#include <string.h>

static const char *k_header =
    "t_us,mode,line_conf,line_centroid_m,line_found,valid_count,"
    "v_target,omega_target,v_max_eff,lambda,kappa_target,"
    "wheel_v_l_target,wheel_v_r_target,wheel_v_l,wheel_v_r,"
    "u_l,u_r,pwm_l,pwm_r,v_rail,v_rail_valid,"
    "safety_faults,safety_latched,safety_derate,safety_ramp,safety_stop,"
    "x,y,theta,recovery_ms";

int nncar_log_open(nncar_logger_t *lg, const char *path)
{
    lg->fp = fopen(path, "w");
    return (lg->fp != NULL) ? 0 : -1;
}

void nncar_log_close(nncar_logger_t *lg)
{
    if (lg->fp != NULL) {
        fclose(lg->fp);
        lg->fp = NULL;
    }
}

void nncar_log_write_header(nncar_logger_t *lg)
{
    if (lg->fp != NULL) {
        fputs(k_header, lg->fp);
        for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
            fprintf(lg->fp, ",gray_value_%02d,gray_exist_%02d,gray_valid_%02d", i, i, i);
        }
        fputc('\n', lg->fp);
    }
}

void nncar_log_write_row(nncar_logger_t *lg, const nncar_log_row_t *r)
{
    if (lg->fp == NULL) {
        return;
    }

    fprintf(lg->fp,
            "%llu,%d,%.6f,%.6f,%d,%d,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%d,"
            "%u,%d,%.6f,%.6f,%d,"
            "%.6f,%.6f,%.6f,%.3f",
            (unsigned long long)r->t_us, r->mode, (double)r->line_conf,
            (double)r->line_centroid_m, r->line_found, r->valid_count,
            (double)r->v_target, (double)r->omega_target, (double)r->v_max_eff,
            (double)r->lambda, (double)r->kappa_target,
            (double)r->wheel_v_l_target, (double)r->wheel_v_r_target,
            (double)r->wheel_v_l, (double)r->wheel_v_r,
            (double)r->u_l, (double)r->u_r, (double)r->pwm_l, (double)r->pwm_r,
            (double)r->v_rail, r->v_rail_valid,
            r->safety_faults, r->safety_latched, (double)r->safety_derate,
            (double)r->safety_ramp, r->safety_stop,
            (double)r->x, (double)r->y, (double)r->theta, (double)r->recovery_ms);

    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        fprintf(lg->fp, ",%.6f,%u,%u", (double)r->gray_value[i],
                (unsigned)r->gray_exist[i], (unsigned)r->gray_valid[i]);
    }
    fputc('\n', lg->fp);
}
