#include "nncar/mixer.h"
#include "nncar/motor.h"
#include "nncar/profile.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_COLUMNS 64
#define LINE_MAX_LEN 4096

typedef struct {
    int t_us;
    int v_target;
    int omega_target;
    int v_max_eff;
    int lambda;
    int wheel_v_l_target;
    int wheel_v_r_target;
    int wheel_v_l;
    int wheel_v_r;
    int u_l;
    int u_r;
    int safety_faults;
} columns_t;

static int find_column(const char *header, const char *name)
{
    char buffer[LINE_MAX_LEN];
    int index = 0;
    size_t len = strlen(header);

    memcpy(buffer, header, len + 1);

    for (char *token = strtok(buffer, ","); token != NULL; token = strtok(NULL, ",")) {
        char *end = token + strlen(token);
        while (end > token && (*end == '\n' || *end == '\r' || *end == ' ')) {
            --end;
            *end = '\0';
        }
        if (strcmp(token, name) == 0) {
            return index;
        }
        index += 1;
    }
    return -1;
}

static int split_row(char *line, double *values, int max_values)
{
    int count = 0;

    for (char *token = strtok(line, ","); token != NULL; token = strtok(NULL, ",")) {
        if (count >= max_values) {
            return -1;
        }
        values[count] = strtod(token, NULL);
        count += 1;
    }
    return count;
}

static int require(const char *header, const char *name)
{
    int index = find_column(header, name);
    if (index < 0) {
        fprintf(stderr, "replay: missing column '%s'\n", name);
    }
    return index;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s run.csv\n", argv[0]);
        return 2;
    }

    FILE *fp = fopen(argv[1], "r");
    if (fp == NULL) {
        fprintf(stderr, "replay: cannot open %s\n", argv[1]);
        return 2;
    }

    char header[LINE_MAX_LEN];
    if (fgets(header, sizeof(header), fp) == NULL) {
        fprintf(stderr, "replay: empty file\n");
        fclose(fp);
        return 2;
    }

    const nncar_profile_t *p = nncar_profile_default();
    const float dt = p->motor_dt_s;

    columns_t col;
    col.t_us = require(header, "t_us");
    col.v_target = require(header, "v_target");
    col.omega_target = require(header, "omega_target");
    col.v_max_eff = require(header, "v_max_eff");
    col.lambda = require(header, "lambda");
    col.wheel_v_l_target = require(header, "wheel_v_l_target");
    col.wheel_v_r_target = require(header, "wheel_v_r_target");
    col.wheel_v_l = require(header, "wheel_v_l");
    col.wheel_v_r = require(header, "wheel_v_r");
    col.u_l = require(header, "u_l");
    col.u_r = require(header, "u_r");
    col.safety_faults = require(header, "safety_faults");

    if (col.t_us < 0 || col.v_target < 0 || col.omega_target < 0 ||
        col.v_max_eff < 0 || col.lambda < 0 || col.wheel_v_l_target < 0 ||
        col.wheel_v_r_target < 0 || col.wheel_v_l < 0 || col.wheel_v_r < 0 ||
        col.u_l < 0 || col.u_r < 0 || col.safety_faults < 0) {
        fclose(fp);
        return 1;
    }

    nncar_motor_state_t motor_l;
    nncar_motor_state_t motor_r;
    nncar_motor_reset(&motor_l);
    nncar_motor_reset(&motor_r);

    double values[MAX_COLUMNS];
    char line[LINE_MAX_LEN];
    float prev_left = 0.0f;
    float prev_right = 0.0f;
    float max_lambda_err = 0.0f;
    float max_wheel_err = 0.0f;
    float max_u_err = 0.0f;
    int rows = 0;
    int violations = 0;
    int skipped_fault_rows = 0;

    while (fgets(line, sizeof(line), fp) != NULL) {
        int count = split_row(line, values, MAX_COLUMNS);
        if (count <= 0) {
            continue;
        }

        rows += 1;

        float v_target = (float)values[col.v_target];
        float omega_target = (float)values[col.omega_target];
        float v_max_eff = (float)values[col.v_max_eff];
        unsigned faults = (unsigned)values[col.safety_faults];

        nncar_twist_t twist;
        twist.v = v_target;
        twist.omega = omega_target;

        nncar_mix_result_t mix = nncar_mixer(p, twist, v_max_eff);

        max_lambda_err = fmaxf(max_lambda_err,
                               fabsf(mix.lambda - (float)values[col.lambda]));

        float wheel_err = fmaxf(
            fabsf(mix.wheels.left - (float)values[col.wheel_v_l_target]),
            fabsf(mix.wheels.right - (float)values[col.wheel_v_r_target]));
        max_wheel_err = fmaxf(max_wheel_err, wheel_err);

        if (faults != 0u) {
            skipped_fault_rows += 1;
            prev_left = (float)values[col.wheel_v_l];
            prev_right = (float)values[col.wheel_v_r];
            continue;
        }

        nncar_motor_cfg_t cfg;
        cfg.mode = NNCAR_MOTOR_MODE_PI_FF;
        cfg.nn = NULL;

        nncar_motor_input_t min;
        memset(&min, 0, sizeof(min));
        min.v_max_eff = v_max_eff;
        min.rail_ratio = 1.0f;
        min.rail_exist = true;
        min.rail_valid = true;
        min.enable = true;

        min.target_mps = mix.wheels.left;
        min.actual_mps = prev_left;
        nncar_motor_out_t ml = nncar_motor_step(p, &motor_l, &cfg, &min, dt);

        min.target_mps = mix.wheels.right;
        min.actual_mps = prev_right;
        nncar_motor_out_t mr = nncar_motor_step(p, &motor_r, &cfg, &min, dt);

        max_u_err = fmaxf(max_u_err, fabsf(ml.u_cmd - (float)values[col.u_l]));
        max_u_err = fmaxf(max_u_err, fabsf(mr.u_cmd - (float)values[col.u_r]));

        prev_left = (float)values[col.wheel_v_l];
        prev_right = (float)values[col.wheel_v_r];
    }

    fclose(fp);

    const float mixer_tol = 1.0e-3f;
    const float motor_tol = 2.0e-2f;

    if (max_lambda_err > mixer_tol || max_wheel_err > mixer_tol ||
        max_u_err > motor_tol) {
        violations += 1;
    }

    printf("replay: rows=%d faults_skipped=%d\n", rows, skipped_fault_rows);
    printf("  max |lambda err|  = %.6g\n", (double)max_lambda_err);
    printf("  max |wheel err|   = %.6g\n", (double)max_wheel_err);
    printf("  max |u_pi err|    = %.6g\n", (double)max_u_err);

    if (violations != 0) {
        fprintf(stderr, "replay: FAILED (mixer tol %.1e, motor tol %.1e)\n",
                (double)mixer_tol, (double)motor_tol);
        return 1;
    }

    printf("replay: ok\n");
    return 0;
}
