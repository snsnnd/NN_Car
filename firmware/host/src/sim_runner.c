#include "nncar/sim_runner.h"

#include "nncar/stack.h"
#include "nncar/velocity.h"

#include <math.h>
#include <string.h>

nncar_sim_run_result_t nncar_sim_run(const nncar_profile_t *p,
                                     nncar_sim_t *sim,
                                     nncar_sim_run_cfg_t cfg,
                                     nncar_row_fn on_row,
                                     void *ctx)
{
    nncar_sim_run_result_t res;
    memset(&res, 0, sizeof(res));

    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);

    nncar_stack_t stack;
    nncar_stack_init(&stack, p);

    nncar_velocity_t vel_l;
    nncar_velocity_t vel_r;
    nncar_velocity_init(&vel_l, p);
    nncar_velocity_init(&vel_r, p);

    const float cpm = nncar_counts_per_meter(p);
    const int ticks = (int)(cfg.duration_s / dt + 0.5f);

    float acc_l = 0.0f;
    float acc_r = 0.0f;
    float v_l = 0.0f;
    float v_r = 0.0f;
    uint64_t now_us = 0;
    uint64_t last_edge_us = 0;
    bool have_edge = false;
    bool edge_prev = false;

    for (int k = 0; k < ticks; ++k) {
        float gray_value[NNCAR_MAX_SENSORS];
        uint8_t gray_exist[NNCAR_MAX_SENSORS];
        uint8_t gray_valid[NNCAR_MAX_SENSORS];

        nncar_sim_read_gray(sim, p, gray_value, gray_exist, gray_valid);

        bool encoder_activity = edge_prev ||
                               (have_edge && (now_us - last_edge_us) < 20000u);

        nncar_stack_input_t in;
        memset(&in, 0, sizeof(in));
        in.v_desired = cfg.v_desired;
        in.v_rail = sim->v_rail;
        in.v_rail_valid = true;
        in.v_rail_exist = true;
        in.rail_age_s = 0.0f;
        in.now_us = now_us;
        in.actual_left_mps = v_l;
        in.actual_right_mps = v_r;
        in.actual_valid = true;
        in.encoder_activity = encoder_activity;
        in.comm_ok = true;
        in.estop = false;
        in.nn_fault = false;
        in.temp_valid = false;

        nncar_stack_output_t out = nncar_stack_step(&stack, p, &in, gray_value,
                                                    gray_exist, gray_valid, dt);

        nncar_sim_step(sim, p, out.pwm, dt);

        acc_l += sim->w_left * p->wheel_radius_m * dt * cpm;
        acc_r += sim->w_right * p->wheel_radius_m * dt * cpm;

        int32_t dl = (int32_t)floorf(acc_l);
        int32_t dr = (int32_t)floorf(acc_r);
        acc_l -= (float)dl;
        acc_r -= (float)dr;

        v_l = nncar_velocity_step(&vel_l, dl, dt);
        v_r = nncar_velocity_step(&vel_r, dr, dt);

        edge_prev = (dl != 0) || (dr != 0);
        if (edge_prev) {
            last_edge_us = now_us;
            have_edge = true;
        }

        float lat = sim->y - nncar_sim_track_y(sim->x, sim);
        if (fabsf(lat) > res.max_lat_m) {
            res.max_lat_m = fabsf(lat);
        }
        res.final_lat_m = fabsf(lat);
        if (fabsf(out.twist.omega) > res.max_omega) {
            res.max_omega = fabsf(out.twist.omega);
        }
        res.fault_union |= out.safety.active_faults;

        if (on_row != NULL) {
            nncar_log_row_t row;
            memset(&row, 0, sizeof(row));
            row.t_us = now_us;
            row.mode = (int)out.mode;
            row.line_conf = out.line.confidence;
            row.line_centroid_m = out.line.centroid_m;
            row.line_found = out.line.line_found ? 1 : 0;
            row.valid_count = (int)out.line.valid_count;
            row.v_target = out.twist.v;
            row.omega_target = out.twist.omega;
            row.v_max_eff = out.safety.v_max_eff;
            row.lambda = out.mix.lambda;
            row.kappa_target = out.mix.kappa_target;
            row.wheel_v_l_target = out.mix.wheels.left;
            row.wheel_v_r_target = out.mix.wheels.right;
            row.wheel_v_l = v_l;
            row.wheel_v_r = v_r;
            row.u_l = out.u_left;
            row.u_r = out.u_right;
            row.pwm_l = out.pwm.left;
            row.pwm_r = out.pwm.right;
            row.v_rail = sim->v_rail;
            row.v_rail_valid = 1;
            row.safety_faults = out.safety.active_faults;
            row.safety_latched = out.safety.latched_faults != 0u ? 1 : 0;
            row.safety_derate = out.safety.derate;
            row.safety_ramp = out.safety.ramp;
            row.safety_stop = out.safety.stop ? 1 : 0;
            row.x = sim->x;
            row.y = sim->y;
            row.theta = sim->theta;
            row.recovery_ms = out.recovery_ms;
            on_row(ctx, &row);
        }

        now_us += dt_us;
        res.ticks += 1u;
    }

    return res;
}
