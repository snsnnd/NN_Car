#include "nncar/sim_runner.h"

#include <math.h>
#include <string.h>

void nncar_sim_session_init(nncar_sim_session_t *session,
                            const nncar_profile_t *p, nncar_sim_t *sim,
                            nncar_sim_run_cfg_t cfg)
{
    memset(session, 0, sizeof(*session));
    session->profile = p;
    session->sim = sim;
    session->cfg = cfg;
    session->total_ticks = (int)(cfg.duration_s / p->motor_dt_s + 0.5f);
    nncar_stack_init(&session->stack, p);
    nncar_velocity_init(&session->velocity_left, p);
    nncar_velocity_init(&session->velocity_right, p);
}

bool nncar_sim_session_step(nncar_sim_session_t *session, nncar_log_row_t *row)
{
    if (session->tick >= session->total_ticks) {
        return false;
    }

    const nncar_profile_t *p = session->profile;
    nncar_sim_t *sim = session->sim;
    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);
    const float cpm = nncar_counts_per_meter(p);

    float gray_value[NNCAR_MAX_SENSORS];
    uint8_t gray_exist[NNCAR_MAX_SENSORS];
    uint8_t gray_valid[NNCAR_MAX_SENSORS];
    nncar_sim_read_gray(sim, p, gray_value, gray_exist, gray_valid);

    bool encoder_activity = session->edge_prev ||
        (session->have_edge && (session->now_us - session->last_edge_us) < 20000u);

    nncar_stack_input_t in;
    memset(&in, 0, sizeof(in));
    in.v_desired = session->cfg.v_desired;
    in.v_rail = sim->v_rail;
    in.v_rail_valid = true;
    in.v_rail_exist = true;
    in.rail_age_s = 0.0f;
    in.now_us = session->now_us;
    in.actual_left_mps = session->wheel_velocity_left;
    in.actual_right_mps = session->wheel_velocity_right;
    in.actual_valid = true;
    in.encoder_activity = encoder_activity;
    in.comm_ok = true;
    in.estop = false;
    in.nn_fault = false;
    in.temp_valid = false;

    nncar_stack_output_t out = nncar_stack_step(&session->stack, p, &in,
                                                 gray_value, gray_exist,
                                                 gray_valid, dt);
    nncar_sim_step(sim, p, out.pwm, dt);

    session->encoder_accum_left += sim->w_left * p->wheel_radius_m * dt * cpm;
    session->encoder_accum_right += sim->w_right * p->wheel_radius_m * dt * cpm;
    int32_t dl = (int32_t)floorf(session->encoder_accum_left);
    int32_t dr = (int32_t)floorf(session->encoder_accum_right);
    session->encoder_accum_left -= (float)dl;
    session->encoder_accum_right -= (float)dr;
    session->wheel_velocity_left = nncar_velocity_step(&session->velocity_left, dl, dt);
    session->wheel_velocity_right = nncar_velocity_step(&session->velocity_right, dr, dt);

    session->edge_prev = (dl != 0) || (dr != 0);
    if (session->edge_prev) {
        session->last_edge_us = session->now_us;
        session->have_edge = true;
    }

    float lateral = sim->y - nncar_sim_track_y(sim->x, sim);
    if (fabsf(lateral) > session->result.max_lat_m) {
        session->result.max_lat_m = fabsf(lateral);
    }
    session->result.final_lat_m = fabsf(lateral);
    if (fabsf(out.twist.omega) > session->result.max_omega) {
        session->result.max_omega = fabsf(out.twist.omega);
    }
    session->result.fault_union |= out.safety.active_faults;

    if (row != NULL) {
        memset(row, 0, sizeof(*row));
        row->t_us = session->now_us;
        row->mode = (int)out.mode;
        row->line_conf = out.line.confidence;
        row->line_centroid_m = out.line.centroid_m;
        row->line_found = out.line.line_found ? 1 : 0;
        row->valid_count = (int)out.line.valid_count;
        row->v_target = out.twist.v;
        row->omega_target = out.twist.omega;
        row->v_max_eff = out.safety.v_max_eff;
        row->lambda = out.mix.lambda;
        row->kappa_target = out.mix.kappa_target;
        row->wheel_v_l_target = out.mix.wheels.left;
        row->wheel_v_r_target = out.mix.wheels.right;
        row->wheel_v_l = session->wheel_velocity_left;
        row->wheel_v_r = session->wheel_velocity_right;
        row->u_l = out.u_left;
        row->u_r = out.u_right;
        row->pwm_l = out.pwm.left;
        row->pwm_r = out.pwm.right;
        row->v_rail = sim->v_rail;
        row->v_rail_valid = 1;
        row->safety_faults = out.safety.active_faults;
        row->safety_latched = out.safety.latched_faults != 0u ? 1 : 0;
        row->safety_derate = out.safety.derate;
        row->safety_ramp = out.safety.ramp;
        row->safety_stop = out.safety.stop ? 1 : 0;
        row->x = sim->x;
        row->y = sim->y;
        row->theta = sim->theta;
        row->recovery_ms = out.recovery_ms;
        for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
            row->gray_value[i] = gray_value[i];
            row->gray_exist[i] = gray_exist[i];
            row->gray_valid[i] = gray_valid[i];
        }
    }

    session->now_us += dt_us;
    session->tick += 1;
    session->result.ticks += 1u;
    return true;
}

nncar_sim_run_result_t nncar_sim_run(const nncar_profile_t *p,
                                     nncar_sim_t *sim,
                                     nncar_sim_run_cfg_t cfg,
                                     nncar_row_fn on_row,
                                     void *ctx)
{
    nncar_sim_session_t session;
    nncar_sim_session_init(&session, p, sim, cfg);

    nncar_log_row_t row;
    while (nncar_sim_session_step(&session, &row)) {
        if (on_row != NULL) {
            on_row(ctx, &row);
        }
    }
    return session.result;
}
