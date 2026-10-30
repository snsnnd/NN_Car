#include "nncar/stack.h"

#include <string.h>

void nncar_stack_init(nncar_stack_t *st, const nncar_profile_t *p)
{
    memset(st, 0, sizeof(*st));
    nncar_line_reset(&st->line);
    nncar_line_set_ctrl(&st->line, NNCAR_LINE_CTRL_PID, NULL);
    nncar_motor_reset(&st->motor_l);
    nncar_motor_reset(&st->motor_r);
    nncar_safety_reset(&st->safety);
    st->motor_cfg.mode = NNCAR_MOTOR_MODE_PI_FF;
    st->motor_cfg.nn = NULL;
    st->line_accum_s = p->line_dt_s;
}

void nncar_stack_set_motor_mode(nncar_stack_t *st, nncar_motor_mode_t mode,
                                const nncar_motor_nn_t *nn)
{
    st->motor_cfg.mode = mode;
    st->motor_cfg.nn = nn;
    nncar_motor_reset(&st->motor_l);
    nncar_motor_reset(&st->motor_r);
}

void nncar_stack_set_line_ctrl(nncar_stack_t *st, nncar_line_ctrl_t ctrl,
                               const nncar_line_nn_t *nn)
{
    nncar_line_set_ctrl(&st->line, ctrl, nn);
}

void nncar_stack_set_line_r4_ctrl(nncar_stack_t *st,
                                  const nncar_line_r4_nn_t *nn)
{
    nncar_line_set_r4_ctrl(&st->line, nn);
}

static nncar_stack_output_t stack_step(nncar_stack_t *st,
                                       const nncar_profile_t *p,
                                       const nncar_stack_input_t *in,
                                       const float *gray_value,
                                       const uint8_t gray_exist[NNCAR_MAX_SENSORS],
                                       const uint8_t gray_valid[NNCAR_MAX_SENSORS],
                                       const float *action,
                                       float dt_s)
{
    nncar_stack_output_t out;

    memset(&out, 0, sizeof(out));

    float v_max_eff = nncar_v_max_eff(p, in->v_rail_valid ? in->v_rail : p->rail_nominal_v);

    st->line_accum_s += dt_s;
    st->twist_age_s += dt_s;

    if (st->line_accum_s + 1.0e-9f >= p->line_dt_s) {
        float line_dt = st->line_accum_s;

        nncar_gray_process(p, gray_value, gray_exist, gray_valid, &st->estimate);

        if (action == NULL) {
            const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
            float v_current = 0.5f * (in->actual_left_mps + in->actual_right_mps);
            st->twist = nncar_line_step_with_speed(
                p, &st->line, &frame, &st->estimate, in->v_desired, v_current,
                v_max_eff, (float)in->now_us * 1.0e-6f, line_dt);
        } else {
            const nncar_gray_frame_t frame = {gray_value, gray_exist, gray_valid};
            float v_current = 0.5f * (in->actual_left_mps + in->actual_right_mps);
            nncar_twist_t recovery = nncar_line_step_with_speed(
                p, &st->line, &frame, &st->estimate, in->v_desired, v_current,
                v_max_eff, (float)in->now_us * 1.0e-6f, line_dt);
            if (st->line.mode == NNCAR_LINE_RECOVERY) {
                st->twist = recovery;
            } else {
                float speed_scale = nncar_clampf(0.5f * (action[0] + 1.0f), 0.0f, 1.0f);
                float curvature = nncar_clampf(action[1], -1.0f, 1.0f);
                st->twist = nncar_twist_from_kappa(p, speed_scale, curvature,
                                                   in->v_desired, v_max_eff);
            }
        }
        st->mix = nncar_mixer(p, st->twist, v_max_eff);
        st->line_accum_s = 0.0f;
        st->twist_age_s = 0.0f;
    }

    out.line_lost = (st->line.mode == NNCAR_LINE_RECOVERY) && (st->twist.v <= 1.0e-4f);

    if (st->mix.lambda < 0.999f) {
        st->lambda_low_s += dt_s;
    } else {
        st->lambda_low_s = 0.0f;
    }

    nncar_safety_input_t sin;
    memset(&sin, 0, sizeof(sin));

    sin.v_max_eff = v_max_eff;
    sin.wheel_target = st->mix.wheels;
    sin.twist_age_s = st->twist_age_s;
    sin.wheel_target_age_s = st->twist_age_s;
    sin.motor_cmd_age_s = 0.0f;

    sin.actual_left.value = in->actual_left_mps;
    sin.actual_left.exist = true;
    sin.actual_left.valid = in->actual_valid && nncar_finite(in->actual_left_mps);
    sin.actual_left.timestamp_us = in->now_us;

    sin.actual_right.value = in->actual_right_mps;
    sin.actual_right.exist = true;
    sin.actual_right.valid = in->actual_valid && nncar_finite(in->actual_right_mps);
    sin.actual_right.timestamp_us = in->now_us;

    sin.v_rail.value = in->v_rail;
    sin.v_rail.exist = in->v_rail_exist;
    sin.v_rail.valid = in->v_rail_valid && nncar_finite(in->v_rail);
    sin.v_rail.timestamp_us = (in->now_us > 0u && in->rail_age_s > 0.0f)
                                  ? in->now_us - (uint64_t)(in->rail_age_s * 1.0e6f)
                                  : in->now_us;
    sin.rail_age_s = in->rail_age_s;
    sin.now_us = in->now_us;

    sin.max_temp_c = in->max_temp_c;
    sin.temp_valid = in->temp_valid;
    sin.nn_fault = in->nn_fault;
    sin.encoder_activity = in->encoder_activity;
    sin.comm_ok = in->comm_ok;
    sin.estop = in->estop;
    sin.line_lost = out.line_lost;
    sin.curvature_fault = (st->lambda_low_s >= p->curvature_fault_s);
    sin.applied_pwm_abs = fmaxf(fabsf(st->motor_l.u), fabsf(st->motor_r.u));

    out.safety = nncar_safety_step(p, &st->safety, &sin, dt_s);

    bool enable = sin.actual_left.valid && sin.actual_right.valid && !out.safety.force_zero_pwm;

    const float rail_ratio =
        in->v_rail_valid ? in->v_rail / p->rail_nominal_v : 1.0f;

    nncar_motor_input_t min;
    memset(&min, 0, sizeof(min));
    min.v_max_eff = v_max_eff;
    min.rail_ratio = rail_ratio;
    min.rail_exist = in->v_rail_exist;
    min.rail_valid = in->v_rail_valid;
    min.enable = enable;

    min.target_mps = out.safety.wheel_out.left;
    min.actual_mps = in->actual_left_mps;
    nncar_motor_out_t ml = nncar_motor_step(p, &st->motor_l, &st->motor_cfg, &min, dt_s);

    min.target_mps = out.safety.wheel_out.right;
    min.actual_mps = in->actual_right_mps;
    nncar_motor_out_t mr = nncar_motor_step(p, &st->motor_r, &st->motor_cfg, &min, dt_s);

    out.u_left = ml.u_cmd;
    out.u_right = mr.u_cmd;
    out.pwm.left = ml.pwm;
    out.pwm.right = mr.pwm;

    out.twist = st->twist;
    out.line = st->estimate;
    out.mix = st->mix;
    out.mode = st->line.mode;
    out.recovery_ms = st->line.recovery_time_s * 1000.0f;

    return out;
}

nncar_stack_output_t nncar_stack_step(nncar_stack_t *st,
                                      const nncar_profile_t *p,
                                      const nncar_stack_input_t *in,
                                      const float *gray_value,
                                      const uint8_t gray_exist[NNCAR_MAX_SENSORS],
                                      const uint8_t gray_valid[NNCAR_MAX_SENSORS],
                                      float dt_s)
{
    return stack_step(st, p, in, gray_value, gray_exist, gray_valid, NULL, dt_s);
}

nncar_stack_output_t nncar_stack_step_action(nncar_stack_t *st,
                                             const nncar_profile_t *p,
                                             const nncar_stack_input_t *in,
                                             const float *gray_value,
                                             const uint8_t gray_exist[NNCAR_MAX_SENSORS],
                                             const uint8_t gray_valid[NNCAR_MAX_SENSORS],
                                             const float action[2],
                                             float dt_s)
{
    return stack_step(st, p, in, gray_value, gray_exist, gray_valid, action, dt_s);
}
