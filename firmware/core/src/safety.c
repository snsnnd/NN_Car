#include "nncar/safety.h"

#include <string.h>

void nncar_safety_reset(nncar_safety_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->ramp = 1.0f;
}

void nncar_safety_clear(nncar_safety_state_t *st, uint32_t faults)
{
    st->latched &= ~faults;
}

nncar_safety_output_t nncar_safety_step(const nncar_profile_t *p,
                                        nncar_safety_state_t *st,
                                        const nncar_safety_input_t *in,
                                        float dt_s)
{
    nncar_safety_output_t out;
    uint32_t active = 0u;
    uint32_t immediate = 0u;

    memset(&out, 0, sizeof(out));
    out.v_max_eff = in->v_max_eff;

    float speed_left = in->actual_left.valid ? in->actual_left.value : 0.0f;
    float speed_right = in->actual_right.valid ? in->actual_right.value : 0.0f;
    float speed_peak = fmaxf(fabsf(speed_left), fabsf(speed_right));

    if (in->estop) {
        active |= NNCAR_FAULT_ESTOP;
    }

    if (in->twist_age_s > p->twist_ttl_s) {
        active |= NNCAR_FAULT_STALE_TWIST;
    }
    if (in->wheel_target_age_s > p->cmd_ttl_s) {
        active |= NNCAR_FAULT_STALE_WHEEL;
    }
    if (in->motor_cmd_age_s > p->cmd_ttl_s) {
        active |= NNCAR_FAULT_STALE_MOTOR_CMD;
    }

    bool speed_ok = nncar_sensor_usable(&in->actual_left, in->now_us, p->cmd_ttl_s) &&
                    nncar_sensor_usable(&in->actual_right, in->now_us, p->cmd_ttl_s);
    if (!in->actual_left.valid || !in->actual_right.valid || !speed_ok) {
        st->speed_invalid_s += dt_s;
        if (st->speed_invalid_s >= p->cmd_ttl_s) {
            active |= NNCAR_FAULT_SPEED_INVALID;
        }
    } else {
        st->speed_invalid_s = 0.0f;
    }

    if (in->v_max_eff > 1.0e-3f && speed_peak > p->overspeed_factor * in->v_max_eff) {
        active |= NNCAR_FAULT_OVERSPEED;
    }

    if (in->v_max_eff > 1.0e-3f && speed_peak > p->hard_overspeed_factor * in->v_max_eff) {
        st->hard_overspeed_s += dt_s;
        if (st->hard_overspeed_s >= p->hard_overspeed_hold_s) {
            active |= NNCAR_FAULT_HARD_OVERSPEED;
        }
    } else {
        st->hard_overspeed_s = 0.0f;
    }

    bool rail_ok = in->v_rail.exist && in->v_rail.valid && nncar_finite(in->v_rail.value) &&
                   (in->rail_age_s <= p->rail_ttl_s);

    if (!rail_ok) {
        active |= NNCAR_FAULT_UNDERVOLT;
    } else {
        if (in->v_rail.value < p->rail_stop_v) {
            st->undervolt_s += dt_s;
            if (st->undervolt_s >= 0.02f) {
                active |= NNCAR_FAULT_UNDERVOLT;
            }
        } else {
            st->undervolt_s = 0.0f;
        }

        if (in->v_rail.value < p->rail_derate_v) {
            st->rail_low_s += dt_s;
            if (st->rail_low_s >= 0.02f) {
                active |= NNCAR_FAULT_RAIL_LOW;
            }
        } else {
            st->rail_low_s = 0.0f;
        }

        if (in->v_rail.value > p->rail_ovp_v) {
            active |= NNCAR_FAULT_OVERVOLT;
        }
    }

    if (in->temp_valid && nncar_finite(in->max_temp_c)) {
        if (in->max_temp_c > p->temp_stop_c) {
            active |= NNCAR_FAULT_TEMP_STOP;
        } else if (in->max_temp_c > p->temp_warn_c) {
            active |= NNCAR_FAULT_TEMP_WARN;
        }
    }

    if (in->nn_fault) {
        active |= NNCAR_FAULT_NN;
    }

    if (in->applied_pwm_abs > p->encoder_min_pwm && !in->encoder_activity) {
        st->no_edge_s += dt_s;
        if (st->no_edge_s >= p->encoder_no_edge_s) {
            active |= NNCAR_FAULT_ENCODER_NO_EDGE;
        }
    } else {
        st->no_edge_s = 0.0f;
    }

    if (!in->comm_ok) {
        active |= NNCAR_FAULT_COMM;
    }

    if (in->line_lost) {
        active |= NNCAR_FAULT_LINE_LOST;
    }

    if (in->curvature_fault) {
        active |= NNCAR_FAULT_CURVATURE;
    }

    immediate |= active & NNCAR_FAULT_IMMEDIATE_MASK;

    /* Only stop-class faults latch; derate faults recover automatically. */
    st->latched |= active & NNCAR_FAULT_STOP_MASK;
    out.latched_faults = st->latched;
    out.active_faults = active;

    bool stop = ((active & NNCAR_FAULT_STOP_MASK) != 0u) || (st->latched != 0u);

    float derate = 1.0f;
    if (active & NNCAR_FAULT_RAIL_LOW) {
        derate *= p->derate_rail_low;
    }
    if (active & NNCAR_FAULT_TEMP_WARN) {
        derate *= p->derate_temp_warn;
    }
    if (active & NNCAR_FAULT_CURVATURE) {
        derate *= 0.8f;
    }
    out.derate = nncar_clampf(derate, 0.0f, 1.0f);

    float ramp_target = stop ? 0.0f : 1.0f;
    float step = dt_s / fmaxf(p->ramp_down_s, 1.0e-3f);
    if (st->ramp < ramp_target) {
        st->ramp = fminf(ramp_target, st->ramp + step);
    } else if (st->ramp > ramp_target) {
        st->ramp = fmaxf(ramp_target, st->ramp - step);
    }
    out.ramp = st->ramp;

    float limit = in->v_max_eff * out.derate;
    out.wheel_out.left = nncar_clampf(in->wheel_target.left * st->ramp, -limit, limit);
    out.wheel_out.right = nncar_clampf(in->wheel_target.right * st->ramp, -limit, limit);

    out.stop = (st->ramp <= 0.0f) && stop;
    out.force_zero_pwm = (immediate != 0u);

    if (out.force_zero_pwm) {
        out.wheel_out.left = 0.0f;
        out.wheel_out.right = 0.0f;
    }

    return out;
}
