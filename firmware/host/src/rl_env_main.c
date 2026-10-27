#include "nncar/gray.h"
#include "nncar/safety.h"
#include "nncar/sim.h"
#include "nncar/stack.h"
#include "nncar/velocity.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const nncar_profile_t *profile;
    nncar_sim_t sim;
    nncar_stack_t stack;
    nncar_stack_output_t last_output;
    nncar_velocity_t velocity_left;
    nncar_velocity_t velocity_right;
    float gray_value[NNCAR_MAX_SENSORS];
    uint8_t gray_exist[NNCAR_MAX_SENSORS];
    uint8_t gray_valid[NNCAR_MAX_SENSORS];
    float encoder_accum_left;
    float encoder_accum_right;
    float wheel_velocity_left;
    float wheel_velocity_right;
    float v_desired;
    float termination_lateral_m;
    float termination_heading_rad;
    float control_dt_s;
    uint64_t now_us;
    uint64_t last_edge_us;
    uint32_t horizon_steps;
    uint32_t step;
    uint32_t episode;
    int substeps;
    int have_edge;
    int edge_prev;
    int active;
    int done;
} env_t;

static int parse_options(int argc, char **argv, uint32_t *horizon_steps,
                         float *termination_lateral_m,
                         float *termination_heading_rad)
{
    *horizon_steps = 1000u;
    *termination_lateral_m = 0.12f;
    *termination_heading_rad = 1.3962634f; /* 80 degrees */

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--horizon-steps") == 0 && i + 1 < argc) {
            char *end = NULL;
            unsigned long value = strtoul(argv[++i], &end, 10);
            if (end == argv[i] || *end != '\0' || value == 0u || value > UINT32_MAX) {
                return -1;
            }
            *horizon_steps = (uint32_t)value;
        } else if (strcmp(argv[i], "--terminate-lateral-m") == 0 && i + 1 < argc) {
            char *end = NULL;
            float value = strtof(argv[++i], &end);
            if (end == argv[i] || *end != '\0' || !isfinite(value) || value <= 0.0f) {
                return -1;
            }
            *termination_lateral_m = value;
        } else if (strcmp(argv[i], "--terminate-heading-deg") == 0 && i + 1 < argc) {
            char *end = NULL;
            float value = strtof(argv[++i], &end);
            if (end == argv[i] || *end != '\0' || !isfinite(value) ||
                value <= 0.0f || value > 180.0f) {
                return -1;
            }
            *termination_heading_rad = value * NNCAR_PI / 180.0f;
        } else {
            return -1;
        }
    }
    return 0;
}

static int only_whitespace_remains(const char *line, int consumed)
{
    if (consumed <= 0) {
        return 0;
    }
    for (const char *p = line + consumed; *p != '\0'; ++p) {
        if (*p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
            return 0;
        }
    }
    return 1;
}

static int parse_reset(const char *line, env_t *env, uint32_t horizon_steps,
                       float termination_lateral_m, float termination_heading_rad)
{
    unsigned seed = 0u;
    unsigned missing_mask = 0u;
    float offset_m = 0.0f;
    float heading_deg = 0.0f;
    float amp_m = 0.0f;
    float wavelength_m = 1.0f;
    float line_width_m = 0.02f;
    float gray_noise = 0.0f;
    float gray_offset = 0.0f;
    float gray_gain = 1.0f;
    float gray_valid_prob = 1.0f;
    float v_desired = 0.9f;
    int consumed = 0;

    int fields = sscanf(line,
                        "RESET %u %f %f %f %f %f %f %f %f %f %u %f %n",
                        &seed, &offset_m, &heading_deg, &amp_m, &wavelength_m,
                        &line_width_m, &gray_noise, &gray_offset, &gray_gain,
                        &gray_valid_prob, &missing_mask, &v_desired, &consumed);
    if (fields != 12 || !only_whitespace_remains(line, consumed) ||
        missing_mask > 0xffffu || !isfinite(offset_m) ||
        !isfinite(heading_deg) || !isfinite(amp_m) || !isfinite(wavelength_m) ||
        !isfinite(line_width_m) || !isfinite(gray_noise) || !isfinite(gray_offset) ||
        !isfinite(gray_gain) || !isfinite(gray_valid_prob) || !isfinite(v_desired) ||
        wavelength_m <= 0.0f || line_width_m <= 0.0f || gray_noise < 0.0f ||
        gray_offset < -1.0f || gray_offset > 1.0f || gray_gain < 0.0f ||
        gray_gain > 2.0f || gray_valid_prob < 0.0f || gray_valid_prob > 1.0f ||
        v_desired <= 0.0f) {
        return -1;
    }

    const nncar_profile_t *p = env->profile;
    nncar_sim_init(&env->sim, p, 0.0f, 0.0f, 0.0f, p->rail_nominal_v);
    env->sim.track_amp_m = amp_m;
    env->sim.track_wavelength_m = wavelength_m;
    env->sim.line_width_m = line_width_m;
    env->sim.gray_noise_std = gray_noise;
    env->sim.gray_offset = gray_offset;
    env->sim.gray_valid_prob = gray_valid_prob;
    nncar_sim_seed(&env->sim, seed);
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        env->sim.gray_gain[i] = gray_gain;
        env->sim.gray_missing[i] = (uint8_t)((missing_mask >> i) & 1u);
    }

    float track_slope = 0.0f;
    if (amp_m != 0.0f) {
        track_slope = amp_m * (2.0f * NNCAR_PI / wavelength_m);
    }
    env->sim.y = nncar_sim_track_y(env->sim.x, &env->sim) + offset_m;
    env->sim.theta = atanf(track_slope) + heading_deg * NNCAR_PI / 180.0f;

    nncar_stack_init(&env->stack, p);
    memset(&env->last_output, 0, sizeof(env->last_output));
    nncar_velocity_init(&env->velocity_left, p);
    nncar_velocity_init(&env->velocity_right, p);
    memset(env->gray_value, 0, sizeof(env->gray_value));
    memset(env->gray_exist, 0, sizeof(env->gray_exist));
    memset(env->gray_valid, 0, sizeof(env->gray_valid));
    env->encoder_accum_left = 0.0f;
    env->encoder_accum_right = 0.0f;
    env->wheel_velocity_left = 0.0f;
    env->wheel_velocity_right = 0.0f;
    env->v_desired = v_desired;
    env->termination_lateral_m = termination_lateral_m;
    env->termination_heading_rad = termination_heading_rad;
    env->horizon_steps = horizon_steps;
    env->control_dt_s = p->line_dt_s;
    env->substeps = (int)lroundf(p->line_dt_s / p->motor_dt_s);
    if (env->substeps < 1 ||
        fabsf((float)env->substeps * p->motor_dt_s - p->line_dt_s) > 1.0e-6f) {
        return -1;
    }
    env->now_us = 0u;
    env->last_edge_us = 0u;
    env->have_edge = 0;
    env->edge_prev = 0;
    env->step = 0u;
    env->episode += 1u;
    env->active = 1;
    env->done = 0;
    nncar_sim_read_gray(&env->sim, p, env->gray_value, env->gray_exist, env->gray_valid);
    return 0;
}

static void emit_observation(const env_t *env)
{
    printf("\"observation\":{\"gray_value\":[");
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        printf("%s%.7g", i == 0 ? "" : ",", (double)env->gray_value[i]);
    }
    printf("],\"gray_exist\":[");
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        printf("%s%u", i == 0 ? "" : ",", (unsigned)env->gray_exist[i]);
    }
    printf("],\"gray_valid\":[");
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        printf("%s%u", i == 0 ? "" : ",", (unsigned)env->gray_valid[i]);
    }
    printf("],\"wheel_velocity_mps\":[%.7g,%.7g]}",
           (double)env->wheel_velocity_left, (double)env->wheel_velocity_right);
}

static float track_heading(const nncar_sim_t *sim)
{
    float slope = 0.0f;
    if (sim->track_amp_m != 0.0f) {
        slope = sim->track_amp_m * (2.0f * NNCAR_PI / sim->track_wavelength_m) *
                cosf(2.0f * NNCAR_PI * sim->x / sim->track_wavelength_m);
    }
    return atanf(slope);
}

static float angle_error(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static int step_environment(env_t *env, float action[2])
{
    const nncar_profile_t *p = env->profile;
    action[0] = nncar_clampf(action[0], -1.0f, 1.0f);
    action[1] = nncar_clampf(action[1], -1.0f, 1.0f);

    float acc_l = env->encoder_accum_left;
    float acc_r = env->encoder_accum_right;
    float v_l = env->wheel_velocity_left;
    float v_r = env->wheel_velocity_right;
    const float dt = p->motor_dt_s;
    const uint64_t dt_us = (uint64_t)(dt * 1.0e6f + 0.5f);
    const float cpm = nncar_counts_per_meter(p);
    nncar_stack_output_t out;
    memset(&out, 0, sizeof(out));

    for (int substep = 0; substep < env->substeps; ++substep) {
        bool encoder_activity = env->edge_prev ||
            (env->have_edge && (env->now_us - env->last_edge_us) < 20000u);

        nncar_stack_input_t in;
        memset(&in, 0, sizeof(in));
        in.v_desired = env->v_desired;
        in.v_rail = env->sim.v_rail;
        in.v_rail_valid = true;
        in.v_rail_exist = true;
        in.rail_age_s = 0.0f;
        in.now_us = env->now_us;
        in.actual_left_mps = v_l;
        in.actual_right_mps = v_r;
        in.actual_valid = true;
        in.encoder_activity = encoder_activity;
        in.comm_ok = true;

        out = nncar_stack_step_action(&env->stack, p, &in,
                                      env->gray_value, env->gray_exist,
                                      env->gray_valid, action, dt);
        nncar_sim_step(&env->sim, p, out.pwm, dt);

        acc_l += env->sim.w_left * p->wheel_radius_m * dt * cpm;
        acc_r += env->sim.w_right * p->wheel_radius_m * dt * cpm;
        int32_t dl = (int32_t)floorf(acc_l);
        int32_t dr = (int32_t)floorf(acc_r);
        acc_l -= (float)dl;
        acc_r -= (float)dr;
        v_l = nncar_velocity_step(&env->velocity_left, dl, dt);
        v_r = nncar_velocity_step(&env->velocity_right, dr, dt);

        env->edge_prev = (dl != 0) || (dr != 0);
        if (env->edge_prev) {
            env->last_edge_us = env->now_us;
            env->have_edge = 1;
        }
        env->now_us += dt_us;
    }

    env->encoder_accum_left = acc_l;
    env->encoder_accum_right = acc_r;
    env->wheel_velocity_left = v_l;
    env->wheel_velocity_right = v_r;
    env->last_output = out;
    env->step += 1u;

    nncar_sim_read_gray(&env->sim, p, env->gray_value, env->gray_exist, env->gray_valid);
    return 0;
}

static void emit_reset(env_t *env)
{
    printf("{\"event\":\"reset\",\"episode\":%u,", env->episode);
    emit_observation(env);
    printf(",\"info\":{\"x\":%.7g,\"y\":%.7g,\"theta\":%.7g,"
           "\"lateral_error_m\":%.7g,\"heading_error_rad\":%.7g,"
           "\"control_dt_s\":%.7g}}\n",
           (double)env->sim.x, (double)env->sim.y, (double)env->sim.theta,
           (double)(env->sim.y - nncar_sim_track_y(env->sim.x, &env->sim)),
           (double)angle_error(env->sim.theta - track_heading(&env->sim)),
           (double)env->control_dt_s);
    fflush(stdout);
}

static void emit_transition(env_t *env, const float action[2])
{
    float lateral = env->sim.y - nncar_sim_track_y(env->sim.x, &env->sim);
    float heading = angle_error(env->sim.theta - track_heading(&env->sim));
    float lateral_ratio = fabsf(lateral) / env->termination_lateral_m;
    float heading_ratio = fabsf(heading) / env->termination_heading_rad;
    float effort = action[0] * action[0] + action[1] * action[1];
    float reward = 1.0f - lateral_ratio - 0.25f * heading_ratio - 0.01f * effort;
    int terminated = lateral_ratio >= 1.0f || heading_ratio >= 1.0f;
    int truncated = !terminated && env->step >= env->horizon_steps;
    const char *reason = "";
    if (lateral_ratio >= 1.0f) {
        reason = "lateral_limit";
    } else if (heading_ratio >= 1.0f) {
        reason = "heading_limit";
    } else if (truncated) {
        reason = "time_limit";
    }
    if (terminated) {
        reward -= 1.0f;
    }
    env->done = terminated || truncated;

    nncar_line_estimate_t line;
    nncar_gray_process(env->profile, env->gray_value, env->gray_exist,
                       env->gray_valid, &line);

    printf("{\"event\":\"transition\",\"episode\":%u,", env->episode);
    emit_observation(env);
    printf(",\"reward\":%.7g,\"terminated\":%s,\"truncated\":%s,"
           "\"info\":{\"step\":%u,\"x\":%.7g,\"y\":%.7g,\"theta\":%.7g,"
           "\"lateral_error_m\":%.7g,\"heading_error_rad\":%.7g,"
           "\"action\":[%.7g,%.7g],\"line_confidence\":%.7g,"
           "\"line_centroid_m\":%.7g,\"line_found\":%d,\"valid_count\":%u,"
           "\"mode\":%d,\"policy_action_applied\":%s,"
           "\"v_target\":%.7g,\"omega_target\":%.7g,\"v_max_eff\":%.7g,"
           "\"lambda\":%.7g,\"kappa_target\":%.7g,"
           "\"wheel_target\":[%.7g,%.7g],\"u\":[%.7g,%.7g],"
           "\"pwm\":[%.7g,%.7g],\"safety_faults\":%u,"
           "\"safety_latched\":%d,\"safety_derate\":%.7g,"
           "\"safety_ramp\":%.7g,\"safety_stop\":%d,\"v_rail\":%.7g,"
           "\"termination_reason\":\"%s\"}}\n",
           reward, terminated ? "true" : "false", truncated ? "true" : "false",
           env->step, (double)env->sim.x, (double)env->sim.y,
           (double)env->sim.theta, (double)lateral, (double)heading,
           (double)action[0], (double)action[1], (double)line.confidence,
           (double)line.centroid_m, line.line_found ? 1 : 0,
           (unsigned)line.valid_count, (int)env->last_output.mode,
           env->last_output.mode == NNCAR_LINE_TRACKING ? "true" : "false",
           (double)env->last_output.twist.v, (double)env->last_output.twist.omega,
           (double)env->last_output.safety.v_max_eff,
           (double)env->last_output.mix.lambda,
           (double)env->last_output.mix.kappa_target,
           (double)env->last_output.mix.wheels.left,
           (double)env->last_output.mix.wheels.right,
           (double)env->last_output.u_left, (double)env->last_output.u_right,
           (double)env->last_output.pwm.left, (double)env->last_output.pwm.right,
           env->last_output.safety.active_faults,
           env->last_output.safety.latched_faults != 0u ? 1 : 0,
           (double)env->last_output.safety.derate,
           (double)env->last_output.safety.ramp,
           env->last_output.safety.stop ? 1 : 0,
           (double)env->sim.v_rail, reason);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    uint32_t horizon_steps;
    float termination_lateral_m;
    float termination_heading_rad;
    if (parse_options(argc, argv, &horizon_steps, &termination_lateral_m,
                      &termination_heading_rad) != 0) {
        fprintf(stderr,
                "usage: %s [--horizon-steps 1000] [--terminate-lateral-m 0.12] "
                "[--terminate-heading-deg 80]\n", argv[0]);
        return 2;
    }

    env_t env;
    memset(&env, 0, sizeof(env));
    env.profile = nncar_profile_default();
    char line[512];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (strncmp(line, "RESET ", 6) == 0) {
            if (parse_reset(line, &env, horizon_steps, termination_lateral_m,
                            termination_heading_rad) != 0) {
                puts("{\"error\":\"invalid reset command\"}");
                fflush(stdout);
                continue;
            }
            emit_reset(&env);
        } else if (strncmp(line, "STEP ", 5) == 0) {
            float action[2] = {0.0f, 0.0f};
            int consumed = 0;
            if (!env.active || env.done ||
                sscanf(line, "STEP %f %f %n", &action[0], &action[1], &consumed) != 2 ||
                !only_whitespace_remains(line, consumed) ||
                !isfinite(action[0]) || !isfinite(action[1])) {
                puts("{\"error\":\"invalid step command or episode is done\"}");
                fflush(stdout);
                continue;
            }
            (void)step_environment(&env, action);
            emit_transition(&env, action);
        } else if (strcmp(line, "CLOSE\n") == 0 || strcmp(line, "CLOSE\r\n") == 0 ||
                   strcmp(line, "CLOSE") == 0) {
            break;
        } else {
            puts("{\"error\":\"unknown command\"}");
            fflush(stdout);
        }
    }
    return 0;
}
