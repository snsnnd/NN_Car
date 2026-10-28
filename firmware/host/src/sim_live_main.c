#define _POSIX_C_SOURCE 200809L

#include "nncar/sim_runner.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    int port;
    float duration_s;
    float v_desired;
    float offset_m;
    float heading_deg;
    float amp_m;
    float wavelength_m;
    float rail_v;
    uint32_t seed;
} options_t;

typedef struct {
    const nncar_profile_t *profile;
    options_t options;
    nncar_sim_t sim;
    nncar_sim_session_t session;
} live_env_t;

static int parse_float(const char *text, float *value)
{
    char *end = NULL;
    *value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(*value) ? 0 : -1;
}

static int parse_u32(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 0);
    if (end == text || *end != '\0' || parsed > UINT32_MAX) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static int parse_options(int argc, char **argv, options_t *options)
{
    options->port = 37621;
    options->duration_s = 60.0f;
    options->v_desired = 0.8f;
    options->offset_m = 0.0f;
    options->heading_deg = 0.0f;
    options->amp_m = 0.08f;
    options->wavelength_m = 3.0f;
    options->rail_v = 12.0f;
    options->seed = 23u;

    for (int i = 1; i < argc; ++i) {
        if (i + 1 >= argc) {
            return -1;
        }
        const char *arg = argv[i++];
        const char *value = argv[i];
        if (strcmp(arg, "--port") == 0) {
            uint32_t port = 0u;
            if (parse_u32(value, &port) != 0 || port == 0u || port > 65535u) {
                return -1;
            }
            options->port = (int)port;
        } else if (strcmp(arg, "--seed") == 0) {
            if (parse_u32(value, &options->seed) != 0) return -1;
        } else if (strcmp(arg, "--duration") == 0) {
            if (parse_float(value, &options->duration_s) != 0) return -1;
        } else if (strcmp(arg, "--v") == 0) {
            if (parse_float(value, &options->v_desired) != 0) return -1;
        } else if (strcmp(arg, "--offset") == 0) {
            if (parse_float(value, &options->offset_m) != 0) return -1;
        } else if (strcmp(arg, "--heading-deg") == 0) {
            if (parse_float(value, &options->heading_deg) != 0) return -1;
        } else if (strcmp(arg, "--amp") == 0) {
            if (parse_float(value, &options->amp_m) != 0) return -1;
        } else if (strcmp(arg, "--wavelength") == 0) {
            if (parse_float(value, &options->wavelength_m) != 0) return -1;
        } else if (strcmp(arg, "--rail") == 0) {
            if (parse_float(value, &options->rail_v) != 0) return -1;
        } else {
            return -1;
        }
    }

    return options->duration_s > 0.0f && options->v_desired >= 0.0f &&
           options->wavelength_m > 0.0f && options->rail_v > 0.0f ? 0 : -1;
}

static void reset_session(live_env_t *env)
{
    const options_t *o = &env->options;
    nncar_sim_init(&env->sim, env->profile, 0.0f, o->offset_m,
                   o->heading_deg * NNCAR_PI / 180.0f, o->rail_v);
    env->sim.track_amp_m = o->amp_m;
    env->sim.track_wavelength_m = o->wavelength_m;
    nncar_sim_seed(&env->sim, o->seed);

    nncar_sim_run_cfg_t cfg = {o->duration_s, o->v_desired};
    nncar_sim_session_init(&env->session, env->profile, &env->sim, cfg);
}

static int send_all(int fd, const char *data, size_t size)
{
    size_t sent = 0u;
    while (sent < size) {
        ssize_t result = send(fd, data + sent, size - sent, MSG_NOSIGNAL);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result <= 0) {
            return -1;
        }
        sent += (size_t)result;
    }
    return 0;
}

static int send_line(int fd, const char *line)
{
    return send_all(fd, line, strlen(line));
}

static int read_line(int fd, char *line, size_t capacity)
{
    size_t used = 0u;
    while (used + 1u < capacity) {
        char ch;
        ssize_t result = recv(fd, &ch, 1u, 0);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result <= 0) {
            return (result == 0) ? 0 : -1;
        }
        if (ch == '\n') {
            line[used] = '\0';
            if (used > 0u && line[used - 1u] == '\r') {
                line[used - 1u] = '\0';
            }
            return 1;
        }
        line[used++] = ch;
    }
    return -1;
}

static int appendf(char *buffer, size_t capacity, size_t *used,
                   const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = vsnprintf(buffer + *used, capacity - *used, format, args);
    va_end(args);
    if (result < 0 || (size_t)result >= capacity - *used) {
        return -1;
    }
    *used += (size_t)result;
    return 0;
}

static float angle_error(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static int send_state(int fd, const live_env_t *env, const nncar_log_row_t *row)
{
    const nncar_sim_t *sim = &env->sim;
    const nncar_profile_t *p = env->profile;
    float slope = sim->track_amp_m * (2.0f * NNCAR_PI / sim->track_wavelength_m) *
        cosf(2.0f * NNCAR_PI * sim->x / sim->track_wavelength_m);
    float heading_error = angle_error(sim->theta - atanf(slope));
    float lateral_error = sim->y - nncar_sim_track_y(sim->x, sim);
    bool done = env->session.tick >= env->session.total_ticks;

    char message[4096];
    size_t used = 0u;
    if (appendf(message, sizeof(message), &used,
                "{\"event\":\"state\",\"t_us\":%llu,\"done\":%s,"
                "\"x\":%.7g,\"y\":%.7g,\"theta\":%.7g,"
                "\"lateral_error_m\":%.7g,\"heading_error_rad\":%.7g,"
                "\"mode\":%d,\"line_conf\":%.7g,\"line_found\":%d,"
                "\"valid_count\":%d,\"line_centroid_m\":%.7g,"
                "\"v_target\":%.7g,\"omega_target\":%.7g,\"kappa_target\":%.7g,"
                "\"v_max_eff\":%.7g,\"lambda\":%.7g,"
                "\"wheel_v_l_target\":%.7g,\"wheel_v_r_target\":%.7g,"
                "\"wheel_v_l\":%.7g,\"wheel_v_r\":%.7g,"
                "\"u_l\":%.7g,\"u_r\":%.7g,"
                "\"pwm_l\":%.7g,\"pwm_r\":%.7g,\"v_rail\":%.7g,"
                "\"v_rail_valid\":1,\"safety_faults\":%u,"
                "\"safety_latched\":%d,\"safety_derate\":%.7g,"
                "\"safety_ramp\":%.7g,\"safety_stop\":%d,"
                "\"gray_value\":[",
                (unsigned long long)row->t_us, done ? "true" : "false",
                (double)row->x, (double)row->y, (double)row->theta,
                (double)lateral_error, (double)heading_error, row->mode,
                (double)row->line_conf, row->line_found, row->valid_count,
                (double)row->line_centroid_m, (double)row->v_target,
                (double)row->omega_target, (double)row->kappa_target,
                (double)row->v_max_eff, (double)row->lambda,
                (double)row->wheel_v_l_target, (double)row->wheel_v_r_target,
                (double)row->wheel_v_l, (double)row->wheel_v_r,
                (double)row->u_l, (double)row->u_r, (double)row->pwm_l,
                (double)row->pwm_r, (double)row->v_rail, row->safety_faults,
                row->safety_latched, (double)row->safety_derate,
                (double)row->safety_ramp, row->safety_stop) != 0) {
        return -1;
    }

    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        if (appendf(message, sizeof(message), &used, "%s%.7g",
                    i == 0 ? "" : ",", (double)row->gray_value[i]) != 0) {
            return -1;
        }
    }
    if (appendf(message, sizeof(message), &used, "],\"gray_exist\":[") != 0) {
        return -1;
    }
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        if (appendf(message, sizeof(message), &used, "%s%u",
                    i == 0 ? "" : ",", (unsigned)row->gray_exist[i]) != 0) {
            return -1;
        }
    }
    if (appendf(message, sizeof(message), &used, "],\"gray_valid\":[") != 0) {
        return -1;
    }
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        if (appendf(message, sizeof(message), &used, "%s%u",
                    i == 0 ? "" : ",", (unsigned)row->gray_valid[i]) != 0) {
            return -1;
        }
    }
    if (appendf(message, sizeof(message), &used,
                "],\"sensor_count\":%d}\n", p->sensor_count) != 0) {
        return -1;
    }
    return send_all(fd, message, used);
}

static int send_ready(int fd, const live_env_t *env)
{
    char message[512];
    const options_t *o = &env->options;
    int length = snprintf(message, sizeof(message),
        "{\"event\":\"ready\",\"protocol\":1,"
        "\"profile_verified\":%s,\"line_dt_s\":%.7g,"
        "\"motor_dt_s\":%.7g,\"sensor_count\":%d,"
        "\"scenario\":{\"duration_s\":%.7g,\"v_desired_mps\":%.7g,"
        "\"offset_m\":%.7g,\"heading_deg\":%.7g,\"amp_m\":%.7g,"
        "\"wavelength_m\":%.7g,\"rail_v\":%.7g,\"seed\":%u}}\n",
        nncar_profile_is_verified() ? "true" : "false",
        (double)env->profile->line_dt_s, (double)env->profile->motor_dt_s,
        env->profile->sensor_count, (double)o->duration_s,
        (double)o->v_desired, (double)o->offset_m, (double)o->heading_deg,
        (double)o->amp_m, (double)o->wavelength_m, (double)o->rail_v,
        (unsigned)o->seed);
    if (length < 0 || (size_t)length >= sizeof(message)) {
        return -1;
    }
    return send_all(fd, message, (size_t)length);
}

static int reset_from_command(live_env_t *env, const char *line)
{
    if (strcmp(line, "RESET") == 0) {
        reset_session(env);
        return 0;
    }

    options_t next = env->options;
    int consumed = 0;
    int fields = sscanf(line, "RESET %u %f %f %f %f %f %f %n",
                        &next.seed, &next.v_desired, &next.offset_m,
                        &next.heading_deg, &next.amp_m, &next.wavelength_m,
                        &next.duration_s, &consumed);
    if (fields != 7 || consumed <= 0 || line[consumed] != '\0' ||
        !isfinite(next.v_desired) || !isfinite(next.offset_m) ||
        !isfinite(next.heading_deg) || !isfinite(next.amp_m) ||
        !isfinite(next.wavelength_m) || !isfinite(next.duration_s) ||
        next.v_desired < 0.0f || next.wavelength_m <= 0.0f ||
        next.duration_s <= 0.0f) {
        return -1;
    }
    env->options = next;
    reset_session(env);
    return 0;
}

static int serve_client(int fd, live_env_t *env)
{
    if (send_ready(fd, env) != 0) {
        return 0;
    }
    char line[512];
    for (;;) {
        int read_result = read_line(fd, line, sizeof(line));
        if (read_result <= 0) {
            return 0;
        }
        if (strcmp(line, "RESET") == 0 || strncmp(line, "RESET ", 6) == 0) {
            if (reset_from_command(env, line) != 0) {
                if (send_line(fd, "{\"event\":\"error\",\"message\":\"invalid RESET\"}\n") != 0) return 0;
            } else if (send_line(fd, "{\"event\":\"reset\",\"t_us\":0}\n") != 0) {
                return 0;
            }
        } else if (strcmp(line, "STEP") == 0) {
            nncar_log_row_t row;
            bool stepped = false;
            const int substeps = (int)lroundf(env->profile->line_dt_s /
                                               env->profile->motor_dt_s);
            for (int i = 0; i < substeps; ++i) {
                if (!nncar_sim_session_step(&env->session, &row)) {
                    break;
                }
                stepped = true;
            }
            if (stepped) {
                if (send_state(fd, env, &row) != 0) return 0;
            } else if (send_line(fd, "{\"event\":\"done\"}\n") != 0) {
                return 0;
            }
        } else if (strcmp(line, "CLOSE") == 0) {
            (void)send_line(fd, "{\"event\":\"closed\"}\n");
            return 0;
        } else if (strcmp(line, "HELLO") == 0) {
            if (send_ready(fd, env) != 0) return 0;
        } else if (send_line(fd, "{\"event\":\"error\",\"message\":\"unknown command\"}\n") != 0) {
            return 0;
        }
    }
}

int main(int argc, char **argv)
{
    options_t options;
    if (parse_options(argc, argv, &options) != 0) {
        fprintf(stderr, "usage: %s [--port 37621] [--duration 60] [--v 0.8] "
                "[--offset 0] [--heading-deg 0] [--amp 0.08] "
                "[--wavelength 3.0] [--rail 12] [--seed 23]\n", argv[0]);
        return 2;
    }

    live_env_t env;
    memset(&env, 0, sizeof(env));
    env.profile = nncar_profile_default();
    env.options = options;
    reset_session(&env);

    int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) {
        perror("nncar_live: socket");
        return 1;
    }
    int reuse = 1;
    (void)setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons((uint16_t)options.port);
    if (bind(server, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server, 1) != 0) {
        perror("nncar_live: bind/listen");
        close(server);
        return 1;
    }
    fprintf(stderr, "nncar_live: listening on 127.0.0.1:%d; profile_verified=%s\n",
            options.port, nncar_profile_is_verified() ? "true" : "false");

    int result = 0;
    for (;;) {
        int client = accept(server, NULL, NULL);
        if (client < 0) {
            if (errno == EINTR) continue;
            perror("nncar_live: accept");
            result = 1;
            break;
        }
        reset_session(&env);
        (void)serve_client(client, &env);
        close(client);
    }
    close(server);
    return result;
}
