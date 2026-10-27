#include "nncar/log_csv.h"
#include "nncar/sim.h"
#include "nncar/sim_runner.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *out;
    const char *manifest;
    float duration_s;
    float v_desired;
    float offset_m;
    float heading_deg;
    float amp_m;
    float wavelength_m;
    float rail_v;
    float line_width_m;
    float motor_inertia_kgm2;
    float motor_damping_nms;
    float motor_torque_gain_nm;
    float motor_stiction_nm;
    float gray_noise_std;
    float gray_offset;
    float gray_gain;
    float gray_valid_prob;
    uint32_t seed;
    uint32_t gray_missing_mask;
    int motor_torque_gain_set;
} options_t;

typedef struct {
    nncar_logger_t *logger;
} row_sink_t;

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--out run.csv] [--manifest manifest.json]\n"
            "          [--duration 8] [--v 0.9] [--offset 0.02] [--heading-deg 3]\n"
            "          [--amp 0] [--wavelength 1.0] [--rail 12.0]\n"
            "          [--seed 0] [--line-width 0.02] [--stiction-nm 0]\n"
            "          [--gray-noise 0] [--gray-offset 0] [--gray-gain 1]\n"
            "          [--gray-valid-prob 1] [--gray-missing-mask 0]\n"
            "          [--motor-inertia 2e-5] [--motor-damping 4e-4]\n"
            "          [--motor-torque-gain <derived from profile>]\n",
            argv0);
}

static int parse_float_arg(int argc, char **argv, int *index, float *out)
{
    if (*index + 1 >= argc) {
        return -1;
    }

    char *end = NULL;
    const char *text = argv[++(*index)];
    float value = strtof(text, &end);
    if (end == text || *end != '\0' || !isfinite(value)) {
        return -1;
    }

    *out = value;
    return 0;
}

static int parse_u32_arg(int argc, char **argv, int *index, uint32_t *out)
{
    if (*index + 1 >= argc) {
        return -1;
    }

    char *end = NULL;
    const char *text = argv[++(*index)];
    unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0' || value > UINT32_MAX) {
        return -1;
    }

    *out = (uint32_t)value;
    return 0;
}

static int parse(int argc, char **argv, options_t *o)
{
    o->out = NULL;
    o->manifest = NULL;
    o->duration_s = 8.0f;
    o->v_desired = 0.9f;
    o->offset_m = 0.02f;
    o->heading_deg = 3.0f;
    o->amp_m = 0.0f;
    o->wavelength_m = 1.0f;
    o->rail_v = 12.0f;
    o->line_width_m = 0.02f;
    o->motor_inertia_kgm2 = 2.0e-5f;
    o->motor_damping_nms = 4.0e-4f;
    o->motor_torque_gain_nm = 0.0f;
    o->motor_stiction_nm = 0.0f;
    o->gray_noise_std = 0.0f;
    o->gray_offset = 0.0f;
    o->gray_gain = 1.0f;
    o->gray_valid_prob = 1.0f;
    o->seed = 0u;
    o->gray_missing_mask = 0u;
    o->motor_torque_gain_set = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--out") == 0 && i + 1 < argc) {
            o->out = argv[++i];
        } else if (strcmp(a, "--manifest") == 0 && i + 1 < argc) {
            o->manifest = argv[++i];
        } else if (strcmp(a, "--duration") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->duration_s) != 0) goto invalid;
        } else if (strcmp(a, "--v") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->v_desired) != 0) goto invalid;
        } else if (strcmp(a, "--offset") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->offset_m) != 0) goto invalid;
        } else if (strcmp(a, "--heading-deg") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->heading_deg) != 0) goto invalid;
        } else if (strcmp(a, "--amp") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->amp_m) != 0) goto invalid;
        } else if (strcmp(a, "--wavelength") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->wavelength_m) != 0) goto invalid;
        } else if (strcmp(a, "--rail") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->rail_v) != 0) goto invalid;
        } else if (strcmp(a, "--line-width") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->line_width_m) != 0) goto invalid;
        } else if (strcmp(a, "--motor-inertia") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->motor_inertia_kgm2) != 0) goto invalid;
        } else if (strcmp(a, "--motor-damping") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->motor_damping_nms) != 0) goto invalid;
        } else if (strcmp(a, "--motor-torque-gain") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->motor_torque_gain_nm) != 0) goto invalid;
            o->motor_torque_gain_set = 1;
        } else if (strcmp(a, "--stiction-nm") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->motor_stiction_nm) != 0) goto invalid;
        } else if (strcmp(a, "--gray-noise") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->gray_noise_std) != 0) goto invalid;
        } else if (strcmp(a, "--gray-offset") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->gray_offset) != 0) goto invalid;
        } else if (strcmp(a, "--gray-gain") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->gray_gain) != 0) goto invalid;
        } else if (strcmp(a, "--gray-valid-prob") == 0 && i + 1 < argc) {
            if (parse_float_arg(argc, argv, &i, &o->gray_valid_prob) != 0) goto invalid;
        } else if (strcmp(a, "--gray-missing-mask") == 0 && i + 1 < argc) {
            if (parse_u32_arg(argc, argv, &i, &o->gray_missing_mask) != 0) goto invalid;
        } else if (strcmp(a, "--seed") == 0 && i + 1 < argc) {
            if (parse_u32_arg(argc, argv, &i, &o->seed) != 0) goto invalid;
        } else {
            goto invalid;
        }
    }

    if (o->out == NULL) {
        o->out = "run.csv";
    }
    if (o->duration_s <= 0.0f || o->v_desired < 0.0f || o->wavelength_m <= 0.0f ||
        o->line_width_m <= 0.0f || o->motor_inertia_kgm2 <= 0.0f ||
        o->motor_damping_nms <= 0.0f || o->motor_torque_gain_nm < 0.0f ||
        o->motor_stiction_nm < 0.0f || o->gray_noise_std < 0.0f ||
        o->gray_offset < -1.0f || o->gray_offset > 1.0f || o->gray_gain < 0.0f ||
        o->gray_gain > 2.0f || o->gray_valid_prob < 0.0f || o->gray_valid_prob > 1.0f ||
        o->gray_missing_mask > 0xffffu) {
        goto invalid;
    }
    return 0;

invalid:
    usage(argv[0]);
    return -1;
}

static void write_manifest(const char *path, const options_t *o, const nncar_profile_t *p)
{
    FILE *fp = fopen(path, "w");

    if (fp == NULL) {
        return;
    }

    const char *commit = getenv("GIT_COMMIT");

    fprintf(fp,
            "{\n"
            "  \"schema_version\": \"1.1.0\",\n"
            "  \"generator\": \"nncar_sim\",\n"
            "  \"profile\": \"%s\",\n"
            "  \"profile_version\": \"%s\",\n"
            "  \"profile_verified\": %s,\n"
            "  \"seed\": %u,\n"
            "  \"git_commit\": \"%s\",\n"
            "  \"tick_dt_s\": %.6f,\n"
            "  \"line_dt_s\": %.6f,\n"
            "  \"scenario\": {\n"
            "    \"duration_s\": %.3f,\n"
            "    \"v_desired_mps\": %.3f,\n"
            "    \"offset_m\": %.3f,\n"
            "    \"heading_deg\": %.3f,\n"
            "    \"amp_m\": %.3f,\n"
            "    \"wavelength_m\": %.3f,\n"
            "    \"rail_v\": %.2f\n"
            "  },\n"
            "  \"vehicle\": {\n"
            "    \"wheel_radius_m\": %.6f,\n"
            "    \"track_width_m\": %.6f,\n"
            "    \"sensor_count\": %d,\n"
            "    \"sensor_to_axle_m\": %.6f,\n"
            "    \"sensor_position_m\": [",
            nncar_profile_name(), nncar_profile_version(),
            nncar_profile_is_verified() ? "true" : "false",
            (unsigned)o->seed, (commit != NULL) ? commit : "unknown", (double)p->motor_dt_s,
            (double)p->line_dt_s, (double)o->duration_s, (double)o->v_desired,
            (double)o->offset_m, (double)o->heading_deg, (double)o->amp_m,
            (double)o->wavelength_m, (double)o->rail_v, (double)p->wheel_radius_m,
            (double)p->track_width_m, p->sensor_count, (double)p->sensor_to_axle_m);

    for (int i = 0; i < p->sensor_count; ++i) {
        fprintf(fp, "%s%.6f", (i == 0) ? "" : ", ", (double)p->sensor_position_m[i]);
    }

    float torque_gain = o->motor_torque_gain_set ? o->motor_torque_gain_nm :
                        o->motor_damping_nms * p->v_max_nom_mps / p->wheel_radius_m;
    fprintf(fp,
            "]\n"
            "  },\n"
            "  \"simulation\": {\n"
            "    \"track_model\": \"sine_centerline\",\n"
            "    \"line_width_m\": %.6f,\n"
            "    \"motor_model\": \"first_order_dc_motor\",\n"
            "    \"motor_inertia_kgm2\": %.9g,\n"
            "    \"motor_damping_nms\": %.9g,\n"
            "    \"motor_torque_gain_nm\": %.9g,\n"
            "    \"motor_stiction_nm\": %.9g,\n"
            "    \"gray_model\": \"top_hat_linear_falloff\",\n"
            "    \"gray_noise_std\": %.6f,\n"
            "    \"gray_offset\": %.6f,\n"
            "    \"gray_gain\": %.6f,\n"
            "    \"gray_valid_prob\": %.6f,\n"
            "    \"gray_missing_mask\": %u\n"
            "  }\n"
            "}\n",
            (double)o->line_width_m, (double)o->motor_inertia_kgm2,
            (double)o->motor_damping_nms, (double)torque_gain,
            (double)o->motor_stiction_nm, (double)o->gray_noise_std,
            (double)o->gray_offset, (double)o->gray_gain,
            (double)o->gray_valid_prob, (unsigned)o->gray_missing_mask);

    fclose(fp);
}

static void on_row(void *ctx, const nncar_log_row_t *row)
{
    row_sink_t *sink = (row_sink_t *)ctx;

    nncar_log_write_row(sink->logger, row);
}

int main(int argc, char **argv)
{
    options_t opt;

    if (parse(argc, argv, &opt) != 0) {
        return 2;
    }

    const nncar_profile_t *p = nncar_profile_default();

    fprintf(stderr, "profile: %s v%s verified=%s\n", nncar_profile_name(),
            nncar_profile_version(), nncar_profile_is_verified() ? "true" : "false");
    if (!nncar_profile_is_verified()) {
        fprintf(stderr, "warning: profile contains unverified placeholder parameters\n");
    }

    nncar_sim_t sim;
    nncar_sim_init(&sim, p, 0.0f, opt.offset_m, opt.heading_deg * NNCAR_PI / 180.0f,
                   opt.rail_v);
    sim.track_amp_m = opt.amp_m;
    sim.track_wavelength_m = opt.wavelength_m;
    sim.line_width_m = opt.line_width_m;
    sim.inertia_kgm2 = opt.motor_inertia_kgm2;
    sim.damping_nms = opt.motor_damping_nms;
    sim.torque_gain_nm = opt.motor_torque_gain_set ? opt.motor_torque_gain_nm :
                         sim.damping_nms * p->v_max_nom_mps / p->wheel_radius_m;
    sim.motor_stiction_nm = opt.motor_stiction_nm;
    sim.gray_noise_std = opt.gray_noise_std;
    sim.gray_offset = opt.gray_offset;
    sim.gray_valid_prob = opt.gray_valid_prob;
    nncar_sim_seed(&sim, opt.seed);
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        sim.gray_gain[i] = opt.gray_gain;
        sim.gray_missing[i] = (uint8_t)((opt.gray_missing_mask >> i) & 1u);
    }

    nncar_logger_t logger;
    if (nncar_log_open(&logger, opt.out) != 0) {
        fprintf(stderr, "cannot open %s\n", opt.out);
        return 1;
    }
    nncar_log_write_header(&logger);

    row_sink_t sink;
    sink.logger = &logger;

    nncar_sim_run_cfg_t cfg;
    cfg.duration_s = opt.duration_s;
    cfg.v_desired = opt.v_desired;

    nncar_sim_run_result_t res = nncar_sim_run(p, &sim, cfg, on_row, &sink);

    nncar_log_close(&logger);

    if (opt.manifest != NULL) {
        write_manifest(opt.manifest, &opt, p);
    }

    fprintf(stderr,
            "done: t=%.3fs x=%.3fm y=%.4fm final_lat=%.4fm max_lat=%.4fm "
            "max_omega=%.3f faults=0x%08x file=%s\n",
            (double)opt.duration_s, (double)sim.x, (double)sim.y,
            (double)res.final_lat_m, (double)res.max_lat_m, (double)res.max_omega,
            res.fault_union, opt.out);

    return 0;
}
