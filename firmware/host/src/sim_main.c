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
    const char *map_name;
    float duration_s;
    float v_desired;
    float offset_m;
    float heading_deg;
    float amp_m;
    float wavelength_m;
    float rail_v;
    bool list_maps;
    bool offset_set;
    bool heading_set;
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
            "          [--map <name>] [--list-maps]\n",
            argv0);
}

static int parse(int argc, char **argv, options_t *o)
{
    o->out = NULL;
    o->manifest = NULL;
    o->map_name = NULL;
    o->duration_s = 8.0f;
    o->v_desired = 0.9f;
    o->offset_m = 0.02f;
    o->heading_deg = 3.0f;
    o->amp_m = 0.0f;
    o->wavelength_m = 1.0f;
    o->rail_v = 12.0f;
    o->list_maps = false;
    o->offset_set = false;
    o->heading_set = false;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--out") == 0 && i + 1 < argc) {
            o->out = argv[++i];
        } else if (strcmp(a, "--manifest") == 0 && i + 1 < argc) {
            o->manifest = argv[++i];
        } else if (strcmp(a, "--map") == 0 && i + 1 < argc) {
            o->map_name = argv[++i];
        } else if (strcmp(a, "--list-maps") == 0) {
            o->list_maps = true;
        } else if (strcmp(a, "--duration") == 0 && i + 1 < argc) {
            o->duration_s = (float)atof(argv[++i]);
        } else if (strcmp(a, "--v") == 0 && i + 1 < argc) {
            o->v_desired = (float)atof(argv[++i]);
        } else if (strcmp(a, "--offset") == 0 && i + 1 < argc) {
            o->offset_m = (float)atof(argv[++i]);
            o->offset_set = true;
        } else if (strcmp(a, "--heading-deg") == 0 && i + 1 < argc) {
            o->heading_deg = (float)atof(argv[++i]);
            o->heading_set = true;
        } else if (strcmp(a, "--amp") == 0 && i + 1 < argc) {
            o->amp_m = (float)atof(argv[++i]);
        } else if (strcmp(a, "--wavelength") == 0 && i + 1 < argc) {
            o->wavelength_m = (float)atof(argv[++i]);
        } else if (strcmp(a, "--rail") == 0 && i + 1 < argc) {
            o->rail_v = (float)atof(argv[++i]);
        } else {
            usage(argv[0]);
            return -1;
        }
    }

    if (o->out == NULL) {
        o->out = "run.csv";
    }
    if (o->duration_s <= 0.0f || o->wavelength_m <= 0.0f) {
        usage(argv[0]);
        return -1;
    }
    return 0;
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
            "  \"schema_version\": \"1.0.0\",\n"
            "  \"generator\": \"nncar_sim\",\n"
            "  \"profile\": \"%s\",\n"
            "  \"profile_version\": \"%s\",\n"
            "  \"profile_verified\": %s,\n"
            "  \"seed\": 0,\n"
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
            "  },\n",
            nncar_profile_name(), nncar_profile_version(),
            nncar_profile_is_verified() ? "true" : "false",
            (commit != NULL) ? commit : "unknown", (double)p->motor_dt_s,
            (double)p->line_dt_s, (double)o->duration_s, (double)o->v_desired,
            (double)o->offset_m, (double)o->heading_deg, (double)o->amp_m,
            (double)o->wavelength_m, (double)o->rail_v);

    const nncar_map_t *map = nncar_map_find(o->map_name);
    if (map != NULL) {
        fprintf(fp,
                "  \"map\": {\n"
                "    \"name\": \"%s\",\n"
                "    \"version\": \"%s\",\n"
                "    \"source\": \"%s\",\n"
                "    \"segment_count\": %d,\n"
                "    \"start_x_m\": %.4f,\n"
                "    \"start_y_m\": %.4f,\n"
                "    \"start_theta_rad\": %.6f\n"
                "  }\n",
                map->name, map->version, map->source, map->segment_count,
                (double)map->start_x, (double)map->start_y,
                (double)map->start_theta);
    } else {
        fprintf(fp, "  \"map\": null\n");
    }
    fprintf(fp, "}\n");

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

    if (opt.list_maps) {
        for (int i = 0; i < nncar_map_count(); ++i) {
            const nncar_map_t *m = nncar_map_at(i);
            printf("%-16s v%s  segments=%3d  x=[%.2f, %.2f] y=[%.2f, %.2f]  start=(%.2f, %.2f)\n",
                   m->name, m->version, m->segment_count,
                   (double)m->x_min, (double)m->x_max,
                   (double)m->y_min, (double)m->y_max,
                   (double)m->start_x, (double)m->start_y);
        }
        return 0;
    }

    const nncar_profile_t *p = nncar_profile_default();

    fprintf(stderr, "profile: %s v%s verified=%s\n", nncar_profile_name(),
            nncar_profile_version(), nncar_profile_is_verified() ? "true" : "false");
    if (!nncar_profile_is_verified()) {
        fprintf(stderr, "warning: profile contains unverified placeholder parameters\n");
    }

    const nncar_map_t *map = NULL;
    if (opt.map_name != NULL) {
        map = nncar_map_find(opt.map_name);
        if (map == NULL) {
            fprintf(stderr, "unknown map: %s (use --list-maps)\n", opt.map_name);
            return 2;
        }
    }

    nncar_sim_t sim;
    if (map != NULL) {
        if (!opt.offset_set) {
            opt.offset_m = 0.0f;
        }
        if (!opt.heading_set) {
            opt.heading_deg = 0.0f;
        }
        const float theta0 = map->start_theta + opt.heading_deg * NNCAR_PI / 180.0f;
        const float x0 = map->start_x - sinf(theta0) * opt.offset_m;
        const float y0 = map->start_y + cosf(theta0) * opt.offset_m;
        nncar_sim_init(&sim, p, x0, y0, theta0, opt.rail_v);
        nncar_sim_set_map(&sim, map);
        fprintf(stderr, "map: %s v%s segments=%d start=(%.3f, %.3f) theta=%.3frad\n",
                map->name, map->version, map->segment_count,
                (double)x0, (double)y0, (double)theta0);
    } else {
        nncar_sim_init(&sim, p, 0.0f, opt.offset_m,
                       opt.heading_deg * NNCAR_PI / 180.0f, opt.rail_v);
        sim.track_amp_m = opt.amp_m;
        sim.track_wavelength_m = opt.wavelength_m;
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
