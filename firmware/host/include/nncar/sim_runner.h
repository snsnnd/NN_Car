#ifndef NNCAR_SIM_RUNNER_H
#define NNCAR_SIM_RUNNER_H

#include "nncar/log_csv.h"
#include "nncar/sim.h"

/* Single-source host closed-loop tick loop used by both the CLI and tests. */
typedef struct {
    float duration_s;
    float v_desired;
} nncar_sim_run_cfg_t;

typedef struct {
    float final_lat_m;
    float max_lat_m;
    float max_omega;
    uint32_t fault_union;
    uint64_t ticks;
} nncar_sim_run_result_t;

typedef void (*nncar_row_fn)(void *ctx, const nncar_log_row_t *row);

nncar_sim_run_result_t nncar_sim_run(const nncar_profile_t *p,
                                     nncar_sim_t *sim,
                                     nncar_sim_run_cfg_t cfg,
                                     nncar_row_fn on_row,
                                     void *ctx);

#endif
