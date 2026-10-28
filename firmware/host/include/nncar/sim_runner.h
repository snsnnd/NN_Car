#ifndef NNCAR_SIM_RUNNER_H
#define NNCAR_SIM_RUNNER_H

#include "nncar/log_csv.h"
#include "nncar/sim.h"
#include "nncar/stack.h"
#include "nncar/velocity.h"

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

/* Persistent fixed-step state shared by batch runs and interactive servers. */
typedef struct {
	const nncar_profile_t *profile;
	nncar_sim_t *sim;
	nncar_sim_run_cfg_t cfg;
	nncar_stack_t stack;
	nncar_velocity_t velocity_left;
	nncar_velocity_t velocity_right;
	float encoder_accum_left;
	float encoder_accum_right;
	float wheel_velocity_left;
	float wheel_velocity_right;
	uint64_t now_us;
	uint64_t last_edge_us;
	int total_ticks;
	int tick;
	bool have_edge;
	bool edge_prev;
	nncar_sim_run_result_t result;
} nncar_sim_session_t;

typedef void (*nncar_row_fn)(void *ctx, const nncar_log_row_t *row);

void nncar_sim_session_init(nncar_sim_session_t *session,
							const nncar_profile_t *p, nncar_sim_t *sim,
							nncar_sim_run_cfg_t cfg);

/* Advances exactly one motor tick and writes that tick's telemetry row. */
bool nncar_sim_session_step(nncar_sim_session_t *session, nncar_log_row_t *row);

nncar_sim_run_result_t nncar_sim_run(const nncar_profile_t *p,
                                     nncar_sim_t *sim,
                                     nncar_sim_run_cfg_t cfg,
                                     nncar_row_fn on_row,
                                     void *ctx);

#endif
