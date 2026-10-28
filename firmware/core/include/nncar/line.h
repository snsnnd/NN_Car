#ifndef NNCAR_LINE_H
#define NNCAR_LINE_H

#include "nncar/gray.h"
#include "nncar/nn.h"
#include "nncar/profile.h"

typedef enum {
    NNCAR_LINE_TRACKING = 0,
    NNCAR_LINE_RECOVERY = 1
} nncar_line_mode_t;

/* Which LineFollower runs in tracking mode; recovery is shared. */
typedef enum {
    NNCAR_LINE_CTRL_PID = 0,
    NNCAR_LINE_CTRL_MLP = 1,
    /*
     * Residual form: kappa = kappa_pid + line_kappa_alpha * net, where the base
     * is the PID evaluated at the same state. Speed still comes from the net's
     * s_v head so the only difference from CTRL_MLP is where the curvature
     * baseline comes from. A NULL network falls back to the PID.
     */
    NNCAR_LINE_CTRL_MLP_RESIDUAL = 2,
    /* Same policy as CTRL_MLP, evaluated with the int8 kernel and weights. */
    NNCAR_LINE_CTRL_MLP_INT8 = 3
} nncar_line_ctrl_t;

#ifndef NNCAR_LINE_INPUT_DIM
#define NNCAR_LINE_INPUT_DIM 68
#endif
#define NNCAR_LINE_H1 64
#define NNCAR_LINE_H2 32
#define NNCAR_LINE_H3 16
#define NNCAR_LINE_HISTORY_TAPS 10

/* Pointer view of one gray frame; the caller keeps ownership. */
typedef struct {
    const float *value;   /* NNCAR_MAX_SENSORS, calibrated to [-1, 1] */
    const uint8_t *exist;
    const uint8_t *valid;
} nncar_gray_frame_t;

/*
 * 68 -> 64 -> 32 -> 16 -> 2, 7,058 parameters, FP32.
 * Outputs are [sigmoid(s_v), tanh(kappa_norm)] (R3 section 4.1).
 */
typedef struct {
    float w1[NNCAR_LINE_H1][NNCAR_LINE_INPUT_DIM];
    float b1[NNCAR_LINE_H1];
    float w2[NNCAR_LINE_H2][NNCAR_LINE_H1];
    float b2[NNCAR_LINE_H2];
    float w3[NNCAR_LINE_H3][NNCAR_LINE_H2];
    float b3[NNCAR_LINE_H3];
    float w4[2][NNCAR_LINE_H3];
    float b4[2];
} nncar_line_nn_t;

typedef struct {
    nncar_line_ctrl_t ctrl;
    const nncar_line_nn_t *nn;
    nncar_line_mode_t mode;
    float low_hold_s;
    float high_hold_s;
    float recovery_time_s;
    float ramp_s;
    float last_centroid_m;
    bool has_last_centroid;
    float prev_centroid_m;
    float history_centroid_m;   /* previous tick's centroid, as fed to the net */
    float centroid_history_m[NNCAR_LINE_HISTORY_TAPS];
    float prev_t_s;
    float d_filtered;
    bool has_prev;
    nncar_twist_t recovery_cmd;
    uint32_t transitions;
} nncar_line_state_t;

void nncar_line_reset(nncar_line_state_t *st);

/* Select the tracking controller; a NULL network falls back to PID. */
void nncar_line_set_ctrl(nncar_line_state_t *st, nncar_line_ctrl_t ctrl,
                         const nncar_line_nn_t *nn);

/*
 * Deterministic PID teacher or the MLP, plus the shared recovery state
 * machine. Returns the commanded twist already clamped by the kinematic /
 * lateral envelope.
 */
nncar_twist_t nncar_line_step(const nncar_profile_t *p,
                              nncar_line_state_t *st,
                              const nncar_gray_frame_t *frame,
                              const nncar_line_estimate_t *est,
                              float v_desired,
                              float v_max_eff,
                              float t_s,
                              float dt_s);

/* R3 network contract: y = [s_v, kappa_norm] -> TargetTwist. */
nncar_twist_t nncar_twist_from_kappa(const nncar_profile_t *p,
                                     float s_v,
                                     float kappa_norm,
                                     float v_desired,
                                     float v_max_eff);

/* Clamp omega by min(omega_kin, omega_lat) and clamp v by v_max_eff. */
nncar_twist_t nncar_twist_envelope(const nncar_profile_t *p,
                                   nncar_twist_t twist,
                                   float v_max_eff);

/* Residual-free network: every weight and bias becomes 0. */
void nncar_line_nn_zero(nncar_line_nn_t *nn);

/* y[0] = sigmoid(s_v) in [0, 1], y[1] = tanh(kappa_norm) in [-1, 1]. */
void nncar_line_nn_eval(const nncar_line_nn_t *nn,
                        const float x[NNCAR_LINE_INPUT_DIM], float y[2]);

/*
 * Advance the shared temporal filter: d_filtered = LPF((e_k - e_{k-1})/dt) and
 * history_centroid_m = e_{k-1}. Called once per line tick, in tracking mode.
 */
void nncar_line_history_update(const nncar_profile_t *p,
                               nncar_line_state_t *st,
                               const nncar_line_estimate_t *est,
                               float dt_s);

/* Frozen 68-D input contract (R3 section 4.1). */
void nncar_line_nn_input(const nncar_profile_t *p,
                         const nncar_gray_frame_t *frame,
                         const nncar_line_estimate_t *est,
                         float v_desired,
                         float x[NNCAR_LINE_INPUT_DIM]);

/* Trained weights, generated from configs/line_nn_weights.json. */
const nncar_line_nn_t *nncar_line_nn_trained(void);

/* Residual-baseline weights, generated from configs/line_nn_residual.json. */
const nncar_line_nn_t *nncar_line_nn_residual_trained(void);

/* int8 weights + per-tensor scales, generated from the same JSON. */
const nncar_nn_q_model_t *nncar_line_nn_int8_trained(void);

/* int8 evaluation of the same policy; used by NNCAR_LINE_CTRL_MLP_INT8. */
void nncar_line_nn_eval_q(const nncar_nn_q_model_t *nn,
                          const float x[NNCAR_LINE_INPUT_DIM], float y[2]);

/*
 * Curvature the PID would ask for at this exact state (post-envelope), i.e.
 * omega / v. Used as the residual baseline and as the dump label anchor.
 */
float nncar_line_pid_kappa(const nncar_profile_t *p, const nncar_line_state_t *st,
                          const nncar_line_estimate_t *est, float v_desired,
                          float v_max_eff);

#endif
