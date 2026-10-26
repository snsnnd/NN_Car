#ifndef NNCAR_MOTOR_H
#define NNCAR_MOTOR_H

#include "nncar/profile.h"

/* Residual network input: the 7 frozen dimensions (R3 section 6.1). */
#define NNCAR_MOTOR_INPUT_DIM 7
#define NNCAR_MOTOR_H1 32
#define NNCAR_MOTOR_H2 16
#define NNCAR_MOTOR_H3 8

/*
 * Three baselines, in increasing order of capability:
 *   PI            Kp*e + Ki*integral only (no feedforward, no deadband comp)
 *   PI_FF         + Kff*v_target/v_max_eff feedforward, rail scaling and
 *                   deterministic deadband compensation
 *   PI_RESIDUAL   + alpha * tanh(MLP(x_motor))
 *
 * PI_RESIDUAL with a zero (or absent) network evaluates to exactly PI_FF, so
 * the safe degradation path is the same code with u_nn = 0.
 */
typedef enum {
    NNCAR_MOTOR_MODE_PI = 0,
    NNCAR_MOTOR_MODE_PI_FF = 1,
    NNCAR_MOTOR_MODE_PI_RESIDUAL = 2
} nncar_motor_mode_t;

/* 7 -> 32 -> 16 -> 8 -> 1, 929 parameters, FP32. */
typedef struct {
    float w1[NNCAR_MOTOR_H1][NNCAR_MOTOR_INPUT_DIM];
    float b1[NNCAR_MOTOR_H1];
    float w2[NNCAR_MOTOR_H2][NNCAR_MOTOR_H1];
    float b2[NNCAR_MOTOR_H2];
    float w3[NNCAR_MOTOR_H3][NNCAR_MOTOR_H2];
    float b3[NNCAR_MOTOR_H3];
    float w4[1][NNCAR_MOTOR_H3];
    float b4[1];
} nncar_motor_nn_t;

typedef struct {
    float integral;
    float u;             /* last command before deadband compensation */
    float applied_pwm;   /* safe_pwm_applied[t-1], feeds previous_applied_pwm */
    bool enable_prev;
} nncar_motor_state_t;

typedef struct {
    nncar_motor_mode_t mode;
    const nncar_motor_nn_t *nn;   /* NULL disables the residual term */
} nncar_motor_cfg_t;

typedef struct {
    float target_mps;
    float actual_mps;
    float v_max_eff;
    float rail_ratio;   /* V_rail / V_rail_nom, clipped by the caller */
    bool rail_exist;
    bool rail_valid;
    bool enable;
} nncar_motor_input_t;

typedef struct {
    float u_pi;    /* PI + feedforward, saturated */
    float u_nn;    /* alpha * tanh(NN(x)) */
    float u_cmd;   /* saturated u_pi + u_nn */
    float pwm;     /* command after deadband compensation */
} nncar_motor_out_t;

void nncar_motor_reset(nncar_motor_state_t *st);

nncar_motor_out_t nncar_motor_step(const nncar_profile_t *p,
                                   nncar_motor_state_t *st,
                                   const nncar_motor_cfg_t *cfg,
                                   const nncar_motor_input_t *in,
                                   float dt_s);

/* Inverse deadband mapping applied by the actuator adapter. */
float nncar_deadband_compensate(const nncar_profile_t *p, float u);

/* Residual MLP. Zeroing the network makes the output identically 0. */
void nncar_motor_nn_zero(nncar_motor_nn_t *nn);
float nncar_motor_nn_eval(const nncar_motor_nn_t *nn,
                          const float x[NNCAR_MOTOR_INPUT_DIM]);

/* Trained weights, generated from configs/motor_nn_weights.json. */
const nncar_motor_nn_t *nncar_motor_nn_trained(void);

/* Frozen input contract, exposed so tests can pin the normalization. */
void nncar_motor_nn_input(const nncar_profile_t *p,
                          const nncar_motor_input_t *in,
                          float u_pi,
                          float previous_pwm,
                          float x[NNCAR_MOTOR_INPUT_DIM]);

#endif
