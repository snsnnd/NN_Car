#ifndef NNCAR_NN_H
#define NNCAR_NN_H

#include "nncar/types.h"

/*
 * Minimal fully-connected inference runtime.
 *
 * Deliberately not a tensor library: one-dimensional layers only, no dynamic
 * allocation, no shape checks at run time. Shapes are frozen by the model
 * structs in line.h / motor.h, so a mismatch is a compile error, not a branch.
 *
 * Weights are addressed by pointer, so the same runtime serves weights that
 * live in a generated C struct, in a flash blob, or in a test fixture. Nothing
 * is copied or repacked at init.
 */
typedef enum {
    NNCAR_NN_ACT_NONE = 0,
    NNCAR_NN_ACT_RELU = 1,
    NNCAR_NN_ACT_TANH = 2,
    NNCAR_NN_ACT_SIGMOID = 3
} nncar_nn_act_t;

/* One dense layer: y[o] = act(b[o] + sum_i w[o * in + i] * x[i]). */
typedef struct {
    uint16_t in;
    uint16_t out;
    uint8_t act;      /* nncar_nn_act_t */
    const float *w;   /* [out][in], row-major */
    const float *b;   /* [out] */
} nncar_nn_layer_t;

/*
 * A stack of layers. The final layer is expected to use ACT_NONE so that the
 * model keeps ownership of its head semantics (s_v is sigmoid while kappa is
 * tanh, which one activation per layer cannot express).
 */
typedef struct {
    const nncar_nn_layer_t *layers;
    uint8_t count;
} nncar_nn_model_t;

/* Floats of scratch required: two ping-pong buffers of the widest layer. */
uint16_t nncar_nn_scratch_len(const nncar_nn_model_t *model);

/*
 * Inference. `scratch` must hold at least nncar_nn_scratch_len(model) floats
 * and `y` must hold model->layers[count - 1].out floats. `y` may alias `x`
 * only when the last layer is wider than the input, which no current model is.
 */
void nncar_nn_forward(const nncar_nn_model_t *model, const float *x, float *y,
                      float *scratch);

/* ------------------------------------------------------------------ */
/* int8 path: same layer shape, integer accumulation.                   */
/* ------------------------------------------------------------------ */

/*
 * Symmetric int8 weights with a per-tensor scale, int32 bias pre-scaled by
 * in_scale * w_scale, and one float multiplier per layer for requantization.
 * The float multiply costs O(out) per layer (a few hundred cycles), not O(in*out).
 *
 * `act` uses nncar_nn_act_t. The last layer must be ACT_NONE: its int32 output
 * is dequantized with `out_scale` and the head is applied by the model.
 */
typedef struct {
    uint16_t in;
    uint16_t out;
    uint8_t act;
    const int8_t *w;   /* [out][in] */
    const int32_t *b;  /* [out], already divided by in_scale * w_scale */
    float mult;        /* in_scale * w_scale / out_scale */
    float out_scale;
} nncar_nn_q_layer_t;

typedef struct {
    const nncar_nn_q_layer_t *layers;
    uint8_t count;
    float input_scale;
} nncar_nn_q_model_t;

/* Two int8 ping-pong buffers of the widest layer. */
uint16_t nncar_nn_q_scratch_len(const nncar_nn_q_model_t *model);

/* `x` is float and is quantized here with input_scale; `y` is float. */
void nncar_nn_q_forward(const nncar_nn_q_model_t *model, const float *x,
                        float *y, int8_t *scratch);

#endif
