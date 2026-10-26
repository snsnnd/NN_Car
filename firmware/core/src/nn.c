#include "nncar/nn.h"

#include <stddef.h>

static float layer_act(uint8_t act, float x)
{
    switch (act) {
    case NNCAR_NN_ACT_RELU:
        return (x > 0.0f) ? x : 0.0f;
    case NNCAR_NN_ACT_TANH:
        return tanhf(x);
    case NNCAR_NN_ACT_SIGMOID:
        return 1.0f / (1.0f + expf(-x));
    default:
        return x;
    }
}

uint16_t nncar_nn_q_scratch_len(const nncar_nn_q_model_t *model)
{
    uint16_t widest = 0;

    for (uint8_t l = 0; l < model->count; ++l) {
        if (model->layers[l].out > widest) {
            widest = model->layers[l].out;
        }
    }

    /* Two ping-pong buffers plus a separate input buffer: the quantized input
     * can be wider than the widest layer (68 vs 64), so it must not overlap. */
    return (uint16_t)(2u * widest + model->layers[0].in);
}

static int8_t quantize_s8(float v, float inv_scale)
{
    float q = roundf(v * inv_scale);

    if (q > 127.0f) {
        q = 127.0f;
    } else if (q < -127.0f) {
        q = -127.0f;
    }

    return (int8_t)q;
}

void nncar_nn_q_forward(const nncar_nn_q_model_t *model, const float *x,
                        float *y, int8_t *scratch)
{
    uint16_t widest = 0;
    for (uint8_t l = 0; l < model->count; ++l) {
        if (model->layers[l].out > widest) {
            widest = model->layers[l].out;
        }
    }
    const float inv_in = 1.0f / fmaxf(model->input_scale, 1.0e-12f);
    int8_t *qin = scratch + 2u * widest;

    /* Quantize the input into its own buffer, outside the ping-pong pair. */
    for (uint16_t i = 0; i < model->layers[0].in; ++i) {
        qin[i] = quantize_s8(x[i], inv_in);
    }

    const int8_t *in = qin;

    for (uint8_t l = 0; l < model->count; ++l) {
        const nncar_nn_q_layer_t *layer = &model->layers[l];
        const bool last = (l + 1u == model->count);
        int8_t *out = last ? NULL
                           : (((l % 2u) == 0u) ? scratch + widest : scratch);

        for (uint16_t o = 0; o < layer->out; ++o) {
            const int8_t *row = layer->w + (unsigned)o * layer->in;
            int32_t acc = layer->b[o];

            for (uint16_t i = 0; i < layer->in; ++i) {
                acc += (int32_t)row[i] * (int32_t)in[i];
            }

            if (last) {
                /* Dequantize the logits; the head belongs to the model. */
                y[o] = (float)acc * layer->out_scale;
                continue;
            }

            /* mult already divides by out_scale, so acc*mult is the quantized
             * value; go back to real units for the activation, then requantize. */
            float z = (float)acc * layer->mult * layer->out_scale;
            if (layer->act == NNCAR_NN_ACT_RELU && z < 0.0f) {
                z = 0.0f;
            } else if (layer->act == NNCAR_NN_ACT_TANH) {
                z = tanhf(z);
            } else if (layer->act == NNCAR_NN_ACT_SIGMOID) {
                z = 1.0f / (1.0f + expf(-z));
            }
            out[o] = quantize_s8(z, 1.0f / fmaxf(layer->out_scale, 1.0e-12f));
        }

        if (!last) {
            in = out;
        }
    }
}

uint16_t nncar_nn_scratch_len(const nncar_nn_model_t *model)
{
    uint16_t widest = 0;

    for (uint8_t l = 0; l < model->count; ++l) {
        if (model->layers[l].out > widest) {
            widest = model->layers[l].out;
        }
    }

    return (uint16_t)(2u * widest);
}

void nncar_nn_forward(const nncar_nn_model_t *model, const float *x, float *y,
                      float *scratch)
{
    const uint16_t widest = (uint16_t)(nncar_nn_scratch_len(model) / 2u);
    const float *in = x;

    for (uint8_t l = 0; l < model->count; ++l) {
        const nncar_nn_layer_t *layer = &model->layers[l];
        /* Ping-pong: even layers land in the first buffer, odd in the second. */
        float *out = (l + 1u == model->count)
                         ? y
                         : (((l % 2u) == 0u) ? scratch : scratch + widest);

        for (uint16_t o = 0; o < layer->out; ++o) {
            const float *row = layer->w + (unsigned)o * layer->in;
            float acc = layer->b[o];
            for (uint16_t i = 0; i < layer->in; ++i) {
                acc += row[i] * in[i];
            }
            out[o] = layer_act(layer->act, acc);
        }

        in = out;
    }
}
