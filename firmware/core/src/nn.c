#include "nncar/nn.h"

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
