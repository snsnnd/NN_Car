#include "nncar/line.h"
#include "nncar/motor.h"
#include "nncar/nn.h"
#include "test_util.h"

/*
 * Golden values captured from the pre-runtime hand-written implementations.
 * They pin the float accumulation order: refactoring the loops must not move
 * a single bit.
 */
static void test_golden(const nncar_line_nn_t *line_nn, const nncar_motor_nn_t *motor_nn)
{
    float lx[NNCAR_LINE_INPUT_DIM];
    for (int i = 0; i < NNCAR_LINE_INPUT_DIM; ++i) {
        lx[i] = (float)((i % 13) - 6) / 6.0f;
    }

    float ly[2];
    nncar_line_nn_eval(line_nn, lx, ly);
    CHECK_NEAR(ly[0], 0.891498804f, 1.0e-9f);
    CHECK_NEAR(ly[1], 0.710869074f, 1.0e-9f);

    const float p1[NNCAR_MOTOR_INPUT_DIM] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    const float p2[NNCAR_MOTOR_INPUT_DIM] = {0.1f, 0.05f, 0.0f, 0.08f, 0.95f, 1.0f, 1.0f};

    CHECK_NEAR(nncar_motor_nn_eval(motor_nn, p1), 0.0230435599f, 1.0e-9f);
    CHECK_NEAR(nncar_motor_nn_eval(motor_nn, p2), 0.999193072f, 1.0e-9f);
}

/* One dense layer with the same accumulate order as the runtime. */
static void test_activations(void)
{
    const float w[2] = {1.0f, 0.0f};  /* out 2, in 1 */
    const float b[2] = {0.0f, 0.0f};
    const float x[1] = {0.5f};
    float scratch[4];
    float y[2];

    const nncar_nn_layer_t layers[1] = {{1, 2, NNCAR_NN_ACT_RELU, w, b}};
    const nncar_nn_model_t model = {layers, 1};

    CHECK(nncar_nn_scratch_len(&model) == 4);

    nncar_nn_forward(&model, x, y, scratch);
    CHECK_NEAR(y[0], 0.5f, 1.0e-9f);
    CHECK_NEAR(y[1], 0.0f, 1.0e-9f);

    /* Negative input: ReLU clamps, NONE passes through. */
    const float xn[1] = {-0.5f};
    const nncar_nn_layer_t relu_neg[1] = {{1, 1, NNCAR_NN_ACT_RELU, w, b}};
    const nncar_nn_layer_t none_neg[1] = {{1, 1, NNCAR_NN_ACT_NONE, w, b}};
    const nncar_nn_model_t m_relu = {relu_neg, 1};
    const nncar_nn_model_t m_none = {none_neg, 1};

    nncar_nn_forward(&m_relu, xn, y, scratch);
    CHECK_NEAR(y[0], 0.0f, 1.0e-9f);
    nncar_nn_forward(&m_none, xn, y, scratch);
    CHECK_NEAR(y[0], -0.5f, 1.0e-9f);

    const nncar_nn_layer_t sat[1] = {{1, 1, NNCAR_NN_ACT_TANH, w, b}};
    const nncar_nn_layer_t sig[1] = {{1, 1, NNCAR_NN_ACT_SIGMOID, w, b}};
    const nncar_nn_model_t m_tanh = {sat, 1};
    const nncar_nn_model_t m_sig = {sig, 1};
    const float x2[1] = {2.0f};

    nncar_nn_forward(&m_tanh, x2, y, scratch);
    CHECK_NEAR(y[0], tanhf(2.0f), 1.0e-6f);
    nncar_nn_forward(&m_sig, x2, y, scratch);
    CHECK_NEAR(y[0], 1.0f / (1.0f + expf(-2.0f)), 1.0e-6f);
}

/*
 * Two hidden layers force the ping-pong buffers to actually alternate: a
 * broken buffer selection passes a single-layer test and fails here.
 */
static void test_ping_pong(void)
{
    static float w1[2][3];
    static float w2[2][2];
    static float w3[1][2];
    static const float b1[2] = {0.1f, -0.2f};
    static const float b2[2] = {0.0f, 0.05f};
    static const float b3[1] = {0.3f};
    for (int o = 0; o < 2; ++o) {
        for (int i = 0; i < 3; ++i) {
            w1[o][i] = 0.1f * (float)(o + 1) - 0.05f * (float)i;
        }
        for (int i = 0; i < 2; ++i) {
            w2[o][i] = 0.2f * (float)(i + 1) - 0.1f * (float)o;
        }
        w3[0][o] = 0.3f - 0.1f * (float)o;
    }

    const nncar_nn_layer_t layers[3] = {
        {3, 2, NNCAR_NN_ACT_RELU, &w1[0][0], b1},
        {2, 2, NNCAR_NN_ACT_RELU, &w2[0][0], b2},
        {2, 1, NNCAR_NN_ACT_NONE, &w3[0][0], b3},
    };
    const nncar_nn_model_t model = {layers, 3};
    const float x[3] = {1.0f, -0.5f, 0.25f};
    float scratch[4];
    float y[1];
    float h1[2];
    float h2[2];

    for (int i = 0; i < 2; ++i) {
        h1[i] = fmaxf(b1[i] + w1[i][0] * x[0] + w1[i][1] * x[1] + w1[i][2] * x[2], 0.0f);
    }
    for (int i = 0; i < 2; ++i) {
        h2[i] = fmaxf(b2[i] + w2[i][0] * h1[0] + w2[i][1] * h1[1], 0.0f);
    }
    const float ref = b3[0] + w3[0][0] * h2[0] + w3[0][1] * h2[1];

    nncar_nn_forward(&model, x, y, scratch);
    CHECK_NEAR(y[0], ref, 1.0e-5f);
}

/* A zeroed network must stay exactly zero through the runtime. */
static void test_zero_network(void)
{
    static float z1[NNCAR_LINE_H1][NNCAR_LINE_INPUT_DIM];
    static float z2[NNCAR_LINE_H2][NNCAR_LINE_H1];
    static float z3[NNCAR_LINE_H3][NNCAR_LINE_H2];
    static float z4[2][NNCAR_LINE_H3];
    static float zb1[NNCAR_LINE_H1];
    static float zb2[NNCAR_LINE_H2];
    static float zb3[NNCAR_LINE_H3];
    static float zb4[2];

    const nncar_nn_layer_t layers[4] = {
        {NNCAR_LINE_INPUT_DIM, NNCAR_LINE_H1, NNCAR_NN_ACT_RELU, &z1[0][0], zb1},
        {NNCAR_LINE_H1, NNCAR_LINE_H2, NNCAR_NN_ACT_RELU, &z2[0][0], zb2},
        {NNCAR_LINE_H2, NNCAR_LINE_H3, NNCAR_NN_ACT_RELU, &z3[0][0], zb3},
        {NNCAR_LINE_H3, 2, NNCAR_NN_ACT_NONE, &z4[0][0], zb4},
    };
    const nncar_nn_model_t model = {layers, 4};
    float x[NNCAR_LINE_INPUT_DIM];
    float scratch[2 * NNCAR_LINE_H1];
    float y[2];

    for (int i = 0; i < NNCAR_LINE_INPUT_DIM; ++i) {
        x[i] = (float)(i % 7) - 3.0f;
    }

    nncar_nn_forward(&model, x, y, scratch);
    CHECK_NEAR(y[0], 0.0f, 1.0e-12f);
    CHECK_NEAR(y[1], 0.0f, 1.0e-12f);
}

/* The int8 kernel must track the float kernel closely; a scale or buffer
 * aliasing bug shows up here long before it reaches the closed loop. */
static void test_int8_matches_float(void)
{
    float x[NNCAR_LINE_INPUT_DIM];
    for (int i = 0; i < NNCAR_LINE_INPUT_DIM; ++i) {
        x[i] = (float)((i % 13) - 6) / 6.0f;
    }

    float yf[2];
    float yq[2];
    nncar_line_nn_eval(nncar_line_nn_trained(), x, yf);
    nncar_line_nn_eval_q(nncar_line_nn_int8_trained(), x, yq);

    CHECK_NEAR(yq[0], yf[0], 0.03f);
    CHECK_NEAR(yq[1], yf[1], 0.06f);

    const nncar_nn_q_model_t *m = nncar_line_nn_int8_trained();
    CHECK(nncar_nn_q_scratch_len(m) == 2u * NNCAR_LINE_H1 + NNCAR_LINE_INPUT_DIM);
}

int main(void)
{
    test_activations();
    test_ping_pong();
    test_golden(nncar_line_nn_trained(), nncar_motor_nn_trained());
    test_zero_network();
    test_int8_matches_float();
    return 0;
}
