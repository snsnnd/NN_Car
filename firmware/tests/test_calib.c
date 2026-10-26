#include "nncar/calib.h"

#include <string.h>

#include "test_util.h"

#define SENTINEL 0xA5

typedef struct {
    uint8_t slot[2][sizeof(nncar_calib_record_t)];
    uint8_t written[2];
} ram_store_t;

static bool ram_read(void *ctx, uint8_t slot, void *dst, uint32_t len)
{
    ram_store_t *s = ctx;
    if (slot > 1u || len != sizeof(nncar_calib_record_t) || !s->written[slot]) {
        return false;
    }
    memcpy(dst, s->slot[slot], len);
    return true;
}

static bool ram_write(void *ctx, uint8_t slot, const void *src, uint32_t len)
{
    ram_store_t *s = ctx;
    if (slot > 1u || len != sizeof(nncar_calib_record_t)) {
        return false;
    }
    memset(s->slot[slot], SENTINEL, sizeof(s->slot[slot]));
    memcpy(s->slot[slot], src, len);
    s->written[slot] = 1u;
    return true;
}

static nncar_calib_store_t ram_store(ram_store_t *s)
{
    memset(s, 0, sizeof(*s));
    nncar_calib_store_t store = {s, ram_read, ram_write};
    return store;
}

static void test_record(void)
{
    nncar_calib_record_t rec;
    nncar_calib_init(&rec);
    CHECK(rec.version == NNCAR_CALIB_VERSION);
    CHECK(rec.count == 0u);
    CHECK(nncar_calib_crc(&rec) == rec.crc);

    nncar_calib_entry_t e;
    CHECK(!nncar_calib_get(&rec, NNCAR_CALIB_MOTOR_KFF, &e));

    CHECK(nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.07f, 0.9f,
                          NNCAR_CALIB_SRC_IDENTIFY));
    CHECK(nncar_calib_get(&rec, NNCAR_CALIB_MOTOR_DEADBAND, &e));
    CHECK_NEAR(e.value, 0.07f, 1.0e-6f);
    CHECK(e.source == NNCAR_CALIB_SRC_IDENTIFY);

    /* Same id updates in place instead of appending a second entry. */
    CHECK(nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.08f, 0.8f,
                          NNCAR_CALIB_SRC_ADAPT));
    CHECK(rec.count == 1u);
    CHECK(nncar_calib_get(&rec, NNCAR_CALIB_MOTOR_DEADBAND, &e));
    CHECK_NEAR(e.value, 0.08f, 1.0e-6f);

    /* Rejections: bad id, NaN, low quality, out of bounds, missing source. */
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_PARAM_COUNT, 0.1f, 1.0f,
                           NNCAR_CALIB_SRC_IDENTIFY));
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_KFF, NAN, 1.0f,
                           NNCAR_CALIB_SRC_IDENTIFY));
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_KFF, 0.9f, 0.1f,
                           NNCAR_CALIB_SRC_IDENTIFY));
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.9f, 1.0f,
                           NNCAR_CALIB_SRC_IDENTIFY));
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_KFF, 0.9f, 1.0f,
                           NNCAR_CALIB_SRC_NONE));

    /* Hand-entered values are protected from the automatic paths. */
    CHECK(nncar_calib_set(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, 1.8f, 1.0f,
                          NNCAR_CALIB_SRC_MANUAL));
    CHECK(!nncar_calib_set(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, 1.5f, 1.0f,
                           NNCAR_CALIB_SRC_ADAPT));
    CHECK(nncar_calib_get(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, &e));
    CHECK_NEAR(e.value, 1.8f, 1.0e-6f);
}

static void test_verified(void)
{
    nncar_calib_record_t rec;
    nncar_calib_init(&rec);

    CHECK(!nncar_calib_verified(&rec));
    CHECK(nncar_calib_pending_mask(&rec) == 0x7u);

    nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.07f, 0.9f, NNCAR_CALIB_SRC_IDENTIFY);
    CHECK(nncar_calib_pending_mask(&rec) == 0x6u);

    nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_KFF, 0.92f, 0.9f, NNCAR_CALIB_SRC_MANUAL);
    CHECK(nncar_calib_pending_mask(&rec) == 0x4u);

    /* A bench value is stored but must never make the record verified. */
    nncar_calib_set(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, 1.8f, 1.0f, NNCAR_CALIB_SRC_BENCH);
    CHECK(!nncar_calib_verified(&rec));
    CHECK(nncar_calib_pending_mask(&rec) == 0x4u);

    nncar_calib_set(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, 1.8f, 1.0f, NNCAR_CALIB_SRC_IDENTIFY);
    CHECK(nncar_calib_verified(&rec));
    CHECK(nncar_calib_pending_mask(&rec) == 0u);
}

static void test_store(void)
{
    ram_store_t ram;
    nncar_calib_store_t store = ram_store(&ram);

    /* Empty store: nothing to load. */
    nncar_calib_record_t rec;
    CHECK(!nncar_calib_load(&store, &rec));

    nncar_calib_init(&rec);
    strncpy(rec.vehicle_id, "car01", NNCAR_CALIB_ID_LEN - 1u);
    rec.stamp_us = 10u;
    nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.07f, 0.9f, NNCAR_CALIB_SRC_IDENTIFY);
    CHECK(nncar_calib_save(&store, &rec));
    CHECK(ram.written[0] == 1u); /* slot = stamp & 1 */

    nncar_calib_record_t back;
    CHECK(nncar_calib_load(&store, &back));
    CHECK(back.stamp_us == 10u);
    CHECK(strcmp(back.vehicle_id, "car01") == 0);
    nncar_calib_entry_t e;
    CHECK(nncar_calib_get(&back, NNCAR_CALIB_MOTOR_DEADBAND, &e));
    CHECK_NEAR(e.value, 0.07f, 1.0e-6f);

    /* A newer record in the other slot wins. */
    nncar_calib_record_t newer;
    nncar_calib_init(&newer);
    newer.stamp_us = 11u;
    nncar_calib_set(&newer, NNCAR_CALIB_MOTOR_DEADBAND, 0.09f, 0.9f, NNCAR_CALIB_SRC_IDENTIFY);
    CHECK(nncar_calib_save(&store, &newer));
    CHECK(ram.written[1] == 1u);
    CHECK(nncar_calib_load(&store, &back));
    CHECK(back.stamp_us == 11u);

    /* Corrupting the newer slot falls back to the older, intact one. */
    ram.slot[1][0] ^= 0xFFu;
    CHECK(nncar_calib_load(&store, &back));
    CHECK(back.stamp_us == 10u);
}

static void test_apply(void)
{
    nncar_calib_record_t rec;
    nncar_calib_init(&rec);
    nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_DEADBAND, 0.09f, 0.9f, NNCAR_CALIB_SRC_IDENTIFY);
    nncar_calib_set(&rec, NNCAR_CALIB_MOTOR_KFF, 0.95f, 0.9f, NNCAR_CALIB_SRC_MANUAL);
    nncar_calib_set(&rec, NNCAR_CALIB_LINE_KAPPA_GAIN, 2.1f, 0.9f, NNCAR_CALIB_SRC_IDENTIFY);

    nncar_profile_t p = *nncar_profile_default();
    const float wheel_radius = p.wheel_radius_m;
    const float track_width = p.track_width_m;

    nncar_calib_apply(&rec, &p);
    CHECK_NEAR(p.motor_deadband, 0.09f, 1.0e-6f);
    CHECK_NEAR(p.motor_kff, 0.95f, 1.0e-6f);
    CHECK_NEAR(p.line_kappa_gain, 2.1f, 1.0e-6f);

    /* Mechanical values are never touched by the record. */
    CHECK_NEAR(p.wheel_radius_m, wheel_radius, 1.0e-9f);
    CHECK_NEAR(p.track_width_m, track_width, 1.0e-9f);

    /* A bench-sourced entry is stored but not applied. */
    nncar_calib_record_t bench;
    nncar_calib_init(&bench);
    nncar_calib_set(&bench, NNCAR_CALIB_MOTOR_KFF, 0.5f, 0.9f, NNCAR_CALIB_SRC_BENCH);

    nncar_profile_t q = *nncar_profile_default();
    const float kff = q.motor_kff;
    nncar_calib_apply(&bench, &q);
    CHECK_NEAR(q.motor_kff, kff, 1.0e-9f);
}

static void test_selftest(void)
{
    const nncar_profile_t *p = nncar_profile_default();
    uint8_t exist[NNCAR_MAX_SENSORS];
    uint8_t valid[NNCAR_MAX_SENSORS];
    for (int i = 0; i < NNCAR_MAX_SENSORS; ++i) {
        exist[i] = 0u;
        valid[i] = 0u;
    }
    for (int i = 0; i < p->sensor_count; ++i) {
        exist[i] = 1u;
        valid[i] = 1u;
    }

    nncar_calib_selftest_input_t in = {
        12.0f, true, true, 30.0f, true, exist, valid,
    };
    CHECK(nncar_calib_selftest(p, &in) == 0u);

    /* Rail outside the accepted band, or missing entirely. */
    in.v_rail = 9.0f;
    CHECK(nncar_calib_selftest(p, &in) == NNCAR_CALIB_ST_RAIL);
    in.v_rail = 12.0f;
    in.rail_valid = false;
    CHECK(nncar_calib_selftest(p, &in) == NNCAR_CALIB_ST_RAIL);
    in.rail_valid = true;

    /* Over-temperature stops the self test. */
    in.max_temp_c = p->temp_stop_c + 1.0f;
    CHECK(nncar_calib_selftest(p, &in) == NNCAR_CALIB_ST_TEMP);
    in.max_temp_c = 30.0f;

    /* Too few usable gray channels. */
    for (int i = 0; i < p->sensor_count; ++i) {
        valid[i] = 0u;
    }
    CHECK(nncar_calib_selftest(p, &in) == NNCAR_CALIB_ST_GRAY_COVERAGE);
}

static void test_deadband(void)
{
    /* Monotone |pwm| ramp, true deadband 0.07, step 0.01. */
    nncar_calib_deadband_t st;
    nncar_calib_deadband_begin(&st, 0.01f, 0.02f);

    for (int i = 0; i <= 20; ++i) {
        const float pwm = 0.01f * (float)i;
        const float speed = (pwm >= 0.07f) ? 0.10f : 0.0f;
        /* Sign must not matter. */
        nncar_calib_deadband_push(&st, (i % 2 == 0) ? pwm : -pwm, speed);
    }

    float value = 0.0f;
    float quality = 0.0f;
    CHECK(nncar_calib_deadband_result(&st, &value, &quality));
    CHECK_NEAR(value, 0.065f, 1.0e-6f); /* midpoint of the last static / first moving */
    CHECK(quality > 0.9f);

    /* A wheel that never turns must not produce a number. */
    nncar_calib_deadband_begin(&st, 0.01f, 0.02f);
    for (int i = 0; i <= 20; ++i) {
        nncar_calib_deadband_push(&st, 0.01f * (float)i, 0.0f);
    }
    CHECK(!nncar_calib_deadband_result(&st, &value, &quality));
}

int main(void)
{
    test_record();
    test_verified();
    test_store();
    test_apply();
    test_selftest();
    test_deadband();
    return 0;
}
