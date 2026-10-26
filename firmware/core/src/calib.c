#include "nncar/calib.h"

#include <stddef.h>
#include <string.h>

/* Plausibility bounds applied when overlaying a record onto a profile. */
typedef struct {
    float min;
    float max;
} calib_bound_t;

static const calib_bound_t k_bounds[NNCAR_CALIB_PARAM_COUNT] = {
    {0.0f, 0.30f},  /* MOTOR_DEADBAND */
    {0.0f, 2.00f},  /* MOTOR_KFF */
    {0.5f, 4.00f},  /* LINE_KAPPA_GAIN */
};

static uint32_t crc32_buf(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }

    return ~crc;
}

uint32_t nncar_calib_crc(const nncar_calib_record_t *rec)
{
    return crc32_buf((const uint8_t *)rec,
                     (uint32_t)offsetof(nncar_calib_record_t, crc));
}

void nncar_calib_init(nncar_calib_record_t *rec)
{
    memset(rec, 0, sizeof(*rec));
    rec->version = NNCAR_CALIB_VERSION;
}

static nncar_calib_entry_t *find_entry(nncar_calib_record_t *rec, uint8_t id)
{
    for (uint16_t i = 0; i < rec->count; ++i) {
        if (rec->entries[i].id == id) {
            return &rec->entries[i];
        }
    }

    return NULL;
}

bool nncar_calib_set(nncar_calib_record_t *rec, uint8_t id, float value,
                     float quality, uint8_t source)
{
    if (id >= NNCAR_CALIB_PARAM_COUNT) {
        return false;
    }
    if (!nncar_finite(value) || !nncar_finite(quality)) {
        return false;
    }
    if (quality < NNCAR_CALIB_MIN_QUALITY || quality > 1.0f) {
        return false;
    }
    if (value < k_bounds[id].min || value > k_bounds[id].max) {
        return false;
    }
    if (source == NNCAR_CALIB_SRC_NONE) {
        return false;
    }

    nncar_calib_entry_t *slot = find_entry(rec, id);

    if (slot == NULL) {
        if (rec->count >= NNCAR_CALIB_MAX_ENTRIES) {
            return false;
        }
        slot = &rec->entries[rec->count++];
    } else if (slot->source == NNCAR_CALIB_SRC_MANUAL &&
               source != NNCAR_CALIB_SRC_MANUAL) {
        /* Hand-entered values win over anything automatic. */
        return false;
    }

    slot->id = id;
    slot->source = source;
    slot->value = value;
    slot->quality = quality;
    rec->crc = nncar_calib_crc(rec);

    return true;
}

bool nncar_calib_get(const nncar_calib_record_t *rec, uint8_t id,
                     nncar_calib_entry_t *out)
{
    if (id >= NNCAR_CALIB_PARAM_COUNT) {
        return false;
    }

    for (uint16_t i = 0; i < rec->count; ++i) {
        if (rec->entries[i].id == id) {
            if (out != NULL) {
                *out = rec->entries[i];
            }
            return true;
        }
    }

    return false;
}

uint32_t nncar_calib_pending_mask(const nncar_calib_record_t *rec)
{
    uint32_t pending = 0u;

    for (uint8_t id = 0; id < NNCAR_CALIB_PARAM_COUNT; ++id) {
        nncar_calib_entry_t e;
        if (!nncar_calib_get(rec, id, &e) ||
            e.quality < NNCAR_CALIB_MIN_QUALITY ||
            e.source == NNCAR_CALIB_SRC_NONE ||
            e.source == NNCAR_CALIB_SRC_BENCH) {
            pending |= (1u << id);
        }
    }

    return pending;
}

bool nncar_calib_verified(const nncar_calib_record_t *rec)
{
    return nncar_calib_pending_mask(rec) == 0u;
}

bool nncar_calib_save(const nncar_calib_store_t *store, nncar_calib_record_t *rec)
{
    if (store->write == NULL) {
        return false;
    }

    rec->crc = nncar_calib_crc(rec);

    /* Four slots would be overkill; alternate so the previous record survives. */
    const uint8_t slot = (uint8_t)(rec->stamp_us & 1u);

    return store->write(store->ctx, slot, rec, (uint32_t)sizeof(*rec));
}

static bool read_valid(const nncar_calib_store_t *store, uint8_t slot,
                       nncar_calib_record_t *dst)
{
    if (store->read == NULL ||
        !store->read(store->ctx, slot, dst, (uint32_t)sizeof(*dst))) {
        return false;
    }
    if (dst->version != NNCAR_CALIB_VERSION || dst->count > NNCAR_CALIB_MAX_ENTRIES) {
        return false;
    }

    return dst->crc == nncar_calib_crc(dst);
}

bool nncar_calib_load(const nncar_calib_store_t *store, nncar_calib_record_t *rec)
{
    nncar_calib_record_t a;
    nncar_calib_record_t b;
    const bool ok_a = read_valid(store, 0u, &a);
    const bool ok_b = read_valid(store, 1u, &b);

    if (ok_a && ok_b) {
        *rec = (b.stamp_us >= a.stamp_us) ? b : a;
        return true;
    }
    if (ok_b) {
        *rec = b;
        return true;
    }
    if (ok_a) {
        *rec = a;
        return true;
    }

    return false;
}

void nncar_calib_apply(const nncar_calib_record_t *rec, nncar_profile_t *p)
{
    for (uint8_t i = 0; i < NNCAR_CALIB_PARAM_COUNT; ++i) {
        nncar_calib_entry_t e;
        if (!nncar_calib_get(rec, i, &e)) {
            continue;
        }
        /* Records come from flash: re-check the value here, not only on set. */
        if (e.quality < NNCAR_CALIB_MIN_QUALITY || !nncar_finite(e.value) ||
            e.value < k_bounds[i].min || e.value > k_bounds[i].max ||
            e.source == NNCAR_CALIB_SRC_NONE || e.source == NNCAR_CALIB_SRC_BENCH) {
            continue;
        }

        switch ((nncar_calib_param_t)i) {
        case NNCAR_CALIB_MOTOR_DEADBAND:
            p->motor_deadband = e.value;
            break;
        case NNCAR_CALIB_MOTOR_KFF:
            p->motor_kff = e.value;
            break;
        case NNCAR_CALIB_LINE_KAPPA_GAIN:
            p->line_kappa_gain = e.value;
            break;
        default:
            break;
        }
    }
}

uint32_t nncar_calib_selftest(const nncar_profile_t *p,
                              const nncar_calib_selftest_input_t *in)
{
    uint32_t failures = 0u;

    if (!in->rail_exist || !in->rail_valid || !nncar_finite(in->v_rail) ||
        in->v_rail < p->rail_min_v || in->v_rail > p->rail_max_v) {
        failures |= NNCAR_CALIB_ST_RAIL;
    }

    if (!in->temp_valid || !nncar_finite(in->max_temp_c) ||
        in->max_temp_c > p->temp_stop_c) {
        failures |= NNCAR_CALIB_ST_TEMP;
    }

    int usable = 0;
    for (int i = 0; i < p->sensor_count && i < NNCAR_MAX_SENSORS; ++i) {
        if (in->gray_exist[i] != 0u && in->gray_valid[i] != 0u) {
            ++usable;
        }
    }
    if (usable < p->conf_valid_min) {
        failures |= NNCAR_CALIB_ST_GRAY_COVERAGE;
    }

    return failures;
}

void nncar_calib_deadband_begin(nncar_calib_deadband_t *st, float pwm_step,
                                float motion_mps)
{
    memset(st, 0, sizeof(*st));
    st->pwm_step = fabsf(pwm_step);
    st->motion_mps = fabsf(motion_mps);
}

void nncar_calib_deadband_push(nncar_calib_deadband_t *st, float pwm,
                               float speed_mps)
{
    const float mag = fabsf(pwm);

    ++st->samples;

    if (fabsf(speed_mps) > st->motion_mps) {
        if (!st->found_moving || mag < st->first_moving) {
            st->first_moving = mag;
            st->found_moving = true;
        }
    } else if (!st->found_moving && (!st->found_static || mag > st->last_static)) {
        st->last_static = mag;
        st->found_static = true;
    }
}

bool nncar_calib_deadband_result(const nncar_calib_deadband_t *st, float *out,
                                 float *quality)
{
    if (!st->found_moving) {
        return false;
    }

    const float hi = st->first_moving;
    const float lo = st->found_static ? st->last_static : 0.0f;
    const float value = 0.5f * (lo + hi);

    /* A narrow bracket means the ramp resolved the knee; a wide one means not. */
    float q = 1.0f;
    if (st->pwm_step > 0.0f) {
        const float slack = (hi - lo) / st->pwm_step;
        q = 1.0f - 0.25f * fmaxf(0.0f, slack - 1.0f);
    }
    q = nncar_clampf(q, 0.0f, 1.0f);

    if (out != NULL) {
        *out = value;
    }
    if (quality != NULL) {
        *quality = q;
    }

    return true;
}
