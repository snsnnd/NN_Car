#ifndef NNCAR_CALIB_H
#define NNCAR_CALIB_H

#include "nncar/profile.h"

/*
 * Calibration record: the runtime side of the "verified" claim.
 *
 * Mechanical constants (wheel radius, track width, sensor geometry, encoder
 * counts) are NOT calibratable on board: they need an external ground truth,
 * so they stay hand-entered in the profile JSON and never appear here.
 *
 * Only parameters with an observable objective live in this record. Every
 * entry carries the source that produced it, because a bench number and a
 * real-car number are not interchangeable.
 */
#define NNCAR_CALIB_VERSION 1u
#define NNCAR_CALIB_MAX_ENTRIES 16
#define NNCAR_CALIB_ID_LEN 16

/* Below this quality an entry is rejected instead of overwriting the old one. */
#define NNCAR_CALIB_MIN_QUALITY 0.5f

typedef enum {
    NNCAR_CALIB_SRC_NONE = 0,
    NNCAR_CALIB_SRC_MANUAL = 1,   /* hand-entered, auto paths may not overwrite */
    NNCAR_CALIB_SRC_FACTORY = 2,
    NNCAR_CALIB_SRC_SELFTEST = 3,
    NNCAR_CALIB_SRC_IDENTIFY = 4,
    NNCAR_CALIB_SRC_ADAPT = 5,
    NNCAR_CALIB_SRC_BENCH = 6     /* simulation bench: never counts as verified */
} nncar_calib_source_t;

typedef enum {
    NNCAR_CALIB_MOTOR_DEADBAND = 0,
    NNCAR_CALIB_MOTOR_KFF = 1,
    NNCAR_CALIB_LINE_KAPPA_GAIN = 2,
    NNCAR_CALIB_PARAM_COUNT = 3
} nncar_calib_param_t;

typedef struct {
    uint8_t id;
    uint8_t source;
    float value;
    float quality; /* 0..1, from the fitting residual or signal-to-noise */
} nncar_calib_entry_t;

typedef struct {
    uint16_t version;
    uint16_t count;
    uint32_t stamp_us; /* caller stamps; newer record wins on load */
    char vehicle_id[NNCAR_CALIB_ID_LEN];
    nncar_calib_entry_t entries[NNCAR_CALIB_MAX_ENTRIES];
    uint32_t crc; /* covers every preceding byte */
} nncar_calib_record_t;

/*
 * Two-slot store (flash sectors on the MCU, files or RAM on the host).
 * save() writes one slot; load() reads both, ignores bad CRC/version and
 * keeps the newer one, so a torn write can never lose a good record.
 * slot = stamp_us & 1, so the caller must advance stamp_us between saves.
 */
typedef struct {
    void *ctx;
    bool (*read)(void *ctx, uint8_t slot, void *dst, uint32_t len);
    bool (*write)(void *ctx, uint8_t slot, const void *src, uint32_t len);
} nncar_calib_store_t;

void nncar_calib_init(nncar_calib_record_t *rec);

/*
 * Insert or update one entry. Rejected (returns false) for unknown ids,
 * non-finite values, quality below NNCAR_CALIB_MIN_QUALITY, or when the stored
 * entry is MANUAL and the caller is not MANUAL.
 */
bool nncar_calib_set(nncar_calib_record_t *rec, uint8_t id, float value,
                     float quality, uint8_t source);

/* Returns false when the id is absent. */
bool nncar_calib_get(const nncar_calib_record_t *rec, uint8_t id,
                     nncar_calib_entry_t *out);

/* Every parameter present, good quality, and not sourced from the bench. */
bool nncar_calib_verified(const nncar_calib_record_t *rec);

/* Bit i set = parameter i missing, low quality, or bench-sourced. */
uint32_t nncar_calib_pending_mask(const nncar_calib_record_t *rec);

uint32_t nncar_calib_crc(const nncar_calib_record_t *rec);

bool nncar_calib_save(const nncar_calib_store_t *store, nncar_calib_record_t *rec);
bool nncar_calib_load(const nncar_calib_store_t *store, nncar_calib_record_t *rec);

/* Overlay the calibrated parameters onto a mutable profile copy. */
void nncar_calib_apply(const nncar_calib_record_t *rec, nncar_profile_t *p);

/* ------------------------------------------------------------------ */
/* Power-on self test: never moves the car, never writes parameters.    */
/* ------------------------------------------------------------------ */

enum {
    NNCAR_CALIB_ST_RAIL = 1u << 0,
    NNCAR_CALIB_ST_TEMP = 1u << 1,
    NNCAR_CALIB_ST_GRAY_COVERAGE = 1u << 2
};

typedef struct {
    float v_rail;
    bool rail_exist;
    bool rail_valid;
    float max_temp_c;
    bool temp_valid;
    const uint8_t *gray_exist; /* NNCAR_MAX_SENSORS */
    const uint8_t *gray_valid;
} nncar_calib_selftest_input_t;

/* Returns the failure bitmask; 0 means the car may run. */
uint32_t nncar_calib_selftest(const nncar_profile_t *p,
                              const nncar_calib_selftest_input_t *in);

/* ------------------------------------------------------------------ */
/* Deadband identification: streaming, caller drives the motion.        */
/* ------------------------------------------------------------------ */

/*
 * The library never commands the motors. A calibration mode (or a bench)
 * applies a monotone |pwm| ramp, feeds each (pwm, measured speed) pair here,
 * and asks for the result at the end. This keeps every policy decision about
 * moving the car outside the library.
 */
typedef struct {
    float pwm_step;    /* |pwm| increment between consecutive samples */
    float motion_mps;  /* speed above which the wheel counts as turning */
    float last_static; /* largest |pwm| seen without motion, 0 = none yet */
    float first_moving; /* smallest |pwm| that produced motion */
    bool found_static;
    bool found_moving;
    uint32_t samples;
} nncar_calib_deadband_t;

void nncar_calib_deadband_begin(nncar_calib_deadband_t *st, float pwm_step,
                                float motion_mps);
void nncar_calib_deadband_push(nncar_calib_deadband_t *st, float pwm,
                               float speed_mps);

/* False when the ramp never showed motion (stuck wheel, no encoder, too short). */
bool nncar_calib_deadband_result(const nncar_calib_deadband_t *st, float *out,
                                 float *quality);

/* ------------------------------------------------------------------ */
/* Feedforward gain identification: streaming steady-state points.      */
/* ------------------------------------------------------------------ */

/*
 * The controller's feedforward is u_ff = kff * v_target / v_max_eff, so the
 * plant's own gain has to be divided out. Callers feed steady-state points as
 * (measured speed, deadband-compensated command, v_max_eff); the fit is a least
 * squares line through the origin and kff is its reciprocal slope.
 */
typedef struct {
    float sum_xx;
    float sum_xy;
    float sum_yy;
    uint32_t samples;
} nncar_calib_kff_t;

void nncar_calib_kff_begin(nncar_calib_kff_t *st);
void nncar_calib_kff_push(nncar_calib_kff_t *st, float speed_mps,
                          float u_compensated, float v_max_eff);

/* False when the points carry no usable slope. */
bool nncar_calib_kff_result(const nncar_calib_kff_t *st, float *out,
                            float *quality);

#endif
