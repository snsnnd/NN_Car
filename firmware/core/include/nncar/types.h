#ifndef NNCAR_TYPES_H
#define NNCAR_TYPES_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define NNCAR_MAX_SENSORS 16
#define NNCAR_PI 3.14159265358979323846f

typedef struct {
    float value;
    bool exist;
    bool valid;
    uint64_t timestamp_us;
} nncar_sensor_t;

typedef struct {
    float v;     /* m/s, forward positive */
    float omega; /* rad/s, counter-clockwise (left turn) positive */
} nncar_twist_t;

typedef struct {
    float left;
    float right;
} nncar_wheel_speed_t;

typedef struct {
    float left;
    float right;
} nncar_pwm_t;

typedef enum {
    NNCAR_DRIVE_LAYOUT_DIFF2 = 0,
    NNCAR_DRIVE_LAYOUT_SKID4 = 1
} nncar_drive_layout_t;

enum {
    NNCAR_WHEEL_FRONT_LEFT = 0,
    NNCAR_WHEEL_REAR_LEFT = 1,
    NNCAR_WHEEL_FRONT_RIGHT = 2,
    NNCAR_WHEEL_REAR_RIGHT = 3,
    NNCAR_WHEEL_COUNT = 4
};

typedef enum {
    NNCAR_STATUS_OK = 0,
    NNCAR_STATUS_STALE,
    NNCAR_STATUS_INVALID,
    NNCAR_STATUS_SATURATED,
    NNCAR_STATUS_FAULT
} nncar_status_t;

static inline float nncar_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline bool nncar_finite(float x)
{
    return isfinite(x) != 0;
}

static inline float nncar_sensor_age_s(const nncar_sensor_t *sensor, uint64_t now_us)
{
    if (now_us <= sensor->timestamp_us) {
        return 0.0f;
    }
    return (float)(now_us - sensor->timestamp_us) * 1.0e-6f;
}

static inline bool nncar_sensor_usable(const nncar_sensor_t *sensor, uint64_t now_us, float ttl_s)
{
    return sensor->exist && sensor->valid && nncar_finite(sensor->value) &&
           (nncar_sensor_age_s(sensor, now_us) <= ttl_s);
}

#endif
