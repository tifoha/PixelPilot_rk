#pragma once
// Minimal ArduPilot WIND message (id 168): direction, speed, speed_z
// Not in the common dialect — added manually.

#define MAVLINK_MSG_ID_WIND 168

typedef struct __mavlink_wind_t {
    float direction; /*< [deg] Wind direction (0=N, clockwise, where wind comes FROM) */
    float speed;     /*< [m/s] Wind speed horizontal */
    float speed_z;   /*< [m/s] Wind speed vertical */
} mavlink_wind_t;

#define MAVLINK_MSG_ID_WIND_LEN 12
#define MAVLINK_MSG_ID_WIND_MIN_LEN 12

static inline void mavlink_msg_wind_decode(const mavlink_message_t *msg, mavlink_wind_t *wind) {
    wind->direction = _MAV_RETURN_float(msg, 0);
    wind->speed     = _MAV_RETURN_float(msg, 4);
    wind->speed_z   = _MAV_RETURN_float(msg, 8);
}
