/**
 * @file type_mask_attitude.h
 */

#ifndef TYPE_MASK_ATTITUDE_H
#define TYPE_MASK_ATTITUDE_H

#include <cstdint>

/**
 * @brief Represents bitmasks for different kind of attitude control within both Ardupilot or Ardupilot when setting a setpoint.
 */
class TypeMaskAttidute {
   public:
    static constexpr uint16_t IGNORE_ROLL_RATE = 1; // body_rate.x
    static constexpr uint16_t IGNORE_PITCH_RATE = 2; // body_rate.y
    static constexpr uint16_t IGNORE_YAW_RATE = 4; // body_rate.z
    static constexpr uint16_t IGNORE_THRUST = 64;
    static constexpr uint16_t IGNORE_ATTITUDE = 128; // orientation field

    static constexpr uint16_t ATTITUDE = //71U
        IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE | IGNORE_THRUST;
};

#endif // TYPE_MASK_ATTITUDE_H
