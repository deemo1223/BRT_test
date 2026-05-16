#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <array>
#include <cstdint>

// Hold the estimated 3D force and torque result.
struct ForceResult {
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
};

// Hold one sensor measurement in a compact form.
struct SensorSample {
    uint8_t device_id = 0;
    uint32_t raw_count = 0;
    double length_mm = 0.0;
    bool valid = false;
};

// Keep array types explicit for the four-sensor layout.
using LengthArray = std::array<double, 4>;
using RawCountArray = std::array<uint32_t, 4>;

#endif
