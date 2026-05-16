#ifndef SENSOR_ARRAY_HPP
#define SENSOR_ARRAY_HPP

#include <array>
#include <vector>

#include "brt_sensor.hpp"
#include "data_types.hpp"

class CANInterface;

class SensorArray {
public:
    // Take ownership of the fixed four-sensor set.
    explicit SensorArray(std::vector<BRTSensor> sensors);

    // Poll every sensor once and update cached values.
    bool update(CANInterface& can);
    // Return the latest displacement values for all sensors.
    std::array<double, 4> getLengthsMm() const;
    // Return the latest raw counts for all sensors.
    std::array<uint32_t, 4> getRawCounts() const;

private:
    // Store the managed sensors in polling order.
    std::vector<BRTSensor> sensors_;
};

#endif
