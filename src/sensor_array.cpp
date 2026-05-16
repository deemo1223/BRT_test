#include "sensor_array.hpp"

#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "can_interface.hpp"

// Keep exactly four sensors in a fixed polling order.
SensorArray::SensorArray(std::vector<BRTSensor> sensors) : sensors_(std::move(sensors)) {
    if (sensors_.size() != 4) {
        throw std::runtime_error("SensorArray currently requires exactly 4 sensors.");
    }
}

// Poll each sensor sequentially and keep going even if one fails.
bool SensorArray::update(CANInterface& can) {
    bool all_ok = true;

    for (BRTSensor& sensor : sensors_) {
        // Skip to the next sensor if the request itself could not be sent.
        if (!sensor.requestPosition(can)) {
            std::cerr << "Failed to request position from sensor ID 0x"
                      << std::hex << static_cast<int>(sensor.getId()) << std::dec << std::endl;
            all_ok = false;
            continue;
        }

        // Try a few reads so unrelated or delayed frames do not fail immediately.
        bool matched_response = false;
        for (int attempt = 0; attempt < 5; ++attempt) {
            uint32_t can_id = 0;
            std::vector<uint8_t> data;
            if (!can.readFrame(can_id, data, 20)) {
                continue;
            }

            // Accept only the reply that matches the current sensor.
            if (sensor.parseResponse(can_id, data)) {
                matched_response = true;
                break;
            }
        }

        // Report missing data but continue polling the rest of the array.
        if (!matched_response) {
            std::cerr << "No valid response from sensor ID 0x"
                      << std::hex << static_cast<int>(sensor.getId()) << std::dec << std::endl;
            all_ok = false;
        }
    }

    return all_ok;
}

// Collect the latest four displacement values into a fixed-size array.
std::array<double, 4> SensorArray::getLengthsMm() const {
    std::array<double, 4> lengths = {0.0, 0.0, 0.0, 0.0};
    for (std::size_t index = 0; index < sensors_.size(); ++index) {
        lengths[index] = sensors_[index].getLengthMm();
    }
    return lengths;
}

// Collect the latest four raw counts into a fixed-size array.
std::array<uint32_t, 4> SensorArray::getRawCounts() const {
    std::array<uint32_t, 4> counts = {0U, 0U, 0U, 0U};
    for (std::size_t index = 0; index < sensors_.size(); ++index) {
        counts[index] = sensors_[index].getRawCount();
    }
    return counts;
}
