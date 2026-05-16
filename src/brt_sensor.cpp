#include "brt_sensor.hpp"

#include <cmath>
#include <vector>

#include "can_interface.hpp"

BRTSensor::BRTSensor(uint8_t device_id, double resolution, double wheel_circumference_mm)
    : device_id_(device_id),
      resolution_(resolution),
      wheel_circumference_mm_(wheel_circumference_mm),
      raw_count_(0),
      zero_count_(0),
      length_mm_(0.0),
      has_valid_data_(false),
      zero_initialized_(false) {}

// Send the fixed BRT position query command for this sensor.
bool BRTSensor::requestPosition(CANInterface& can) {
    const std::vector<uint8_t> command = {0x04, device_id_, 0x01, 0x00};
    return can.sendFrame(device_id_, command);
}

// Accept only matching BRT reply frames and decode the little-endian count.
bool BRTSensor::parseResponse(uint32_t can_id, const std::vector<uint8_t>& data) {
    // Ignore frames that do not match this sensor and command format.
    if (can_id != device_id_ || data.size() < 7) {
        return false;
    }

    if (data[0] != 0x07 || data[1] != device_id_ || data[2] != 0x01) {
        return false;
    }

    // Reassemble the 32-bit encoder count from the reply payload.
    raw_count_ = static_cast<uint32_t>(data[3]) |
                 (static_cast<uint32_t>(data[4]) << 8U) |
                 (static_cast<uint32_t>(data[5]) << 16U) |
                 (static_cast<uint32_t>(data[6]) << 24U);

    // Preserve the current behavior where the first valid sample becomes zero.
    if (!zero_initialized_) {
        setSoftwareZero();
    }

    // Convert counts relative to zero into rounded millimeters.
    const double delta_count =
        static_cast<double>(static_cast<int64_t>(raw_count_) - static_cast<int64_t>(zero_count_));
    const double length = delta_count * wheel_circumference_mm_ / resolution_;

    length_mm_ = std::round(length * 10.0) / 10.0;
    // remove error length data
    length_mm_ = (length_mm_ >= 350.0 || length_mm_ < 0.0) ? length_mm_ : 0.0;
    
    has_valid_data_ = true;
    return true;
}

// Store the current reading as the reference point for future displacement.
void BRTSensor::setSoftwareZero() {
    zero_count_ = raw_count_;
    zero_initialized_ = true;
    length_mm_ = 0.0;
}

// Return the latest computed displacement value.
double BRTSensor::getLengthMm() const {
    return length_mm_;
}

// Return the latest raw encoder count without further processing.
uint32_t BRTSensor::getRawCount() const {
    return raw_count_;
}

// Return the configured CAN device ID for this sensor.
uint8_t BRTSensor::getId() const {
    return device_id_;
}

// Report whether at least one valid measurement is available.
bool BRTSensor::hasValidData() const {
    return has_valid_data_;
}
