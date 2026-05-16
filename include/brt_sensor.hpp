#ifndef BRT_SENSOR_HPP
#define BRT_SENSOR_HPP

#include <cstdint>
#include <vector>

class CANInterface;

class BRTSensor {
public:
    // Store the fixed protocol and geometry parameters for one sensor.
    BRTSensor(uint8_t device_id, double resolution, double wheel_circumference_mm);

    // Send the BRT position read command for this device ID.
    bool requestPosition(CANInterface& can);
    // Validate a reply frame and update the cached measurement.
    bool parseResponse(uint32_t can_id, const std::vector<uint8_t>& data);
    // Set the current raw count as the software zero reference.
    void setSoftwareZero();
    // Return the latest displacement in millimeters.
    double getLengthMm() const;
    // Return the latest raw encoder count.
    uint32_t getRawCount() const;
    // Return the configured CAN device ID.
    uint8_t getId() const;
    // Report whether at least one valid reply has been parsed.
    bool hasValidData() const;

private:
    // Keep the sensor identity, scaling, and most recent measurement state.
    uint8_t device_id_;
    double resolution_;
    double wheel_circumference_mm_;
    uint32_t raw_count_;
    uint32_t zero_count_;
    double length_mm_;
    bool has_valid_data_;
    bool zero_initialized_;
};

#endif
