#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <vector>

class CANInterface {
public:
    // Construct the interface manager and initialize the CAN device.
    CANInterface(const std::string& interface_name, int bitrate);
    // Release the socket and shut the interface down on exit.
    ~CANInterface();

    CANInterface(const CANInterface&) = delete;
    CANInterface& operator=(const CANInterface&) = delete;

    // Send one standard CAN frame on the bound interface.
    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data);
    // Read one CAN frame with a bounded wait time.
    bool readFrame(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms);

private:
    // Bring the Linux CAN network interface into the requested state.
    bool configureInterface() const;
    // Bring the Linux CAN network interface down.
    bool bringInterfaceDown() const;
    // Create and bind the raw SocketCAN socket.
    bool openSocket();
    // Apply a receive timeout so reads do not block forever.
    bool setReadTimeout(int timeout_ms);

    // Store the target interface name, bitrate, and socket handle.
    std::string interface_name_;
    int bitrate_;
    int socket_fd_;
};

#endif
