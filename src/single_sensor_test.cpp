#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "brt_sensor.hpp"
#include "can_interface.hpp"

namespace {

// Keep the test loop running until Ctrl+C requests shutdown.
std::atomic<bool> g_running{true};

// Flip the loop flag when SIGINT is received.
void handleSignal(int signal_number) {
    if (signal_number == SIGINT) {
        g_running = false;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    // Allow the interface, bitrate, and sensor ID to be set from the CLI.
    const std::string interface_name = (argc > 1) ? argv[1] : "can0";
    const int bitrate = (argc > 2) ? std::atoi(argv[2]) : 500000;
    const uint8_t device_id = (argc > 3) ? static_cast<uint8_t>(std::strtoul(argv[3], nullptr, 0)) : 0x01;

    // Register Ctrl+C handling before starting the polling loop.
    std::signal(SIGINT, handleSignal);

    try {
        // Create the CAN interface and one sensor instance for direct testing.
        CANInterface can(interface_name, bitrate);
        BRTSensor sensor(device_id, 4096.0, 60.0);

        // Use short retries to wait for the matching sensor response.
        constexpr int read_timeout_ms = 50;
        constexpr int max_attempts = 5;
        constexpr auto loop_period = std::chrono::milliseconds(20);

        while (g_running) {
            // Send one read request at the start of each polling cycle.
            if (!sensor.requestPosition(can)) {
                std::cerr << "Failed to send read command to sensor ID 0x"
                          << std::hex << static_cast<int>(sensor.getId()) << std::dec << std::endl;
                std::this_thread::sleep_for(loop_period);
                continue;
            }

            // Retry reads briefly until the matching response arrives.
            bool got_response = false;
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                uint32_t can_id = 0;
                std::vector<uint8_t> data;

                if (!can.readFrame(can_id, data, read_timeout_ms)) {
                    continue;
                }

                // Ignore unrelated frames and stop only on a valid sensor reply.
                if (sensor.parseResponse(can_id, data)) {
                    got_response = true;
                    break;
                }
            }

            // Print the decoded reading or note that this poll cycle failed.
            if (got_response) {
                std::cout << std::fixed << std::setprecision(1)
                          << "sensor=0x" << std::hex << static_cast<int>(sensor.getId()) << std::dec
                          << " raw_count=" << sensor.getRawCount()
                          << " length_mm=" << sensor.getLengthMm()
                          << std::endl;
            } else {
                std::cerr << "No valid response from sensor ID 0x"
                          << std::hex << static_cast<int>(sensor.getId()) << std::dec << std::endl;
            }

            // apply delay 
            std::this_thread::sleep_for(loop_period);
        }
    } catch (const std::exception& exception) {
        // Surface startup or runtime failures with a single fatal message.
        std::cerr << "Fatal error: " << exception.what() << std::endl;
        return 1;
    }

    // Report normal shutdown after Ctrl+C.
    std::cout << "Exiting." << std::endl;
    return 0;
}
