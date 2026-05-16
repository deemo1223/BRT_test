#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "brt_sensor.hpp"
#include "can_interface.hpp"
#include "force_estimator.hpp"
#include "sensor_array.hpp"

namespace {

// Keep the main loop running until Ctrl+C requests shutdown.
std::atomic<bool> g_running{true};

// Flip the loop flag when SIGINT is received.
void handleSignal(int signal_number) {
    if (signal_number == SIGINT) {
        g_running = false;
    }
}

// Print the current sensor state and placeholder wrench in one line.
void printStatus(const RawCountArray& counts,
                 const LengthArray& lengths,
                 const ForceResult& wrench,
                 bool update_ok) {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << (update_ok ? "[OK]  " : "[WARN] ");
    std::cout << "counts=["
              << counts[0] << ", "
              << counts[1] << ", "
              << counts[2] << ", "
              << counts[3] << "] ";
    std::cout << "lengths_mm=["
              << lengths[0] << ", "
              << lengths[1] << ", "
              << lengths[2] << ", "
              << lengths[3] << "] ";
    std::cout << "wrench=["
              << wrench.fx << ", "
              << wrench.fy << ", "
              << wrench.fz << ", "
              << wrench.tx << ", "
              << wrench.ty << ", "
              << wrench.tz << "]"
              << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Allow the interface name and bitrate to be overridden from the CLI.
    const std::string interface_name = (argc > 1) ? argv[1] : "can0";
    const int bitrate = (argc > 2) ? std::atoi(argv[2]) : 500000;

    // Register Ctrl+C handling before starting the polling loop.
    std::signal(SIGINT, handleSignal);

    try {
        // Open and configure the CAN interface before creating sensors.
        CANInterface can(interface_name, bitrate);

        // Create the four sensors in their intended CAN ID order.
        std::vector<BRTSensor> sensors;
        sensors.emplace_back(0x01, 4096.0, 60.0);
        sensors.emplace_back(0x02, 4096.0, 60.0);
        sensors.emplace_back(0x03, 4096.0, 60.0);
        sensors.emplace_back(0x04, 4096.0, 60.0);

        // Assemble the polling and estimation objects used by the loop.
        SensorArray sensor_array(std::move(sensors));
        ForceEstimator force_estimator;

        // Run at a modest rate that is easy on the bus and console output.
        constexpr auto loop_period = std::chrono::milliseconds(20);

        while (g_running) {
            // Poll the sensors, estimate the wrench, and print the result.
            const bool update_ok = sensor_array.update(can);
            const RawCountArray counts = sensor_array.getRawCounts();
            const LengthArray lengths = sensor_array.getLengthsMm();
            const ForceResult wrench = force_estimator.estimate(lengths);

            printStatus(counts, lengths, wrench, update_ok);
            // Sleep to keep the polling loop near the target rate.
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
