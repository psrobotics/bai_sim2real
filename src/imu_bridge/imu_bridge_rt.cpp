#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <csignal>
#include <chrono>

#include "vofa_reader_sg.hpp"
#include "../utils/rt_loop.hpp" // Include the real-time loop header

// Global pointer to the real-time loop for the signal handler
static real_time_loop::loop* g_rt_loop = nullptr;

// The signal handler now shuts down our real-time loop gracefully
void handle_sigint(int)
{
    if (g_rt_loop) {
        g_rt_loop->shutdown();
    }
}

int main(int argc, char *argv[])
{

    const std::string port = "/dev/ttyACM0";
    speed_t baud = B921600;
    int hz = 200;
    std::string lcm_topic = "IMU_CMD";

    vofa_reader reader(port, lcm_topic, baud,
                       /*timeout_ms=*/500);

    if (!reader.open_port()) {
        return 1;
    }

    // The period for the real-time loop
    real_time_loop::period_t loop_period = std::chrono::duration<double>(1.0 / hz);
    // Bind loop fcn
    real_time_loop::loop_func vofa_loop(
        "vofa_reader_loop",
        loop_period,
        [&reader]() {
            reader.run_once();
        }
    );

    // --- Start Execution ---
    g_rt_loop = &vofa_loop;
    std::signal(SIGINT, handle_sigint);

    std::cout << "Reading VOFA data on " << port
              << " at baud " << ((baud == B115200) ? "115200" : (baud == B9600 ? "9600" : "921600"))
              << " at " << hz << " Hz. Press Ctrl-C to exit.\n";

    vofa_loop.start();

    // The main thread will wait here until the loop is shut down by Ctrl-C.
    // The is_running() method is assumed to be in your rt_loop.hpp
    while (vofa_loop.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nExiting..." << std::endl;

    return 0;
}
