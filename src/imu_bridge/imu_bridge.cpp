#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cmath> 

#include "vofa_reader.hpp"

static vofa_reader *g_reader = nullptr;

void handle_sigint(int)
{
    if (g_reader)
        g_reader->stop();
}

int main(int argc, char *argv[])
{
    // Device serial port
    const std::string port = "/dev/ttyACM0";
    // B9600, B115200, or B921600
    speed_t baud = B921600;
    // Loop frequency
    int hz = 200;
    // LCM topic
    std::string lcm_topic = "IMU_CMD";

    // Pass the recording stream by reference.
    vofa_reader reader(port, lcm_topic, baud, /*timeout_ms=*/500);
    g_reader = &reader;

    if (!reader.open_port())
    {
        return 1;
    }

    std::signal(SIGINT, handle_sigint);
    std::cout << "Reading VOFA data on " << port
              << " at baud " << ((baud == B115200) ? "115200" : (baud == B9600 ? "9600" : "921600"))
              << " at " << hz << " Hz. Press Ctrl-C to exit.\n";

    reader.start(hz);

    while (!reader.is_stoped())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }

    std::cout << "\nExiting..." << std::endl;

    return 0;
}

