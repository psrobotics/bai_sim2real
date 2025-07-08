#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <cmath>

#include <lcm/lcm-cpp.hpp>
#include "../exlcm/imu_cmd.hpp"

// Low-level serial reader to interact with the STM32 imu
class vofa_reader
{
public:
    vofa_reader(const std::string &device,
                std::string _lcm_topic,
                speed_t baud,
                int timeout_ms);

    ~vofa_reader();

    bool open_port();
    void start(int hz);
    void stop();
    bool is_stoped();

private:
    void close_port();
    void read_loop_hz(int hz);
    void decode_frame(const std::vector<uint8_t> &frame);
    void publish_frame(const std::vector<uint8_t> &frame);

    std::string device_;
    speed_t baud_;
    int timeout_ms_;
    int fd_;
    bool stop_requested_;
    // Frame counter
    long long frame_nr_;
    std::thread read_thread_;
    std::vector<uint8_t> buffer_;

    // LCM obj for pushing imu message
    lcm::LCM lcm_;
    std::string lcm_topic_;
};
