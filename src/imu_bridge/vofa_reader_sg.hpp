#ifndef VOFA_READER_SG_HPP
#define VOFA_READER_SG_HPP

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <algorithm>

#include <lcm/lcm-cpp.hpp>
#include "../exlcm/imu_cmd.hpp"

class vofa_reader
{
public:
    vofa_reader(const std::string &device, std::string _lcm_topic,
                speed_t baud,
                int timeout_ms);

    ~vofa_reader();
    bool open_port();
    void close_port();
    void run_once();

private:
    void decode_frame(const std::vector<uint8_t> &frame);
    void publish_frame(const std::vector<uint8_t> &frame);

    std::string device_;
    speed_t baud_;
    int timeout_ms_;
    int fd_;
    std::vector<uint8_t> buffer_;
    uint64_t frame_nr_;

    // LCM related
    lcm::LCM lcm_;
    std::string lcm_topic_;
};

#endif
