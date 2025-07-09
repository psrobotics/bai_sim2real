#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include <vector>
#include <map>
#include <string>
#include "MotorDriver.hpp"

// Mujoco style wrapper for joint-group low-level motor control
class motor_group
{
public:
    motor_group(const std::vector<int> &_motor_ids,
                const std::string &_usd2can_id,
                const std::vector<double> &_kps,
                const std::vector<double> &_kds,
                motor_driver::MotorType motor_type = motor_driver::MotorType::AK80_6_V1p1);
    ~motor_group();

    int init();

    int enable(std::vector<bool> &_m_id);
    int disable(std::vector<bool> &_m_id);

    int zero(std::vector<bool> &_m_id);

    int set_pos_ctr(const std::vector<double> &_j_pos,
                    const std::vector<double> &_tau_ff);

    int set_kp(const std::vector<double> &_kp);
    int set_kd(const std::vector<double> &_kd);

    std::vector<double> read_q();
    std::vector<double> read_dq();
    std::vector<double> read_tau();

private:
    std::vector<int> motor_ids;
    std::string usb2can_id;
    std::vector<double> kps;
    std::vector<double> kds;
    motor_driver::MotorType motor_type;
    motor_driver::MotorDriver *driver;

    // last-know states, updated on each call
    std::map<int, motor_driver::motorState> state_n;
};

#endif