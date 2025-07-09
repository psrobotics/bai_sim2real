#include "motor_driver/motor_group.hpp"
#include <stdexcept>

motor_group::motor_group(const std::vector<int> &_motor_ids,
                         const std::string &_usb2can_id,
                         const std::vector<double> &_kps,
                         const std::vector<double> &_kds,
                         motor_driver::MotorType _motor_type)
    : motor_ids(_motor_ids),
      usb2can_id(_usb2can_id),
      kps(_kps),
      kds(_kds),
      motor_type(_motor_type),
      driver(nullptr)
{
    if (kps.size() != motor_ids.size() || kds.size() != motor_ids.size())
    {
        throw std::invalid_argument("kps and kds vectors must match motor_ids length");
    }
}

int motor_group::init()
{
    driver = new motor_driver::MotorDriver(
        motor_ids,
        usb2can_id.c_str(),
        motor_type);
    return 0;
}

int motor_group::enable(std::vector<bool> &mask)
{
    if (mask.size() != motor_ids.size())
        return -1;

    std::vector<int> to_enable;
    for (std::size_t i = 0; i < motor_ids.size(); ++i)
    {
        if (mask[i])
            to_enable.push_back(motor_ids[i]);
    }
    state_n = driver->enableMotor(to_enable);

    return 0;
}

int motor_group::disable(std::vector<bool> &mask)
{
    if (mask.size() != motor_ids.size())
        return -1;

    std::vector<int> to_disable;
    for (std::size_t i = 0; i < motor_ids.size(); ++i)
    {
        if (mask[i])
            to_disable.push_back(motor_ids[i]);
    }
    state_n = driver->disableMotor(to_disable);
    return 0;
}

int motor_group::zero(std::vector<bool> &mask)
{
    if (mask.size() != motor_ids.size())
        return -1;

    std::vector<int> to_zero;
    for (std::size_t i = 0; i < motor_ids.size(); ++i)
    {
        if (mask[i])
            to_zero.push_back(motor_ids[i]);
    }
    state_n = driver->setZeroPosition(to_zero);
    return 0;
}

// Write desired control set
int motor_group::set_pos_ctr(const std::vector<double> &_j_pos,
                             const std::vector<double> &_tau_ff)
{
    if (_j_pos.size() != motor_ids.size() ||
        _tau_ff.size() != motor_ids.size())
    {
        return -1;
    }
    std::map<int, motor_driver::motorCommand> cmd_map;
    for (std::size_t i = 0; i < motor_ids.size(); ++i)
    {
        motor_driver::motorCommand cmd;
        cmd.p_des = _j_pos[i];
        cmd.v_des = 0.0;
        cmd.tau_ff = _tau_ff[i];
        cmd.kp = kps[i];
        cmd.kd = kds[i];

        cmd_map[motor_ids[i]] = cmd;
    }
    // Update downstream state to local state array
    state_n = driver->sendRadCommand(cmd_map);

    return 0;
}

int motor_group::set_kp(const std::vector<double> &_kps)
{
    if (_kps.size() != motor_ids.size())
        return -1;

    kps = _kps;
    return 0;
}

int motor_group::set_kd(const std::vector<double> &_kds)
{
    if (_kds.size() != motor_ids.size())
        return -1;

    kds = _kds;
    return 0;
}

// Get states from local state_n, no local updates for these calls
std::vector<double> motor_group::read_q()
{
    std::vector<double> qs;
    qs.reserve(motor_ids.size());
    for (int id : motor_ids)
        qs.push_back(state_n[id].position);

    return qs;
}

std::vector<double> motor_group::read_dq()
{
    std::vector<double> dqs;
    dqs.reserve(motor_ids.size());
    for (int id : motor_ids)
        dqs.push_back(state_n[id].velocity);

    return dqs;
}

std::vector<double> motor_group::read_tau()
{
    std::vector<double> taus;
    taus.reserve(motor_ids.size());
    for (int id : motor_ids)
        taus.push_back(state_n[id].torque);

    return taus;
}

motor_group::~motor_group()
{
    delete driver;
}
