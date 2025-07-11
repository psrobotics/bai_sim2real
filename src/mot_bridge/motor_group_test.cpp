// main.cpp for dual CAN bus control

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono> // For high-precision timing

#include <rerun.hpp>

#include "motor_driver/motor_group.hpp"

int main(int argc, char **argv)
{
    // --- Rerun Initialization ---
    rerun::RecordingStream rec("rerun_dual_can_vis");
    rec.spawn().exit_on_failure();

    // --- Bus 1 Setup (can0) ---
    const std::string can0_if = "can0";
    std::vector<int> motor_ids_bus0 = {1, 2, 3}; // Motors on the first bus
    const double default_kp_bus0 = 3.5;
    const double default_kd_bus0 = 0.1;
    std::vector<double> kps_bus0(motor_ids_bus0.size(), default_kp_bus0);
    std::vector<double> kds_bus0(motor_ids_bus0.size(), default_kd_bus0);
    std::vector<bool> all_on_bus0(motor_ids_bus0.size(), true);

    // Instantiate and init motor group for can0
    motor_group mg0(motor_ids_bus0, can0_if, kps_bus0, kds_bus0, motor_driver::MotorType::AK80_6_V1p1);
    std::cout << "Initializing motor group on " << can0_if << std::endl;
    mg0.init();


    // --- Bus 2 Setup (can1) ---
    const std::string can1_if = "can1";
    std::vector<int> motor_ids_bus1 = {4, 5, 6}; // Motors on the second bus
    const double default_kp_bus1 = 3.5;
    const double default_kd_bus1 = 0.1;
    std::vector<double> kps_bus1(motor_ids_bus1.size(), default_kp_bus1);
    std::vector<double> kds_bus1(motor_ids_bus1.size(), default_kd_bus1);
    std::vector<bool> all_on_bus1(motor_ids_bus1.size(), true);

    // Instantiate and init motor group for can1
    motor_group mg1(motor_ids_bus1, can1_if, kps_bus1, kds_bus1, motor_driver::MotorType::AK80_6_V1p1);
    std::cout << "Initializing motor group on " << can1_if << std::endl;
    mg1.init();


    // --- Enable and Zero Both Groups ---
    std::cout << "Enabling motors... g0" << std::endl;
    mg0.enable(all_on_bus0);
    std::cout << "Enabling motors... g1" << std::endl;
    mg1.enable(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Setting new zero..." << std::endl;
    mg0.zero(all_on_bus0);
    mg1.zero(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Starting sine wave test on both buses..." << std::endl;

    // --- Sine-wave test ---
    double phase = 0.0;
    const double freq = 4.0; // Hz
    const double dt = 0.01; // loop period (10ms -> 100Hz)
    const int N = 10000;   // number of steps
    const double p2p = M_PI;

    // Convert the double dt (in seconds) to a chrono duration object that is compatible with the clock.
    auto dt_duration = std::chrono::microseconds(static_cast<long long>(dt * 1e6));
    auto next_cycle_time = std::chrono::steady_clock::now();

    for (int i = 0; i < N; ++i)
    {
        // Increment the next cycle time by the fixed duration
        next_cycle_time += dt_duration;
        rec.set_time_sequence("step", i);

        phase += 2 * M_PI * freq * dt;

        // --- Command Generation for Bus 0 ---
        std::vector<double> targets0;
        targets0.reserve(motor_ids_bus0.size());
        for (size_t j = 0; j < motor_ids_bus0.size(); ++j)
        {
            targets0.push_back(std::sin(phase + j * 0.2) * p2p);
        }
        std::vector<double> tau_ff0(motor_ids_bus0.size(), 0.0);
        mg0.set_pos_ctr(targets0, tau_ff0);

        // --- Command Generation for Bus 1 ---
        std::vector<double> targets1;
        targets1.reserve(motor_ids_bus1.size());
        for (size_t j = 0; j < motor_ids_bus1.size(); ++j)
        {
            // Use a different phase shift for the second group for visual distinction
            targets1.push_back(std::sin(phase + j * 0.2) * p2p);
        }
        std::vector<double> tau_ff1(motor_ids_bus1.size(), 0.0);
        mg1.set_pos_ctr(targets1, tau_ff1);


        // --- Logging to Rerun (optional) ---
        if (i % 10 == 0)
        {
            // Log data from bus 0
            auto qs0 = mg0.read_q();
            for (size_t j = 0; j < qs0.size(); ++j)
            {
                std::string motor_id_str = "motor_" + std::to_string(motor_ids_bus0[j]);
                rec.log("q/" + motor_id_str, rerun::Scalar(qs0[j]));
            }

            // Log data from bus 1
            auto qs1 = mg1.read_q();
            for (size_t j = 0; j < qs1.size(); ++j)
            {
                std::string motor_id_str = "motor_" + std::to_string(motor_ids_bus1[j]);
                rec.log("q/" + motor_id_str, rerun::Scalar(qs1[j]));
            }
        }

        std::this_thread::sleep_until(next_cycle_time);
    }

    // --- Stop and Disable Both Groups ---
    std::cout << "Stopping motors..." << std::endl;
    std::vector<double> zero_pos0(motor_ids_bus0.size(), 0.0);
    std::vector<double> zero_tau0(motor_ids_bus0.size(), 0.0);
    mg0.set_pos_ctr(zero_pos0, zero_tau0);

    std::vector<double> zero_pos1(motor_ids_bus1.size(), 0.0);
    std::vector<double> zero_tau1(motor_ids_bus1.size(), 0.0);
    mg1.set_pos_ctr(zero_pos1, zero_tau1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Disabling motors..." << std::endl;
    mg0.disable(all_on_bus0);
    mg1.disable(all_on_bus1);

    return 0;
}
