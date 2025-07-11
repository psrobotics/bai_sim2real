// main.cpp for multi-threaded, dual CAN bus control

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>

#include <rerun.hpp>
#include "motor_driver/motor_group.hpp"

// --- Shared data structures for thread communication ---
struct MotorGroupControl {
    std::mutex mtx;
    std::vector<double> targets;
    std::vector<double> tau_ff;
};

// --- Thread function for controlling a motor group ---
void motor_control_thread(
    motor_group& mg,
    MotorGroupControl& control_data,
    std::atomic<bool>& stop_token,
    std::vector<bool>& motor_mask)
{
    while (!stop_token) {
        std::vector<double> current_targets;
        std::vector<double> current_tau_ff;

        // Lock the mutex to safely copy the command data
        {
            std::lock_guard<std::mutex> lock(control_data.mtx);
            current_targets = control_data.targets;
            current_tau_ff = control_data.tau_ff;
        }

        // Send the command if there are targets
        if (!current_targets.empty()) {
            mg.set_pos_ctr(current_targets, current_tau_ff);
        }
        
        // A small sleep to prevent this thread from busy-looping at 100% CPU
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // When stopping, send a zero command and disable motors
    std::cout << "Thread stopping motors..." << std::endl;
    std::vector<double> zero_pos(motor_mask.size(), 0.0);
    std::vector<double> zero_tau(motor_mask.size(), 0.0);
    mg.set_pos_ctr(zero_pos, zero_tau);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mg.disable(motor_mask);
}


int main(int argc, char **argv)
{
    // --- Rerun Initialization ---
    rerun::RecordingStream rec("rerun_dual_can_vis");
    rec.spawn().exit_on_failure();

    // --- Bus 1 Setup (can0) ---
    const std::string can0_if = "can0";
    std::vector<int> motor_ids_bus0 = {1, 2, 3};
    std::vector<double> kps_bus0(motor_ids_bus0.size(), 3.5);
    std::vector<double> kds_bus0(motor_ids_bus0.size(), 0.1);
    std::vector<bool> all_on_bus0(motor_ids_bus0.size(), true);
    motor_group mg0(motor_ids_bus0, can0_if, kps_bus0, kds_bus0);
    std::cout << "Initializing motor group on " << can0_if << std::endl;
    mg0.init();

    // --- Bus 2 Setup (can1) ---
    const std::string can1_if = "can1";
    std::vector<int> motor_ids_bus1 = {4, 5, 6};
    std::vector<double> kps_bus1(motor_ids_bus1.size(), 3.5);
    std::vector<double> kds_bus1(motor_ids_bus1.size(), 0.1);
    std::vector<bool> all_on_bus1(motor_ids_bus1.size(), true);
    motor_group mg1(motor_ids_bus1, can1_if, kps_bus1, kds_bus1);
    std::cout << "Initializing motor group on " << can1_if << std::endl;
    mg1.init();

    // --- Enable and Zero Both Groups ---
    std::cout << "Enabling motors..." << std::endl;
    mg0.enable(all_on_bus0);
    mg1.enable(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Setting new zero..." << std::endl;
    mg0.zero(all_on_bus0);
    mg1.zero(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // --- Setup Threads ---
    MotorGroupControl control0, control1;
    std::atomic<bool> stop_token(false);

    std::cout << "Spawning control threads..." << std::endl;
    std::thread thread0(motor_control_thread, std::ref(mg0), std::ref(control0), std::ref(stop_token), std::ref(all_on_bus0));
    std::thread thread1(motor_control_thread, std::ref(mg1), std::ref(control1), std::ref(stop_token), std::ref(all_on_bus1));
    
    std::cout << "Starting sine wave test on both buses..." << std::endl;

    // --- Main Loop ---
    double phase = 0.0;
    const double freq = 2.0;
    const double dt = 0.002;
    const int N = 2000;
    const double p2p = M_PI;
    auto dt_duration = std::chrono::microseconds(static_cast<long long>(dt * 1e6));
    auto next_cycle_time = std::chrono::steady_clock::now();

    for (int i = 0; i < N; ++i)
    {
        next_cycle_time += dt_duration;
        rec.set_time_sequence("step", i);
        phase += 2 * M_PI * freq * dt;

        // --- Update Shared Commands (protected by mutex) ---
        {
            std::lock_guard<std::mutex> lock(control0.mtx);
            control0.targets.assign(motor_ids_bus0.size(), 0.0);
            control0.tau_ff.assign(motor_ids_bus0.size(), 0.0);
            for (size_t j = 0; j < motor_ids_bus0.size(); ++j) {
                control0.targets[j] = std::sin(phase + j * 0.2) * p2p;
            }
        }
        {
            std::lock_guard<std::mutex> lock(control1.mtx);
            control1.targets.assign(motor_ids_bus1.size(), 0.0);
            control1.tau_ff.assign(motor_ids_bus1.size(), 0.0);
            for (size_t j = 0; j < motor_ids_bus1.size(); ++j) {
                // Using the same command for direct comparison
                control1.targets[j] = std::sin(phase + j * 0.2) * p2p;
            }
        }

        // --- Logging (reads from the motor_group's internal state) ---
        if (i % 10 == 0) {
            auto qs0 = mg0.read_q();
            auto qs1 = mg1.read_q();
            for (size_t j = 0; j < qs0.size(); ++j) {
                rec.log("q/motor_" + std::to_string(motor_ids_bus0[j]), rerun::Scalar(qs0[j]));
            }
            for (size_t j = 0; j < qs1.size(); ++j) {
                rec.log("q/motor_" + std::to_string(motor_ids_bus1[j]), rerun::Scalar(qs1[j]));
            }
        }

        std::this_thread::sleep_until(next_cycle_time);
    }

    // --- Stop and Cleanup Threads ---
    std::cout << "Stopping control threads..." << std::endl;
    stop_token = true;
    thread0.join();
    thread1.join();

    std::cout << "Main thread finished." << std::endl;
    return 0;
}
