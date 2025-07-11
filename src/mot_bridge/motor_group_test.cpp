// main.cpp for multi-threaded, dual CAN bus control with LCM

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>

// --- LCM Includes ---
#include <lcm/lcm-cpp.hpp>
#include "../exlcm/mot_cmd.hpp"
#include "../exlcm/mot_state.hpp"

#include "motor_driver/motor_group.hpp"

// --- Shared data structures for thread communication ---
struct MotorGroupControl {
    std::mutex mtx;
    std::vector<double> targets;
    std::vector<double> tau_ff;
    // We can also update Kp/Kd dynamically
    std::vector<double> kps;
    std::vector<double> kds;
};

// --- LCM Handler Class ---
class CommandHandler {
public:
    // Pass references to the control data for both motor groups
    CommandHandler(MotorGroupControl& ctrl0, MotorGroupControl& ctrl1)
        : control0(ctrl0), control1(ctrl1) {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const exlcm::mot_cmd* msg)
    {
        // Lock both mutexes to update the shared data safely
        std::lock_guard<std::mutex> lock0(control0.mtx);
        std::lock_guard<std::mutex> lock1(control1.mtx);

        // --- Distribute commands to the two motor groups ---
        // Group 0 gets the first 3 joints
        control0.targets.assign(msg->q, msg->q + 3);
        control0.tau_ff.assign(msg->tau, msg->tau + 3);

        // Group 1 gets the next 3 joints
        control1.targets.assign(msg->q + 3, msg->q + 6);
        control1.tau_ff.assign(msg->tau + 3, msg->tau + 6);

        // Update Kp/Kd for both groups if they are provided
        if (msg->kp_tar > 0) {
            std::fill(control0.kps.begin(), control0.kps.end(), msg->kp_tar);
            std::fill(control1.kps.begin(), control1.kps.end(), msg->kp_tar);
        }
        if (msg->kd_tar > 0) {
            std::fill(control0.kds.begin(), control0.kds.end(), msg->kd_tar);
            std::fill(control1.kds.begin(), control1.kds.end(), msg->kd_tar);
        }
    }

private:
    MotorGroupControl& control0;
    MotorGroupControl& control1;
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
        
        {
            std::lock_guard<std::mutex> lock(control_data.mtx);
            current_targets = control_data.targets;
            current_tau_ff = control_data.tau_ff;
            // Update the motor group's Kp/Kd values
            mg.set_kp(control_data.kps);
            mg.set_kd(control_data.kds);
        }

        if (!current_targets.empty()) {
            mg.set_pos_ctr(current_targets, current_tau_ff);
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "Thread stopping motors..." << std::endl;
    std::vector<double> zero_pos(motor_mask.size(), 0.0);
    mg.set_pos_ctr(zero_pos, zero_pos); // Use zero_pos for tau_ff as well
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mg.disable(motor_mask);
}


int main(int argc, char **argv)
{
    // --- LCM Initialization ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Error: Could not initialize LCM." << std::endl;
        return 1;
    }

    // --- Bus Setups ---
    const std::string can0_if = "can0";
    std::vector<int> motor_ids_bus0 = {1, 2, 3};
    MotorGroupControl control0;
    control0.kps.assign(motor_ids_bus0.size(), 3.5);
    control0.kds.assign(motor_ids_bus0.size(), 0.1);
    motor_group mg0(motor_ids_bus0, can0_if, control0.kps, control0.kds);
    mg0.init();

    const std::string can1_if = "can1";
    std::vector<int> motor_ids_bus1 = {4, 5, 6};
    MotorGroupControl control1;
    control1.kps.assign(motor_ids_bus1.size(), 3.5);
    control1.kds.assign(motor_ids_bus1.size(), 0.1);
    motor_group mg1(motor_ids_bus1, can1_if, control1.kps, control1.kds);
    mg1.init();

    // --- Enable and Zero ---
    std::vector<bool> all_on_bus0(motor_ids_bus0.size(), true);
    std::vector<bool> all_on_bus1(motor_ids_bus1.size(), true);
    std::cout << "Enabling motors..." << std::endl;
    mg0.enable(all_on_bus0);
    mg1.enable(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mg0.zero(all_on_bus0);
    mg1.zero(all_on_bus1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // --- Setup LCM Subscription and Thread ---
    CommandHandler handler(control0, control1);
    lcm.subscribe("MOT_CMD", &CommandHandler::handleMessage, &handler);
    std::thread lcm_thread([&]() {
        while (0 == lcm.handle());
    });
    lcm_thread.detach(); // Let the LCM thread run in the background

    // --- Setup Motor Control Threads ---
    std::atomic<bool> stop_token(false);
    std::thread thread0(motor_control_thread, std::ref(mg0), std::ref(control0), std::ref(stop_token), std::ref(all_on_bus0));
    std::thread thread1(motor_control_thread, std::ref(mg1), std::ref(control1), std::ref(stop_token), std::ref(all_on_bus1));
    
    // --- Main Loop (now for publishing state) ---
    std::cout << "Starting state publishing loop..." << std::endl;
    const double dt = 0.002; // 500 Hz publishing rate
    auto dt_duration = std::chrono::microseconds(static_cast<long long>(dt * 1e6));
    auto next_cycle_time = std::chrono::steady_clock::now();
    int step = 0;

    while (true) // Run indefinitely until Ctrl+C
    {
        next_cycle_time += dt_duration;

        // --- Read current state from both motor groups ---
        auto qs0 = mg0.read_q();
        auto dqs0 = mg0.read_dq();
        auto taus0 = mg0.read_tau();
        auto qs1 = mg1.read_q();
        auto dqs1 = mg1.read_dq();
        auto taus1 = mg1.read_tau();

        // --- Publish combined state via LCM ---
        exlcm::mot_state state_msg;
        state_msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::copy(qs0.begin(), qs0.end(), state_msg.q);
        std::copy(qs1.begin(), qs1.end(), state_msg.q + 3);
        std::copy(dqs0.begin(), dqs0.end(), state_msg.dq);
        std::copy(dqs1.begin(), dqs1.end(), state_msg.dq + 3);
        std::copy(taus0.begin(), taus0.end(), state_msg.tau);
        std::copy(taus1.begin(), taus1.end(), state_msg.tau + 3);
        lcm.publish("MOT_STATE", &state_msg);

        std::this_thread::sleep_until(next_cycle_time);
    }

    // --- Cleanup (in a real app, you'd handle Ctrl+C to get here) ---
    stop_token = true;
    thread0.join();
    thread1.join();
    return 0;
}
