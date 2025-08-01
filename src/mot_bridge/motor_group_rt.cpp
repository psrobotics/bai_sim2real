// main.cpp for multi-threaded, dual CAN bus control with LCM and rt_loop
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include <chrono>
#include <algorithm>

// --- LCM Includes ---
#include <lcm/lcm-cpp.hpp>
#include "../exlcm/mot_cmd.hpp"
#include "../exlcm/mot_state.hpp"

// --- Custom Libraries ---
#include "motor_driver/motor_group.hpp"
#include "rt_loop.hpp"

// --- Shared data structures for thread communication ---
struct motor_group_control {
    std::mutex mtx;
    std::vector<double> targets;
    std::vector<double> tau_ff;
    std::vector<double> kps;
    std::vector<double> kds;
};

// --- LCM Handler Class ---
class command_handler {
public:
    command_handler(motor_group_control& ctrl0, motor_group_control& ctrl1)
        : control0(ctrl0), control1(ctrl1) {}

    void handle_message(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const exlcm::mot_cmd* msg)
    {
        // Lock both mutexes to update the shared data safely
        std::lock_guard<std::mutex> lock0(control0.mtx);
        std::lock_guard<std::mutex> lock1(control1.mtx);

        // --- Distribute commands to the two motor groups ---
        control0.targets.assign(msg->q, msg->q + 3);
        control0.tau_ff.assign(msg->tau, msg->tau + 3);
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
    motor_group_control& control0;
    motor_group_control& control1;
};

// --- Global list of loops for the signal handler ---
static std::vector<real_time_loop::loop*> g_rt_loops;
static std::atomic<bool> g_shutdown_requested(false);

// --- Signal handler for graceful shutdown ---
void handle_sigint(int)
{
    g_shutdown_requested = true;
    for (auto& loop : g_rt_loops) {
        if (loop) loop->shutdown();
    }
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
    motor_group_control control0;
    control0.kps.assign(motor_ids_bus0.size(), 3.5);
    control0.kds.assign(motor_ids_bus0.size(), 0.1);
    motor_group mg0(motor_ids_bus0, can0_if, control0.kps, control0.kds);
    mg0.init();

    const std::string can1_if = "can1";
    std::vector<int> motor_ids_bus1 = {4, 5, 6};
    motor_group_control control1;
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
    command_handler handler(control0, control1);
    lcm.subscribe("MOT_CMD", &command_handler::handle_message, &handler);
    std::thread lcm_thread([&]() {
        while (!g_shutdown_requested) {
            // Handle with a timeout so the thread can exit cleanly
            lcm.handle();
        }
    });

    // --- Setup Motor Control Loops ---
    const auto motor_loop_period = std::chrono::microseconds(2000); // 500Hz
    
    real_time_loop::loop_func motor_loop0("motor_can0", motor_loop_period, [&]() {
        std::vector<double> current_targets, current_tau_ff;
        {
            std::lock_guard<std::mutex> lock(control0.mtx);
            current_targets = control0.targets;
            current_tau_ff = control0.tau_ff;
            mg0.set_kp(control0.kps);
            mg0.set_kd(control0.kds);
        }
        if (!current_targets.empty()) {
            mg0.set_pos_ctr(current_targets, current_tau_ff);
        }
    });

    real_time_loop::loop_func motor_loop1("motor_can1", motor_loop_period, [&]() {
        std::vector<double> current_targets, current_tau_ff;
        {
            std::lock_guard<std::mutex> lock(control1.mtx);
            current_targets = control1.targets;
            current_tau_ff = control1.tau_ff;
            mg1.set_kp(control1.kps);
            mg1.set_kd(control1.kds);
        }
        if (!current_targets.empty()) {
            mg1.set_pos_ctr(current_targets, current_tau_ff);
        }
    });

    // --- Setup State Publishing Loop ---
    const auto pub_loop_period = std::chrono::microseconds(4000); // 250 Hz
    
    real_time_loop::loop_func pub_loop("state_publisher", pub_loop_period, [&]() {
        auto qs0 = mg0.read_q(), dqs0 = mg0.read_dq(), taus0 = mg0.read_tau();
        auto qs1 = mg1.read_q(), dqs1 = mg1.read_dq(), taus1 = mg1.read_tau();

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
    });

    // --- Start all loops ---
    g_rt_loops = {&motor_loop0, &motor_loop1, &pub_loop};
    std::signal(SIGINT, handle_sigint);
    
    std::cout << "Starting control and publishing loops. Press Ctrl+C to exit." << std::endl;
    for (auto& loop : g_rt_loops) {
        loop->start();
    }

    // --- Wait for shutdown signal ---
    lcm_thread.join(); // The main thread will now block here until shutdown.

    // --- Cleanup ---
    std::cout << "\nShutdown signal received. Disabling motors..." << std::endl;
    std::vector<double> zero_pos(3, 0.0);
    mg0.set_pos_ctr(zero_pos, zero_pos);
    mg1.set_pos_ctr(zero_pos, zero_pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for command to send
    //mg0.disable(all_on_bus0);
    //mg1.disable(all_on_bus1);
    
    std::cout << "Exiting." << std::endl;
    return 0;
}
