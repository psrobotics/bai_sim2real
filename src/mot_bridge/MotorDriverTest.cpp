// main.cpp

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <rerun.hpp>

#include "motor_driver/motor_group.hpp"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0]
                  << " canX <motor_id_1> <motor_id_2> ...\n";
        return 1;
    }

    // Parse CAN interface
    const std::string can_if = "can0";
    assert(can_if.rfind("can", 0) == 0 &&
           "First argument must be a CAN interface (e.g. can0)");

    // --- Rerun Initialization ---
    // Create a Rerun recording stream, which is the main entry point to the SDK.
    // We give it a title that will show up in the Rerun Viewer.
    rerun::RecordingStream rec("rerun_motor_vis");
    // Spawn a Rerun Viewer to which we'll stream our data.
    // .exit_on_failure() will cause the program to exit if it can't connect to the viewer.
    rec.spawn().exit_on_failure();

    // Parse motor IDs
    std::vector<int> motor_ids = {1};
    /*
    for (int i = 2; i < argc; ++i) {
        motor_ids.push_back(std::atoi(argv[i]));
    }
    */

    // Build uniform Kp/Kd vectors
    const double default_kp = 3.5;
    const double default_kd = 0.1;
    std::vector<double> kps(motor_ids.size(), default_kp);
    std::vector<double> kds(motor_ids.size(), default_kd);

    // A mask of “true” for all motors
    std::vector<bool> all_on(motor_ids.size(), true);

    // Instantiate and init
    motor_group mg(motor_ids, can_if, kps, kds, motor_driver::MotorType::AK80_6_V1p1);
    std::cout << "init bf" << std::endl;
    mg.init();

    std::cout << "init finished" << std::endl;

    // Enable & zero all joints
    mg.enable(all_on);
    std::cout << "init 2 finished" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mg.zero(all_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Sine‐wave test
    double phase = 0.0;
    const double freq = 1.0; // Hz
    const double dt = 0.001; // loop period (1 ms)
    const int N = 1000000;   // number of steps
    const double p2p = M_PI;

    for (int i = 0; i < N; ++i)
    {
        phase += 2 * M_PI * freq * dt;

        // build target positions
        std::vector<double> targets;
        targets.reserve(motor_ids.size());
        for (size_t j = 0; j < motor_ids.size(); ++j)
        {
            // offset each joint by a phase shift
            targets.push_back(std::sin(phase + j * 0.2) * p2p);
        }

        // zero feed‐forward torque
        std::vector<double> tau_ff(motor_ids.size(), 0.0);

        // send command
        mg.set_pos_ctr(targets, tau_ff);

        // optional: read back and print every 10000 steps
        if (i % 10 == 0)
        {
            auto qs = mg.read_q();
            auto dqs = mg.read_dq();
            auto taus = mg.read_tau();

            // --- Logging to Rerun ---
            // Log the scalar values for each motor.
            // The first argument is the "entity path", which organizes the data in the viewer.
            for (size_t j = 0; j < motor_ids.size(); ++j)
            {
                std::string motor_id_str = "motor_" + std::to_string(motor_ids[j]);
                rec.log("q/" + motor_id_str, rerun::Scalar(qs[j]));
                rec.log("dq/" + motor_id_str, rerun::Scalar(dqs[j]));
                rec.log("tau/" + motor_id_str, rerun::Scalar(taus[j]));
            }

            std::cout << "Step " << i << " q[0]=" << qs[0] << "\n";
            std::cout << "            " << " dq[0]=" << dqs[0] << "\n";
            std::cout << "            " << " tau[0]=" << taus[0] << "\n";
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    // Stop motors (send zero pos/zero torque)
    std::vector<double> zero_pos(motor_ids.size(), 0.0),
        zero_tau(motor_ids.size(), 0.0);
    mg.set_pos_ctr(zero_pos, zero_tau);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Disable all
    mg.disable(all_on);

    return 0;
}
