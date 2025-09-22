/*
 * Titati joint posture bring-up diagnostic.
 * Commands the 16 Titati joints to the default standing pose and holds it.
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace
{
std::atomic<bool> g_running{true};

void SignalHandler(int)
{
    g_running = false;
}

constexpr int kTitatiDofs = 16;
} // namespace

int main()
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "[Titati Test] Initialising CAN interface..." << std::endl;
    tita_robot robot(kTitatiDofs);

    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "[Titati Test] Failed to switch MCU to direct control mode."
                  << " Please verify the slave Jetson is in forced direct mode." << std::endl;
    }

    const std::vector<double> default_q = {
        0.00, 0.40, -0.917,
        0.00, 0.40, -0.917,
        0.00, -0.40, 0.917,
        0.00, -0.40, 0.917,
        0.00, 0.00, 0.00, 0.00
    };

    std::vector<double> kp(kTitatiDofs, 40.0);
    std::vector<double> kd(kTitatiDofs, 2.0);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        kp[idx] = 5.0;
        kd[idx] = 0.5;
    }

    std::cout << "[Titati Test] Waiting for joint state feedback..." << std::endl;
    std::vector<double> measured = robot.get_joint_q();
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (measured.size() != static_cast<size_t>(kTitatiDofs) && std::chrono::steady_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        measured = robot.get_joint_q();
    }

    if (measured.size() != static_cast<size_t>(kTitatiDofs))
    {
        std::cout << "[Titati Test] Joint feedback unavailable, assuming zero start pose." << std::endl;
        measured.assign(kTitatiDofs, 0.0);
    }

    const double ramp_duration = 3.0;
    std::vector<double> target_q = measured;
    std::vector<double> target_dq(kTitatiDofs, 0.0);
    std::vector<double> target_tau(kTitatiDofs, 0.0);

    std::cout << "[Titati Test] Bringing robot to default standing posture. Press Ctrl+C to hold immediately." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_log = start_time;

    while (g_running.load())
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        double alpha = std::min(elapsed / ramp_duration, 1.0);

        for (int i = 0; i < kTitatiDofs; ++i)
        {
            target_q[i] = measured[i] + alpha * (default_q[i] - measured[i]);
        }

        robot.set_target_joint_mit(target_q, target_dq, kp, kd, target_tau);

        if (now - last_log >= std::chrono::seconds(1))
        {
            auto current = robot.get_joint_q();
            std::cout << std::fixed << std::setprecision(3)
                      << "[Titati Test] progress=" << (alpha * 100.0)
                      << "% | FR_hip=" << (current.size() > 0 ? current[0] : 0.0)
                      << " rad, RL_foot=" << (current.size() > 15 ? current[15] : 0.0)
                      << " rad" << std::endl;
            last_log = now;
        }

        if (alpha >= 1.0)
        {
            robot.set_target_joint_mit(default_q, target_dq, kp, kd, target_tau);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n[Titati Test] Holding default posture." << std::endl;
    robot.set_target_joint_mit(default_q, target_dq, kp, kd, target_tau);
    robot.set_target_joint_t(std::vector<double>(kTitatiDofs, 0.0));

    return 0;
}
