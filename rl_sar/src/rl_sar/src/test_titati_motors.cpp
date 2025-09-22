/*
 * Simple diagnostic program for Titati hardware.
 * Sends smooth MIT commands to all 16 joints to verify motion.
 */

#include "tita_robot/tita_robot.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
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
constexpr double kPi = 3.14159265358979323846;
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

    const std::vector<double> amplitude = {
        0.20, 0.35, 0.35,
        0.20, 0.35, 0.35,
        0.20, 0.35, 0.35,
        0.20, 0.35, 0.35,
        0.10, 0.10, 0.10, 0.10
    };

    const double frequency_hz = 0.25; // slow sweep
    const double omega = 2.0 * kPi * frequency_hz;

    std::vector<double> target_q(kTitatiDofs, 0.0);
    std::vector<double> target_dq(kTitatiDofs, 0.0);
    std::vector<double> target_tau(kTitatiDofs, 0.0);

    auto start_time = std::chrono::steady_clock::now();
    auto last_log = start_time;

    std::cout << "[Titati Test] Sending sinusoidal trajectories to all joints."
              << " Press Ctrl+C to stop." << std::endl;

    while (g_running.load())
    {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();

        for (int i = 0; i < kTitatiDofs; ++i)
        {
            double phase = static_cast<double>(i) * kPi / 8.0;
            target_q[i] = default_q[i] + amplitude[i] * std::sin(omega * t + phase);
            target_dq[i] = amplitude[i] * omega * std::cos(omega * t + phase);
        }

        robot.set_target_joint_mit(target_q, target_dq, kp, kd, target_tau);

        if (now - last_log >= std::chrono::seconds(1))
        {
            auto measured = robot.get_joint_q();
            std::cout << std::fixed << std::setprecision(3)
                      << "[Titati Test] t=" << t
                      << "s | FR_hip=" << (measured.size() > 0 ? measured[0] : 0.0)
                      << " rad, RL_foot=" << (measured.size() > 15 ? measured[15] : 0.0)
                      << " rad" << std::endl;
            last_log = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n[Titati Test] Stopping and holding default posture." << std::endl;
    robot.set_target_joint_mit(default_q, std::vector<double>(kTitatiDofs, 0.0), kp, kd, target_tau);
    robot.set_target_joint_t(std::vector<double>(kTitatiDofs, 0.0));

    return 0;
}
