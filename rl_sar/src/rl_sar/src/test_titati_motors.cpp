/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
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
constexpr double kSweepFrequencyHz = 0.4;
constexpr double kSweepDurationSec = 6.0;
constexpr double kHoldDurationSec = 1.0;
constexpr double kTwoPi = 6.28318530717958647692;

std::vector<double> BuildDefaultGains()
{
    std::vector<double> gains(kTitatiDofs, 40.0);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        gains[idx] = 5.0;
    }
    return gains;
}

std::vector<double> BuildDefaultDamping()
{
    std::vector<double> damping(kTitatiDofs, 2.0);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        damping[idx] = 0.5;
    }
    return damping;
}

std::vector<double> MotorAmplitudeProfile()
{
    std::vector<double> profile(kTitatiDofs, 0.35);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        profile[idx] = 0.12;
    }
    return profile;
}

} // namespace

int main()
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "[Titati Motor Test] Initialising CAN interface..." << std::endl;
    tita_robot robot(kTitatiDofs);

    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "[Titati Motor Test] Failed to switch MCU to direct control mode."
                  << " Please verify the slave Jetson is forwarding CAN frames." << std::endl;
    }

    std::cout << "[Titati Motor Test] Waiting for joint state feedback..." << std::endl;
    auto measured = robot.get_joint_q();
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (measured.size() != static_cast<size_t>(kTitatiDofs) && std::chrono::steady_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        measured = robot.get_joint_q();
    }

    if (measured.size() != static_cast<size_t>(kTitatiDofs))
    {
        std::cout << "[Titati Motor Test] Joint feedback unavailable, assuming zero start pose." << std::endl;
        measured.assign(kTitatiDofs, 0.0);
    }

    const auto kp = BuildDefaultGains();
    const auto kd = BuildDefaultDamping();
    const auto amplitude = MotorAmplitudeProfile();

    std::vector<double> target_q = measured;
    std::vector<double> target_dq(kTitatiDofs, 0.0);
    std::vector<double> target_tau(kTitatiDofs, 0.0);

    std::cout << "[Titati Motor Test] Exercising each joint individually." << std::endl;
    std::cout << "[Titati Motor Test] Press Ctrl+C at any time to stop and hold position." << std::endl;

    for (int joint = 0; joint < kTitatiDofs && g_running.load(); ++joint)
    {
        auto neutral = measured;
        std::cout << "\n[Joint " << joint << "] sweeping +/-" << amplitude[joint] << " rad" << std::endl;

        const auto start_time = std::chrono::steady_clock::now();
        auto last_log = start_time;

        while (g_running.load())
        {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= kSweepDurationSec)
            {
                break;
            }

            const double phase = kTwoPi * kSweepFrequencyHz * elapsed;
            const double offset = amplitude[joint] * std::sin(phase);

            target_q = neutral;
            target_q[joint] = neutral[joint] + offset;

            robot.set_target_joint_mit(target_q, target_dq, kp, kd, target_tau);

            if (now - last_log >= std::chrono::seconds(1))
            {
                const auto feedback = robot.get_joint_q();
                const double reported = (feedback.size() == static_cast<size_t>(kTitatiDofs)) ? feedback[joint] : 0.0;
                std::cout << std::fixed << std::setprecision(3)
                          << "  -> command=" << target_q[joint]
                          << " rad, measured=" << reported
                          << " rad" << std::endl;
                last_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::cout << "[Joint " << joint << "] hold neutral" << std::endl;
        const auto hold_until = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(kHoldDurationSec));
        while (g_running.load() && std::chrono::steady_clock::now() < hold_until)
        {
            robot.set_target_joint_mit(neutral, target_dq, kp, kd, target_tau);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        measured = robot.get_joint_q();
        if (measured.size() != static_cast<size_t>(kTitatiDofs))
        {
            measured = neutral;
        }
    }

    std::cout << "\n[Titati Motor Test] Stopping. Commanding zero torque." << std::endl;
    robot.set_target_joint_t(std::vector<double>(kTitatiDofs, 0.0));

    return 0;
}
