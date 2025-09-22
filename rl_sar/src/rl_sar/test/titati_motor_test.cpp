/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::unique_ptr<tita_robot> g_robot;
size_t g_dofs = 16;

void SafeShutdown()
{
    if (g_robot)
    {
        std::vector<double> zero(g_dofs, 0.0);
        g_robot->set_target_joint_t(zero);
        g_robot->set_motors_sdk(false);
        g_robot.reset();
    }
}

void SignalHandler(int)
{
    SafeShutdown();
    std::cout << "\n[MotorTest] Stop command received, motors disabled." << std::endl;
    std::exit(0);
}

std::string ParseVariant(int argc, char **argv)
{
    std::string variant = "titati";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg.rfind("--robot=", 0) == 0)
        {
            variant = arg.substr(std::strlen("--robot="));
        }
        else if (arg.rfind("--robot_name=", 0) == 0)
        {
            variant = arg.substr(std::strlen("--robot_name="));
        }
    }
    if (variant != "tita" && variant != "titati")
    {
        std::cout << "[MotorTest] Unknown robot variant '" << variant << "', fallback to titati" << std::endl;
        variant = "titati";
    }
    return variant;
}

std::vector<double> DefaultPose(const std::string &variant)
{
    if (variant == "tita")
    {
        return {0.00, 0.80, -1.50,
                0.00, 0.80, -1.50,
                0.00, 0.00};
    }
    return {0.00, 0.40, -0.917,
            0.00, 0.40, -0.917,
            0.00, -0.40, 0.917,
            0.00, -0.40, 0.917,
            0.00, 0.00, 0.00, 0.00};
}

std::vector<double> GainVector(size_t dofs, size_t wheel_count)
{
    std::vector<double> gains(dofs, 40.0);
    size_t leg_dofs = dofs - wheel_count;
    for (size_t i = leg_dofs; i < dofs; ++i)
    {
        gains[i] = 5.0;
    }
    return gains;
}

} // namespace

int main(int argc, char **argv)
{
    std::string variant = ParseVariant(argc, argv);
    g_dofs = (variant == "tita") ? 8 : 16;
    std::vector<double> default_pose = DefaultPose(variant);
    std::vector<double> kp = GainVector(g_dofs, variant == "tita" ? 2 : 4);
    std::vector<double> kd(g_dofs, 1.0);

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "[MotorTest] Initialising tita motors for variant '" << variant << "'" << std::endl;
    g_robot = std::make_unique<tita_robot>(g_dofs);
    if (!g_robot->set_motors_sdk(true))
    {
        std::cerr << "[MotorTest] Failed to enter direct control mode. Check CAN-FD setup." << std::endl;
        return 1;
    }

    std::vector<double> zeros(g_dofs, 0.0);
    g_robot->set_target_joint_t(zeros);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[MotorTest] Holding default posture before sweep..." << std::endl;
    g_robot->set_target_joint_mit(default_pose, zeros, kp, kd, zeros);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const double frequency = 0.5;      // Hz
    const double duration = 3.0;       // seconds per joint
    const double leg_amplitude = 0.3;  // radians
    const double wheel_amplitude = 0.2;

    for (size_t joint = 0; joint < g_dofs; ++joint)
    {
        std::cout << "[MotorTest] Exciting joint " << joint << std::endl;
        auto pose = default_pose;
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            if (elapsed >= duration)
            {
                break;
            }
            double amplitude = (joint >= g_dofs - (variant == "tita" ? 2 : 4)) ? wheel_amplitude : leg_amplitude;
            pose[joint] = default_pose[joint] + amplitude * std::sin(2.0 * M_PI * frequency * elapsed);
            std::vector<double> v(g_dofs, 0.0);
            g_robot->set_target_joint_mit(pose, v, kp, kd, zeros);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        g_robot->set_target_joint_mit(default_pose, zeros, kp, kd, zeros);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "[MotorTest] Sweep complete. Motors will be disabled." << std::endl;
    SafeShutdown();
    return 0;
}
