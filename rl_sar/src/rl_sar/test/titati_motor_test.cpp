/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
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
std::vector<double> g_torque_limits;

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

std::vector<double> TorqueLimits(size_t dofs, size_t wheel_count)
{
    std::vector<double> limits(dofs, 85.0);
    size_t leg_dofs = dofs - wheel_count;
    for (size_t i = leg_dofs; i < dofs; ++i)
    {
        limits[i] = 7.5;
    }
    return limits;
}

bool DispatchTorques(const std::vector<double> &target,
                     const std::vector<double> &kp,
                     const std::vector<double> &kd)
{
    auto q = g_robot->get_joint_q();
    auto dq = g_robot->get_joint_v();
    std::vector<double> tau(g_dofs, 0.0);

    for (size_t i = 0; i < g_dofs; ++i)
    {
        double err = target[i] - q[i];
        double derr = -dq[i];
        double cmd = kp[i] * err + kd[i] * derr;
        if (i < g_torque_limits.size())
        {
            cmd = std::clamp(cmd, -g_torque_limits[i], g_torque_limits[i]);
        }
        tau[i] = cmd;
    }
    return g_robot->set_target_joint_t(tau);
}

void MaintainPose(const std::vector<double> &target,
                  const std::vector<double> &kp,
                  const std::vector<double> &kd,
                  std::chrono::milliseconds duration)
{
    auto deadline = std::chrono::steady_clock::now() + duration;
    bool warned = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (!DispatchTorques(target, kp, kd) && !warned)
        {
            std::cerr << "[MotorTest] Warning: failed to send torque command." << std::endl;
            warned = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

} // namespace

int main(int argc, char **argv)
{
    std::string variant = ParseVariant(argc, argv);
    g_dofs = (variant == "tita") ? 8 : 16;
    size_t wheel_count = (variant == "tita") ? 2 : 4;
    std::vector<double> default_pose = DefaultPose(variant);
    std::vector<double> kp = GainVector(g_dofs, wheel_count);
    std::vector<double> kd(g_dofs, 2.0);
    for (size_t i = g_dofs - wheel_count; i < g_dofs; ++i)
    {
        kd[i] = 0.5;
    }
    g_torque_limits = TorqueLimits(g_dofs, wheel_count);

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "[MotorTest] Initialising tita motors for variant '" << variant << "'" << std::endl;
    g_robot = std::make_unique<tita_robot>(g_dofs);
    if (!g_robot->set_motors_sdk(true))
    {
        std::cerr << "[MotorTest] Failed to enter direct control mode. Check CAN-FD setup." << std::endl;
        return 1;
    }

    std::vector<double> target = default_pose;

    std::vector<double> zeros(g_dofs, 0.0);
    g_robot->set_target_joint_t(zeros);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[MotorTest] Holding default posture before sweep..." << std::endl;
    MaintainPose(target, kp, kd, std::chrono::seconds(2));

    const double frequency = 0.5;      // Hz
    const double duration = 3.0;       // seconds per joint
    const double leg_amplitude = 0.3;  // radians
    const double wheel_amplitude = 0.2;

    for (size_t joint = 0; joint < g_dofs; ++joint)
    {
        std::cout << "[MotorTest] Exciting joint " << joint << std::endl;
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            if (elapsed >= duration)
            {
                break;
            }
            double amplitude = (joint >= g_dofs - wheel_count) ? wheel_amplitude : leg_amplitude;
            target[joint] = default_pose[joint] + amplitude * std::sin(2.0 * M_PI * frequency * elapsed);
            if (!DispatchTorques(target, kp, kd))
            {
                std::cerr << "[MotorTest] Warning: torque dispatch failed during excitation." << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        target[joint] = default_pose[joint];
        MaintainPose(target, kp, kd, std::chrono::milliseconds(500));
    }

    std::cout << "[MotorTest] Sweep complete. Motors will be disabled." << std::endl;
    SafeShutdown();
    return 0;
}
