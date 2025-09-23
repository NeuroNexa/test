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
#include <optional>
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

struct Options
{
    std::string variant {"titati"};
    bool use_default_pose {false};
    std::optional<size_t> single_joint {};
    double leg_amplitude {0.3};
    double wheel_amplitude {0.2};
    double frequency {0.5};
    double duration {3.0};
    double hold_time {2.0};
    double settle_time {0.5};
    double kp_leg {40.0};
    double kd_leg {2.0};
    double kp_wheel {5.0};
    double kd_wheel {0.5};
    bool torque_mode {false};
    double torque_amplitude {5.0};
    double torque_bias {0.0};
};

void PrintHelp()
{
    std::cout << "Usage: titati_motor_test [options]\n"
              << "  --robot=<tita|titati>        Select robot variant (default titati)\n"
              << "  --pose=default               Hold the predefined default pose (otherwise use current pose)\n"
              << "  --joint=<index>              Only excite the given joint (0-based)\n"
              << "  --amplitude=<rad>            Leg joint amplitude (alias of --leg-amplitude)\n"
              << "  --leg-amplitude=<rad>        Motion amplitude for leg joints\n"
              << "  --wheel-amplitude=<rad>      Motion amplitude for wheel joints\n"
              << "  --frequency=<hz>             Excitation frequency in Hertz\n"
              << "  --duration=<sec>             Excitation duration per joint in seconds\n"
              << "  --kp=<value>                 Proportional gain for leg joints\n"
              << "  --kd=<value>                 Derivative gain for leg joints\n"
              << "  --wheel-kp=<value>           Proportional gain for wheel joints\n"
              << "  --wheel-kd=<value>           Derivative gain for wheel joints\n"
              << "  --mode=torque                Drive the target joint with a pure torque waveform\n"
              << "  --torque-amplitude=<Nm>      Sine torque amplitude (torque mode)\n"
              << "  --torque-bias=<Nm>           Torque offset added to the sine wave\n"
              << "  --hold-time=<sec>            Duration to stabilise before excitation\n"
              << "  --settle-time=<sec>          Duration to settle between joints\n"
              << std::endl;
}

bool ParseOptions(int argc, char **argv, Options &opts)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            PrintHelp();
            return false;
        }
        if (arg.rfind("--robot=", 0) == 0)
        {
            opts.variant = arg.substr(std::strlen("--robot="));
        }
        else if (arg.rfind("--robot_name=", 0) == 0)
        {
            opts.variant = arg.substr(std::strlen("--robot_name="));
        }
        else if (arg == "--pose=default")
        {
            opts.use_default_pose = true;
        }
        else if (arg.rfind("--joint=", 0) == 0)
        {
            opts.single_joint = static_cast<size_t>(std::stoul(arg.substr(std::strlen("--joint="))));
        }
        else if (arg.rfind("--amplitude=", 0) == 0 || arg.rfind("--leg-amplitude=", 0) == 0)
        {
            auto value = std::stod(arg.substr(arg.find('=') + 1));
            opts.leg_amplitude = value;
        }
        else if (arg.rfind("--wheel-amplitude=", 0) == 0)
        {
            opts.wheel_amplitude = std::stod(arg.substr(std::strlen("--wheel-amplitude=")));
        }
        else if (arg.rfind("--frequency=", 0) == 0)
        {
            opts.frequency = std::stod(arg.substr(std::strlen("--frequency=")));
        }
        else if (arg.rfind("--duration=", 0) == 0)
        {
            opts.duration = std::stod(arg.substr(std::strlen("--duration=")));
        }
        else if (arg.rfind("--kp=", 0) == 0)
        {
            opts.kp_leg = std::stod(arg.substr(std::strlen("--kp=")));
        }
        else if (arg.rfind("--kd=", 0) == 0)
        {
            opts.kd_leg = std::stod(arg.substr(std::strlen("--kd=")));
        }
        else if (arg.rfind("--wheel-kp=", 0) == 0)
        {
            opts.kp_wheel = std::stod(arg.substr(std::strlen("--wheel-kp=")));
        }
        else if (arg.rfind("--wheel-kd=", 0) == 0)
        {
            opts.kd_wheel = std::stod(arg.substr(std::strlen("--wheel-kd=")));
        }
        else if (arg == "--mode=torque" || arg == "--torque-mode")
        {
            opts.torque_mode = true;
        }
        else if (arg.rfind("--torque-amplitude=", 0) == 0)
        {
            opts.torque_amplitude = std::stod(arg.substr(std::strlen("--torque-amplitude=")));
        }
        else if (arg.rfind("--torque-bias=", 0) == 0)
        {
            opts.torque_bias = std::stod(arg.substr(std::strlen("--torque-bias=")));
        }
        else if (arg.rfind("--hold-time=", 0) == 0)
        {
            opts.hold_time = std::stod(arg.substr(std::strlen("--hold-time=")));
        }
        else if (arg.rfind("--settle-time=", 0) == 0)
        {
            opts.settle_time = std::stod(arg.substr(std::strlen("--settle-time=")));
        }
        else
        {
            std::cout << "[MotorTest] Unknown option: " << arg << std::endl;
            PrintHelp();
            return false;
        }
    }

    if (opts.variant != "tita" && opts.variant != "titati")
    {
        std::cout << "[MotorTest] Unknown robot variant '" << opts.variant << "', fallback to titati" << std::endl;
        opts.variant = "titati";
    }
    return true;
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

std::vector<double> GainVector(size_t dofs, size_t wheel_count, double kp_leg, double kp_wheel)
{
    std::vector<double> gains(dofs, kp_leg);
    size_t leg_dofs = dofs - wheel_count;
    for (size_t i = leg_dofs; i < dofs; ++i)
    {
        gains[i] = kp_wheel;
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
    Options options;
    if (!ParseOptions(argc, argv, options))
    {
        return 0;
    }

    g_dofs = (options.variant == "tita") ? 8 : 16;
    size_t wheel_count = (options.variant == "tita") ? 2 : 4;
    std::vector<double> predefined_pose = DefaultPose(options.variant);
    std::vector<double> kp = GainVector(g_dofs, wheel_count, options.kp_leg, options.kp_wheel);
    std::vector<double> kd(g_dofs, options.kd_leg);
    for (size_t i = g_dofs - wheel_count; i < g_dofs; ++i)
    {
        kd[i] = options.kd_wheel;
    }
    g_torque_limits = TorqueLimits(g_dofs, wheel_count);

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "[MotorTest] Initialising tita motors for variant '" << options.variant << "'" << std::endl;
    g_robot = std::make_unique<tita_robot>(g_dofs);
    if (!g_robot->set_motors_sdk(true))
    {
        std::cerr << "[MotorTest] Failed to enter direct control mode. Check CAN-FD setup." << std::endl;
        return 1;
    }

    std::vector<double> hold_pose = options.use_default_pose ? predefined_pose : g_robot->get_joint_q();
    if (hold_pose.size() != g_dofs)
    {
        hold_pose.resize(g_dofs, 0.0);
    }

    std::vector<double> zeros(g_dofs, 0.0);
    g_robot->set_target_joint_t(zeros);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (options.hold_time > 0.0)
    {
        std::cout << "[MotorTest] Stabilising pose for " << options.hold_time << " seconds..." << std::endl;
        MaintainPose(hold_pose, kp, kd, std::chrono::milliseconds(static_cast<int>(options.hold_time * 1000.0)));
    }

    auto excite_joint = [&](size_t joint)
    {
        std::cout << "[MotorTest] Exciting joint " << joint << std::endl;
        auto start = std::chrono::steady_clock::now();
        std::vector<double> target = hold_pose;
        std::vector<double> torques(g_dofs, 0.0);
        while (true)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            if (elapsed >= options.duration)
            {
                break;
            }
            if (options.torque_mode)
            {
                torques.assign(g_dofs, 0.0);
                double cmd = options.torque_bias + options.torque_amplitude * std::sin(2.0 * M_PI * options.frequency * elapsed);
                if (joint < g_torque_limits.size())
                {
                    cmd = std::clamp(cmd, -g_torque_limits[joint], g_torque_limits[joint]);
                }
                torques[joint] = cmd;
                if (!g_robot->set_target_joint_t(torques))
                {
                    std::cerr << "[MotorTest] Warning: torque dispatch failed during excitation." << std::endl;
                    break;
                }
            }
            else
            {
                double amplitude = (joint >= g_dofs - wheel_count) ? options.wheel_amplitude : options.leg_amplitude;
                target[joint] = hold_pose[joint] + amplitude * std::sin(2.0 * M_PI * options.frequency * elapsed);
                if (!DispatchTorques(target, kp, kd))
                {
                    std::cerr << "[MotorTest] Warning: torque dispatch failed during excitation." << std::endl;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!options.torque_mode)
        {
            target[joint] = hold_pose[joint];
            if (options.settle_time > 0.0)
            {
                MaintainPose(target, kp, kd, std::chrono::milliseconds(static_cast<int>(options.settle_time * 1000.0)));
            }
        }
        else if (options.settle_time > 0.0)
        {
            g_robot->set_target_joint_t(zeros);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(options.settle_time * 1000.0)));
        }
    };

    if (options.single_joint)
    {
        size_t joint = *options.single_joint;
        if (joint >= g_dofs)
        {
            std::cerr << "[MotorTest] Joint index " << joint << " exceeds available DOFs (" << g_dofs << ")." << std::endl;
        }
        else
        {
            excite_joint(joint);
        }
    }
    else
    {
        for (size_t joint = 0; joint < g_dofs; ++joint)
        {
            excite_joint(joint);
        }
    }

    std::cout << "[MotorTest] Test complete. Motors will be disabled." << std::endl;
    SafeShutdown();
    return 0;
}
