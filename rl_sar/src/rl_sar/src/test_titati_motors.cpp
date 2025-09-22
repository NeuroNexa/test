/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
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

enum class CommandMode
{
    Torque,
    Velocity,
};

struct CommandLineOptions
{
    std::vector<int> joints;
    CommandMode mode = CommandMode::Torque;
    double command_value = 3.0;         // Nm or rad/s depending on the mode
    double command_duration_sec = 1.0;  // seconds
    double rest_duration_sec = 1.0;     // seconds between joints
    double velocity_kd = 0.8;           // derivative gain used for velocity mode
    double log_interval_sec = 1.0;      // seconds
    bool show_help = false;
};

void PrintUsage()
{
    std::cout << "Usage: test_titati_motors [options]\n"
              << "  --joint <index>        Add a joint to test (0-15). Repeat for multiple joints.\n"
              << "  --all                  Test every joint sequentially (default if none provided).\n"
              << "  --mode <torque|velocity>  Command constant torque (Nm) or velocity (rad/s).\n"
              << "  --value <number>       Command magnitude. Default 3.0 Nm or rad/s.\n"
              << "  --duration <sec>       Time to apply the command per joint (default 1.0).\n"
              << "  --rest <sec>           Zero-torque rest time after each joint (default 1.0).\n"
              << "  --velocity-kd <gain>   KD gain used for velocity mode (default 0.8).\n"
              << "  --log-interval <sec>   Interval between status printouts (default 1.0).\n"
              << "  -h, --help             Show this message.\n";
}

bool ParseJointIndex(const std::string &value, int &joint)
{
    try
    {
        joint = std::stoi(value);
    }
    catch (const std::exception &)
    {
        return false;
    }

    if (joint < 0 || joint >= kTitatiDofs)
    {
        return false;
    }

    return true;
}

bool ParseDouble(const std::string &name, int &idx, int argc, char **argv, double &target)
{
    if (++idx >= argc)
    {
        std::cerr << "Missing value for " << name << std::endl;
        return false;
    }

    try
    {
        target = std::stod(argv[idx]);
    }
    catch (const std::exception &)
    {
        std::cerr << "Invalid floating point value for " << name << std::endl;
        return false;
    }

    return true;
}

bool ParseCommandLine(int argc, char **argv, CommandLineOptions &options)
{
    for (int idx = 1; idx < argc; ++idx)
    {
        const std::string argument(argv[idx]);
        if (argument == "--help" || argument == "-h")
        {
            options.show_help = true;
            return true;
        }
        if (argument == "--all")
        {
            options.joints.clear();
            for (int joint = 0; joint < kTitatiDofs; ++joint)
            {
                options.joints.push_back(joint);
            }
            continue;
        }
        if (argument == "--joint")
        {
            if (++idx >= argc)
            {
                std::cerr << "Missing value for --joint" << std::endl;
                return false;
            }
            int joint = 0;
            if (!ParseJointIndex(argv[idx], joint))
            {
                std::cerr << "Invalid joint index: " << argv[idx] << std::endl;
                return false;
            }
            options.joints.push_back(joint);
            continue;
        }
        if (argument == "--mode")
        {
            if (++idx >= argc)
            {
                std::cerr << "Missing value for --mode" << std::endl;
                return false;
            }
            std::string mode_string(argv[idx]);
            std::transform(mode_string.begin(), mode_string.end(), mode_string.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            if (mode_string == "torque")
            {
                options.mode = CommandMode::Torque;
            }
            else if (mode_string == "velocity")
            {
                options.mode = CommandMode::Velocity;
            }
            else
            {
                std::cerr << "Unknown mode: " << mode_string << std::endl;
                return false;
            }
            continue;
        }
        if (argument == "--value")
        {
            if (!ParseDouble(argument, idx, argc, argv, options.command_value))
            {
                return false;
            }
            continue;
        }
        if (argument == "--duration")
        {
            if (!ParseDouble(argument, idx, argc, argv, options.command_duration_sec))
            {
                return false;
            }
            continue;
        }
        if (argument == "--rest")
        {
            if (!ParseDouble(argument, idx, argc, argv, options.rest_duration_sec))
            {
                return false;
            }
            continue;
        }
        if (argument == "--velocity-kd")
        {
            if (!ParseDouble(argument, idx, argc, argv, options.velocity_kd))
            {
                return false;
            }
            continue;
        }
        if (argument == "--log-interval")
        {
            if (!ParseDouble(argument, idx, argc, argv, options.log_interval_sec))
            {
                return false;
            }
            continue;
        }

        std::cerr << "Unknown argument: " << argument << std::endl;
        return false;
    }

    return true;
}

std::string ModeToString(CommandMode mode)
{
    return mode == CommandMode::Torque ? "torque" : "velocity";
}

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    CommandLineOptions options;
    if (!ParseCommandLine(argc, argv, options))
    {
        PrintUsage();
        return 1;
    }

    if (options.show_help)
    {
        PrintUsage();
        return 0;
    }

    if (options.joints.empty())
    {
        for (int joint = 0; joint < kTitatiDofs; ++joint)
        {
            options.joints.push_back(joint);
        }
    }

    std::sort(options.joints.begin(), options.joints.end());
    options.joints.erase(std::unique(options.joints.begin(), options.joints.end()), options.joints.end());

    if (options.command_duration_sec <= 0.0)
    {
        std::cerr << "Command duration must be positive." << std::endl;
        return 1;
    }

    if (options.mode == CommandMode::Velocity && options.velocity_kd <= 0.0)
    {
        std::cerr << "Velocity mode requires a positive --velocity-kd gain." << std::endl;
        return 1;
    }

    std::cout << "[Titati Motor Test] Initialising CAN interface..." << std::endl;
    tita_robot robot(kTitatiDofs);

    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "[Titati Motor Test] Failed to switch MCU to direct control mode."
                  << " Ensure the slave Jetson is forwarding CAN frames." << std::endl;
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
        std::cout << "[Titati Motor Test] Joint feedback unavailable, assuming zero pose." << std::endl;
        measured.assign(kTitatiDofs, 0.0);
    }

    std::cout << "[Titati Motor Test] Testing joints:";
    for (int joint : options.joints)
    {
        std::cout << ' ' << joint;
    }
    std::cout << "\n[Titati Motor Test] Mode: constant " << ModeToString(options.mode)
              << ", value=" << options.command_value
              << (options.mode == CommandMode::Torque ? " Nm" : " rad/s")
              << ", duration=" << options.command_duration_sec << " s"
              << ", rest=" << options.rest_duration_sec << " s" << std::endl;

    if (options.mode == CommandMode::Velocity)
    {
        std::cout << "[Titati Motor Test] Velocity KD gain: " << options.velocity_kd << std::endl;
    }

    std::vector<double> torque_command(kTitatiDofs, 0.0);
    std::vector<double> target_q(kTitatiDofs, 0.0);
    std::vector<double> target_v(kTitatiDofs, 0.0);
    std::vector<double> zero_gains(kTitatiDofs, 0.0);
    std::vector<double> kd_values(kTitatiDofs, 0.0);
    std::vector<double> zero_tau(kTitatiDofs, 0.0);

    for (int joint : options.joints)
    {
        if (!g_running.load())
        {
            break;
        }

        std::cout << "\n[Joint " << joint << "] applying " << ModeToString(options.mode)
                  << " command" << std::endl;

        const auto start_time = std::chrono::steady_clock::now();
        auto last_log = start_time;

        while (g_running.load())
        {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= options.command_duration_sec)
            {
                break;
            }

            std::vector<double> feedback_q;
            std::vector<double> feedback_dq;
            std::vector<double> feedback_tau;

            if (options.mode == CommandMode::Torque)
            {
                torque_command.assign(kTitatiDofs, 0.0);
                torque_command[joint] = options.command_value;
                robot.set_target_joint_t(torque_command);

                feedback_q = robot.get_joint_q();
                feedback_dq = robot.get_joint_v();
                feedback_tau = robot.get_joint_t();
            }
            else
            {
                feedback_q = robot.get_joint_q();
                if (feedback_q.size() != static_cast<size_t>(kTitatiDofs))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                feedback_dq = robot.get_joint_v();
                target_q = feedback_q;
                target_v.assign(kTitatiDofs, 0.0);
                kd_values.assign(kTitatiDofs, 0.0);
                target_v[joint] = options.command_value;
                kd_values[joint] = options.velocity_kd;
                robot.set_target_joint_mit(target_q, target_v, zero_gains, kd_values, zero_tau);
                feedback_tau = robot.get_joint_t();
            }

            if (std::chrono::duration<double>(now - last_log).count() >= options.log_interval_sec)
            {
                const auto position = (feedback_q.size() == static_cast<size_t>(kTitatiDofs)) ? feedback_q[joint]
                                                                                              : std::numeric_limits<double>::quiet_NaN();
                const auto velocity = (feedback_dq.size() == static_cast<size_t>(kTitatiDofs)) ? feedback_dq[joint]
                                                                                                : std::numeric_limits<double>::quiet_NaN();
                const auto torque = (feedback_tau.size() == static_cast<size_t>(kTitatiDofs)) ? feedback_tau[joint]
                                                                                              : std::numeric_limits<double>::quiet_NaN();
                std::cout << std::fixed << std::setprecision(3)
                          << "  -> measured position=" << position << " rad"
                          << ", velocity=" << velocity << " rad/s"
                          << ", torque=" << torque << " Nm" << std::endl;
                last_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        torque_command.assign(kTitatiDofs, 0.0);
        robot.set_target_joint_t(torque_command);

        if (options.rest_duration_sec > 0.0)
        {
            std::cout << "[Joint " << joint << "] resting for " << options.rest_duration_sec << " s" << std::endl;
            const auto rest_until = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                     std::chrono::duration<double>(options.rest_duration_sec));
            while (g_running.load() && std::chrono::steady_clock::now() < rest_until)
            {
                robot.set_target_joint_t(torque_command);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    std::cout << "\n[Titati Motor Test] Stopping. Commanding zero torque." << std::endl;
    robot.set_target_joint_t(std::vector<double>(kTitatiDofs, 0.0));

    return 0;
}
