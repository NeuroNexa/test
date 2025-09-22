/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
std::atomic<bool> g_running{true};

void SignalHandler(int)
{
    g_running = false;
}

constexpr int kTitatiDofs = 16;
constexpr double kTwoPi = 6.28318530717958647692;

struct CommandLineOptions
{
    std::vector<int> joints;
    double sweep_frequency_hz = 0.4;
    double sweep_duration_sec = 6.0;
    double hold_duration_sec = 1.0;
    double default_kp = 40.0;
    double ankle_kp = 5.0;
    double default_kd = 2.0;
    double ankle_kd = 0.5;
    double default_amplitude = 0.35;
    double ankle_amplitude = 0.12;
    std::vector<std::pair<int, double>> amplitude_overrides;
    std::vector<std::pair<int, double>> kp_overrides;
    std::vector<std::pair<int, double>> kd_overrides;
    bool show_help = false;
};

void PrintUsage()
{
    std::cout << "Usage: test_titati_motors [options]\n"
              << "  --joint <index>       Specify a joint to exercise (0-15)."
              << " Repeat for multiple joints.\n"
              << "  --all                 Sweep all 16 joints sequentially.\n"
              << "  --frequency <Hz>      Sinusoid frequency (default 0.4).\n"
              << "  --duration <sec>      Duration of each sweep (default 6.0).\n"
              << "  --hold <sec>          Hold time after sweep (default 1.0).\n"
              << "  --kp <value>          Default proportional gain (default 40).\n"
              << "  --ankle-kp <value>    Proportional gain for joints 12-15 (default 5).\n"
              << "  --kd <value>          Default derivative gain (default 2).\n"
              << "  --ankle-kd <value>    Derivative gain for joints 12-15 (default 0.5).\n"
              << "  --amplitude <rad>     Default sweep amplitude (default 0.35).\n"
              << "  --ankle-amplitude <rad>  Amplitude for joints 12-15 (default 0.12).\n"
              << "  --amp <joint:value>   Override amplitude for a specific joint.\n"
              << "  --kp-joint <joint:value> Override kp for a specific joint.\n"
              << "  --kd-joint <joint:value> Override kd for a specific joint.\n"
              << "  -h, --help            Show this message.\n";
}

bool ParseJointValuePair(const std::string &argument, std::pair<int, double> &result)
{
    const auto delimiter = argument.find(':');
    if (delimiter == std::string::npos)
    {
        return false;
    }

    try
    {
        const int joint = std::stoi(argument.substr(0, delimiter));
        const double value = std::stod(argument.substr(delimiter + 1));
        result = {joint, value};
    }
    catch (const std::exception &)
    {
        return false;
    }

    return true;
}

bool ParseCommandLine(int argc, char **argv, CommandLineOptions &options)
{
    for (int idx = 1; idx < argc; ++idx)
    {
        std::string argument(argv[idx]);
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
        if (argument == "--joint" || argument == "-j")
        {
            if (++idx >= argc)
            {
                std::cerr << "Missing value for --joint" << std::endl;
                return false;
            }
            const std::string value(argv[idx]);
            try
            {
                const int joint = std::stoi(value);
                if (joint < 0 || joint >= kTitatiDofs)
                {
                    std::cerr << "Joint index out of range: " << joint << std::endl;
                    return false;
                }
                options.joints.push_back(joint);
            }
            catch (const std::exception &)
            {
                std::cerr << "Invalid joint index: " << value << std::endl;
                return false;
            }
            continue;
        }

        auto parse_scalar = [&](const std::string &name, double &target) -> bool {
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
        };

        if (argument == "--frequency")
        {
            if (!parse_scalar(argument, options.sweep_frequency_hz))
            {
                return false;
            }
            continue;
        }
        if (argument == "--duration")
        {
            if (!parse_scalar(argument, options.sweep_duration_sec))
            {
                return false;
            }
            continue;
        }
        if (argument == "--hold")
        {
            if (!parse_scalar(argument, options.hold_duration_sec))
            {
                return false;
            }
            continue;
        }
        if (argument == "--kp")
        {
            if (!parse_scalar(argument, options.default_kp))
            {
                return false;
            }
            continue;
        }
        if (argument == "--ankle-kp")
        {
            if (!parse_scalar(argument, options.ankle_kp))
            {
                return false;
            }
            continue;
        }
        if (argument == "--kd")
        {
            if (!parse_scalar(argument, options.default_kd))
            {
                return false;
            }
            continue;
        }
        if (argument == "--ankle-kd")
        {
            if (!parse_scalar(argument, options.ankle_kd))
            {
                return false;
            }
            continue;
        }
        if (argument == "--amplitude")
        {
            if (!parse_scalar(argument, options.default_amplitude))
            {
                return false;
            }
            continue;
        }
        if (argument == "--ankle-amplitude")
        {
            if (!parse_scalar(argument, options.ankle_amplitude))
            {
                return false;
            }
            continue;
        }
        if (argument == "--amp" || argument == "--kp-joint" || argument == "--kd-joint")
        {
            if (++idx >= argc)
            {
                std::cerr << "Missing value for " << argument << std::endl;
                return false;
            }

            std::pair<int, double> parsed_pair{};
            if (!ParseJointValuePair(argv[idx], parsed_pair))
            {
                std::cerr << "Invalid format for " << argument
                          << ". Expected <joint:value>." << std::endl;
                return false;
            }

            if (parsed_pair.first < 0 || parsed_pair.first >= kTitatiDofs)
            {
                std::cerr << "Joint index out of range: " << parsed_pair.first << std::endl;
                return false;
            }

            if (argument == "--amp")
            {
                options.amplitude_overrides.push_back(parsed_pair);
            }
            else if (argument == "--kp-joint")
            {
                options.kp_overrides.push_back(parsed_pair);
            }
            else
            {
                options.kd_overrides.push_back(parsed_pair);
            }
            continue;
        }

        std::cerr << "Unknown argument: " << argument << std::endl;
        return false;
    }

    return true;
}

std::vector<double> BuildGainProfile(double default_gain,
                                     double ankle_gain,
                                     const std::vector<std::pair<int, double>> &overrides)
{
    std::vector<double> gains(kTitatiDofs, default_gain);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        gains[idx] = ankle_gain;
    }
    for (const auto &override_pair : overrides)
    {
        gains[override_pair.first] = override_pair.second;
    }
    return gains;
}

std::vector<double> BuildAmplitudeProfile(double default_amplitude,
                                          double ankle_amplitude,
                                          const std::vector<std::pair<int, double>> &overrides)
{
    std::vector<double> profile(kTitatiDofs, default_amplitude);
    for (int idx = 12; idx < kTitatiDofs; ++idx)
    {
        profile[idx] = ankle_amplitude;
    }
    for (const auto &override_pair : overrides)
    {
        profile[override_pair.first] = override_pair.second;
    }
    return profile;
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
        std::cout << "No joints specified. Use --joint <index> or --all.\n";
        PrintUsage();
        return 0;
    }

    std::sort(options.joints.begin(), options.joints.end());
    options.joints.erase(std::unique(options.joints.begin(), options.joints.end()),
                         options.joints.end());

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

    const auto kp = BuildGainProfile(options.default_kp, options.ankle_kp, options.kp_overrides);
    const auto kd = BuildGainProfile(options.default_kd, options.ankle_kd, options.kd_overrides);
    const auto amplitude = BuildAmplitudeProfile(options.default_amplitude,
                                                 options.ankle_amplitude,
                                                 options.amplitude_overrides);

    std::vector<double> target_q = measured;
    std::vector<double> target_dq(kTitatiDofs, 0.0);
    std::vector<double> command_tau(kTitatiDofs, 0.0);

    std::cout << "[Titati Motor Test] Exercising selected joints." << std::endl;
    std::cout << "[Titati Motor Test] Parameters: frequency=" << options.sweep_frequency_hz
              << " Hz, duration=" << options.sweep_duration_sec
              << " s, hold=" << options.hold_duration_sec << " s" << std::endl;
    std::cout << "[Titati Motor Test] Press Ctrl+C at any time to stop and hold position." << std::endl;

    for (int joint : options.joints)
    {
        if (!g_running.load())
        {
            break;
        }

        auto neutral = measured;
        std::cout << "\n[Joint " << joint << "] sweeping +/-" << amplitude[joint] << " rad" << std::endl;

        const auto start_time = std::chrono::steady_clock::now();
        auto last_log = start_time;

        while (g_running.load())
        {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= options.sweep_duration_sec)
            {
                break;
            }

            const double phase = kTwoPi * options.sweep_frequency_hz * elapsed;
            const double offset = amplitude[joint] * std::sin(phase);
            const double offset_velocity =
                amplitude[joint] * kTwoPi * options.sweep_frequency_hz * std::cos(phase);

            target_q = neutral;
            target_dq.assign(kTitatiDofs, 0.0);
            target_q[joint] = neutral[joint] + offset;
            target_dq[joint] = offset_velocity;

            auto feedback_q = robot.get_joint_q();
            auto feedback_dq = robot.get_joint_v();
            if (feedback_q.size() != static_cast<size_t>(kTitatiDofs) ||
                feedback_dq.size() != static_cast<size_t>(kTitatiDofs))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            for (int idx = 0; idx < kTitatiDofs; ++idx)
            {
                const double pos_error = target_q[idx] - feedback_q[idx];
                const double vel_error = target_dq[idx] - feedback_dq[idx];
                command_tau[idx] = kp[idx] * pos_error + kd[idx] * vel_error;
            }

            robot.set_target_joint_t(command_tau);

            if (now - last_log >= std::chrono::seconds(1))
            {
                const double reported = feedback_q[joint];
                const double reported_dq = feedback_dq[joint];
                std::cout << std::fixed << std::setprecision(3)
                          << "  -> command=" << target_q[joint]
                          << " rad, measured=" << reported
                          << " rad, vel=" << reported_dq
                          << " rad" << std::endl;
                last_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::cout << "[Joint " << joint << "] hold neutral" << std::endl;
        const auto hold_until = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(options.hold_duration_sec));
        while (g_running.load() && std::chrono::steady_clock::now() < hold_until)
        {
            auto feedback_q = robot.get_joint_q();
            auto feedback_dq = robot.get_joint_v();
            if (feedback_q.size() != static_cast<size_t>(kTitatiDofs) ||
                feedback_dq.size() != static_cast<size_t>(kTitatiDofs))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            for (int idx = 0; idx < kTitatiDofs; ++idx)
            {
                const double pos_error = neutral[idx] - feedback_q[idx];
                const double vel_error = 0.0 - feedback_dq[idx];
                command_tau[idx] = kp[idx] * pos_error + kd[idx] * vel_error;
            }

            robot.set_target_joint_t(command_tau);
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
