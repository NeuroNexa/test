/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace LOGGER
{
const char *const INFO = "\033[0;37m[INFO]\033[0m ";
const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
const char *const ERROR = "\033[0;31m[ERROR]\033[0m ";
}

namespace
{
std::atomic<bool> running{true};

void SignalHandler(int)
{
    running = false;
}

struct Options
{
    std::string mode = "scan"; // scan, single, torque, monitor
    int joint = -1;
    bool absolute = false;
    double offset = 0.15; // rad
    double kp = 35.0;
    double kd = 1.5;
    double torque = 2.0;
    double rate_hz = 400.0;
    double duration = 2.0; // seconds per target
    double settle = 1.0;   // seconds to return to neutral
    size_t num_dofs = 16;
    std::string feedback_can = "can0";
    std::string command_can = "can0";
};

bool WaitForFeedback(tita_robot &robot, size_t expected_dofs, double timeout_sec)
{
    auto start = std::chrono::steady_clock::now();
    while (running)
    {
        auto joints = robot.get_joint_q();
        if (joints.size() == expected_dofs)
        {
            return true;
        }
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout_sec)
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void PrintUsage(const char *binary)
{
    std::cout << "Usage: " << binary
              << " [--mode scan|single|torque|monitor] [--joint IDX] [--offset RAD] [--absolute]" << std::endl
              << "             [--kp VAL] [--kd VAL] [--torque NM] [--duration SEC] [--settle SEC]" << std::endl
              << "             [--rate HZ] [--num-dofs N] [--feedback-can IFACE] [--command-can IFACE]" << std::endl
              << std::endl
              << "Examples:" << std::endl
              << "  " << binary << " --mode scan --offset 0.1" << std::endl
              << "  " << binary << " --mode single --joint 3 --offset -0.2 --kp 40 --kd 1.5" << std::endl
              << "  " << binary << " --mode torque --joint 5 --torque 3.0" << std::endl
              << "  " << binary << " --mode monitor" << std::endl;
}

bool ParseOptions(int argc, char **argv, Options &opts, bool &show_usage)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        auto need_value = [&](const std::string &name) -> bool {
            if (i + 1 >= argc)
            {
                std::cout << LOGGER::ERROR << "Option '" << name << "' expects a value." << std::endl;
                return false;
            }
            return true;
        };

        try
        {
            if (arg == "--mode")
            {
                if (!need_value(arg)) return false;
                opts.mode = argv[++i];
            }
            else if (arg == "--joint")
            {
                if (!need_value(arg)) return false;
                opts.joint = std::stoi(argv[++i]);
            }
            else if (arg == "--offset")
            {
                if (!need_value(arg)) return false;
                opts.offset = std::stod(argv[++i]);
            }
            else if (arg == "--kp")
            {
                if (!need_value(arg)) return false;
                opts.kp = std::stod(argv[++i]);
            }
            else if (arg == "--kd")
            {
                if (!need_value(arg)) return false;
                opts.kd = std::stod(argv[++i]);
            }
            else if (arg == "--torque")
            {
                if (!need_value(arg)) return false;
                opts.torque = std::stod(argv[++i]);
            }
            else if (arg == "--duration")
            {
                if (!need_value(arg)) return false;
                opts.duration = std::stod(argv[++i]);
            }
            else if (arg == "--settle")
            {
                if (!need_value(arg)) return false;
                opts.settle = std::stod(argv[++i]);
            }
            else if (arg == "--rate")
            {
                if (!need_value(arg)) return false;
                opts.rate_hz = std::stod(argv[++i]);
            }
            else if (arg == "--num-dofs")
            {
                if (!need_value(arg)) return false;
                opts.num_dofs = static_cast<size_t>(std::stoul(argv[++i]));
            }
            else if (arg == "--feedback-can")
            {
                if (!need_value(arg)) return false;
                opts.feedback_can = argv[++i];
            }
            else if (arg == "--command-can")
            {
                if (!need_value(arg)) return false;
                opts.command_can = argv[++i];
            }
            else if (arg == "--can" || arg == "--interface")
            {
                if (!need_value(arg)) return false;
                opts.feedback_can = argv[++i];
                opts.command_can = opts.feedback_can;
            }
            else if (arg == "--absolute")
            {
                opts.absolute = true;
            }
            else if (arg == "--help" || arg == "-h")
            {
                show_usage = true;
                PrintUsage(argv[0]);
                return false;
            }
            else
            {
                std::cout << LOGGER::ERROR << "Unknown option '" << arg << "'." << std::endl;
                return false;
            }
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Failed to parse argument for '" << arg << "': " << e.what() << std::endl;
            return false;
        }
    }

    return true;
}

} // namespace

int main(int argc, char **argv)
{
    Options opts;
    bool show_usage = false;
    if (!ParseOptions(argc, argv, opts, show_usage))
    {
        if (show_usage)
        {
            return 0;
        }
        return -1;
    }

    if (opts.rate_hz <= 0.0)
    {
        std::cout << LOGGER::ERROR << "Rate must be positive." << std::endl;
        return -1;
    }
    if (opts.num_dofs == 0)
    {
        std::cout << LOGGER::ERROR << "Number of DOFs must be greater than zero." << std::endl;
        return -1;
    }

    std::signal(SIGINT, SignalHandler);
    running = true;

    std::cout << LOGGER::INFO << "Using Titati CAN interfaces (feedback '" << opts.feedback_can << "', command '"
              << opts.command_can << "')." << std::endl;
    if (opts.mode != "monitor" && opts.feedback_can == opts.command_can)
    {
        std::cout << LOGGER::WARNING
                  << "Feedback and command share the same CAN bus. If your wiring splits them (for example CAN0 for feedback"
                     " and CAN1 for torque commands), rerun with --feedback-can/--command-can so the commands reach the motor"
                     " controller."
                  << std::endl;
    }

    tita_robot robot(opts.num_dofs, opts.feedback_can, opts.command_can);

    const bool require_direct_control = opts.mode != "monitor";
    bool motors_enabled = false;
    std::vector<double> zero_torque(opts.num_dofs, 0.0);
    constexpr double kMotionThresholdRad = 0.03; // ~1.7 deg
    constexpr double kTorqueThresholdNm = 0.5;

    if (require_direct_control)
    {
        motors_enabled = robot.set_motors_sdk(true);
        if (!motors_enabled)
        {
            std::cout << LOGGER::ERROR << "Unable to switch Titati to SDK control mode on CAN interface '"
                      << opts.command_can << "'." << std::endl;
            return -1;
        }
        std::cout << LOGGER::INFO << "Titati MCU switched to FORCE_DIRECT (SDK) control on '" << opts.command_can
                  << "'." << std::endl;
    }

    auto cleanup = [&]() {
        if (require_direct_control && motors_enabled)
        {
            robot.set_target_joint_t(zero_torque);
            robot.set_motors_sdk(false);
            std::cout << LOGGER::INFO << "Restored Titati MCU to AUTO_LOCOMOTION and cleared commanded torques." << std::endl;
        }
    };

    if (!WaitForFeedback(robot, opts.num_dofs, 2.0))
    {
        std::cout << LOGGER::ERROR << "Failed to receive Titati feedback within timeout." << std::endl;
        cleanup();
        return -1;
    }
    std::cout << LOGGER::INFO << "Feedback stream detected for " << opts.num_dofs
              << " joints. Continuing with requested mode." << std::endl;

    const std::chrono::duration<double> period(1.0 / opts.rate_hz);
    std::vector<double> dq(opts.num_dofs, 0.0);
    std::vector<double> kp(opts.num_dofs, 0.0);
    std::vector<double> kd(opts.num_dofs, 0.0);
    std::vector<double> tau(opts.num_dofs, 0.0);

    if (opts.mode == "monitor")
    {
        std::cout << LOGGER::INFO << "Streaming Titati joint states. Press Ctrl+C to exit." << std::endl;
        while (running)
        {
            auto q = robot.get_joint_q();
            auto v = robot.get_joint_v();
            auto t = robot.get_joint_t();
            if (q.size() == opts.num_dofs && v.size() == opts.num_dofs)
            {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "q:";
                for (size_t i = 0; i < opts.num_dofs; ++i)
                {
                    std::cout << " " << q[i];
                }
                std::cout << std::endl << "dq:";
                for (size_t i = 0; i < opts.num_dofs; ++i)
                {
                    std::cout << " " << v[i];
                }
                if (t.size() == opts.num_dofs)
                {
                    std::cout << std::endl << "tau:";
                    for (size_t i = 0; i < opts.num_dofs; ++i)
                    {
                        std::cout << " " << t[i];
                    }
                }
                std::cout << std::endl;
            }
            std::this_thread::sleep_for(period);
        }
        cleanup();
        return 0;
    }

    if (opts.mode == "scan")
    {
        auto initial = robot.get_joint_q();
        if (initial.size() != opts.num_dofs)
        {
            std::cout << LOGGER::ERROR << "Unexpected joint vector size during scan." << std::endl;
            cleanup();
            return -1;
        }
        std::cout << LOGGER::INFO << "Sequentially exciting each joint by " << opts.offset << " rad." << std::endl;
        for (size_t joint = 0; joint < opts.num_dofs && running; ++joint)
        {
            std::cout << LOGGER::INFO << "Testing joint " << joint << std::endl;
            auto target = initial;
            kp.assign(opts.num_dofs, 0.0);
            kd.assign(opts.num_dofs, 0.0);
            kp[joint] = opts.kp;
            kd[joint] = opts.kd;

            double desired = opts.absolute ? opts.offset : initial[joint] + opts.offset;
            target[joint] = desired;
            double observed_peak = 0.0;

            auto start = std::chrono::steady_clock::now();
            while (running && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < opts.duration)
            {
                if (!robot.set_target_joint_mit(target, dq, kp, kd, tau))
                {
                    std::cout << LOGGER::ERROR
                              << "Failed to push MIT command frame. Check the command CAN interface and retry." << std::endl;
                    cleanup();
                    return -1;
                }
                auto q = robot.get_joint_q();
                if (q.size() == opts.num_dofs)
                {
                    observed_peak = std::max(observed_peak, std::abs(q[joint] - initial[joint]));
                }
                std::this_thread::sleep_for(period);
            }

            target[joint] = initial[joint];
            start = std::chrono::steady_clock::now();
            while (running && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < opts.settle)
            {
                robot.set_target_joint_mit(target, dq, kp, kd, tau);
                std::this_thread::sleep_for(period);
            }

            if (observed_peak < kMotionThresholdRad)
            {
                std::cout << LOGGER::ERROR
                          << "Joint " << joint
                          << " barely moved (" << observed_peak
                          << " rad). Verify that the command CAN bus is correct (many Titati builds use"
                             " --command-can can1) and that the slave 'titati_canfd_router' is running."
                          << std::endl;
                cleanup();
                return -1;
            }
        }
        cleanup();
        return 0;
    }

    if (opts.mode == "single" || opts.mode == "torque")
    {
        if (opts.joint < 0 || static_cast<size_t>(opts.joint) >= opts.num_dofs)
        {
            std::cout << LOGGER::ERROR << "Joint index out of range." << std::endl;
            cleanup();
            return -1;
        }

        if (opts.mode == "single")
        {
            auto initial = robot.get_joint_q();
            if (initial.size() != opts.num_dofs)
            {
                std::cout << LOGGER::ERROR << "Unexpected joint vector size." << std::endl;
                cleanup();
                return -1;
            }

            auto target = initial;
            kp.assign(opts.num_dofs, 0.0);
            kd.assign(opts.num_dofs, 0.0);
            kp[opts.joint] = opts.kp;
            kd[opts.joint] = opts.kd;
            double desired = opts.absolute ? opts.offset : initial[opts.joint] + opts.offset;
            target[opts.joint] = desired;
            double observed_peak = 0.0;

            std::cout << LOGGER::INFO << "Driving joint " << opts.joint << " toward target " << desired << " rad." << std::endl;
            auto start = std::chrono::steady_clock::now();
            while (running && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < opts.duration)
            {
                robot.set_target_joint_mit(target, dq, kp, kd, tau);
                auto q = robot.get_joint_q();
                if (q.size() == opts.num_dofs)
                {
                    observed_peak = std::max(observed_peak, std::abs(q[opts.joint] - initial[opts.joint]));
                }
                std::this_thread::sleep_for(period);
            }

            target[opts.joint] = initial[opts.joint];
            start = std::chrono::steady_clock::now();
            while (running && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < opts.settle)
            {
                if (!robot.set_target_joint_mit(target, dq, kp, kd, tau))
                {
                    std::cout << LOGGER::ERROR
                              << "Failed to push MIT command frame. Check the command CAN interface and retry." << std::endl;
                    cleanup();
                    return -1;
                }
                std::this_thread::sleep_for(period);
            }

            if (observed_peak < kMotionThresholdRad)
            {
                std::cout << LOGGER::ERROR
                          << "Joint " << opts.joint
                          << " barely moved (" << observed_peak
                          << " rad). Verify the command CAN assignment (try --command-can can1) and confirm the slave"
                             " controller keeps Titati in FORCE_DIRECT mode."
                          << std::endl;
                cleanup();
                return -1;
            }
        }
        else // torque mode
        {
            std::cout << LOGGER::INFO << "Applying " << opts.torque << " Nm torque to joint " << opts.joint << "." << std::endl;
            auto torques = zero_torque;
            torques[opts.joint] = opts.torque;
            double observed_tau = 0.0;
            auto start = std::chrono::steady_clock::now();
            while (running && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < opts.duration)
            {
                if (!robot.set_target_joint_t(torques))
                {
                    std::cout << LOGGER::ERROR
                              << "Failed to push torque command frame. Check the command CAN interface and retry." << std::endl;
                    cleanup();
                    return -1;
                }
                auto tau_measured = robot.get_joint_t();
                if (tau_measured.size() == opts.num_dofs)
                {
                    observed_tau = std::max(observed_tau, std::abs(tau_measured[opts.joint]));
                }
                std::this_thread::sleep_for(period);
            }
            if (!robot.set_target_joint_t(zero_torque))
            {
                std::cout << LOGGER::WARNING
                          << "Unable to send zero torque command at the end of the test. Ensure the CAN link is healthy." << std::endl;
            }

            if (observed_tau < kTorqueThresholdNm)
            {
                std::cout << LOGGER::ERROR
                          << "Joint " << opts.joint
                          << " reported only " << observed_tau
                          << " Nm during torque mode. The actuator likely did not receive the command. Double-check the"
                             " command CAN interface and that the Titati slave router is active."
                          << std::endl;
                cleanup();
                return -1;
            }
        }

        cleanup();
        return 0;
    }

    std::cout << LOGGER::ERROR << "Unknown mode '" << opts.mode << "'." << std::endl;
    cleanup();
    return -1;
}
