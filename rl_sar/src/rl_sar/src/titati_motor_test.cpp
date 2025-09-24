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
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

enum class CommandMode
{
    Torque = 0,
    MIT = 1,
};

std::atomic<bool> *g_running_flag = nullptr;

void SignalHandler(int)
{
    if (g_running_flag)
    {
        g_running_flag->store(false);
    }
}

std::string ToLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [] (unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string ModeToString(CommandMode mode)
{
    return mode == CommandMode::Torque ? "torque" : "mit";
}

void PrintHelp(std::mutex &io_mutex)
{
    std::lock_guard<std::mutex> lock(io_mutex);
    std::cout << "Available commands:\n"
              << "  help                         Show this message\n"
              << "  status [index]              Print the latest joint state (all motors or a single motor)\n"
              << "  mode torque|mit             Switch the command mode and clear all pending targets\n"
              << "  set <index> <values...>     Update the command for a motor (0-based index)\n"
              << "       torque mode:           set <index> <torque_Nm>\n"
              << "       mit mode:              set <index> <pos_rad> [vel_rad_s kp kd ff_torque] (missing terms default to 0)\n"
              << "  zero [index|all]            Zero the command for one motor or every motor\n"
              << "  monitor [period_ms|off]     Toggle periodic state printing (default 500 ms)\n"
              << "  exit / quit                 Stop commanding torques and exit\n"
              << std::flush;
}

struct CommandBuffers
{
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> torque;
};

void ResetCommands(CommandBuffers &buffers)
{
    std::fill(buffers.position.begin(), buffers.position.end(), 0.0);
    std::fill(buffers.velocity.begin(), buffers.velocity.end(), 0.0);
    std::fill(buffers.kp.begin(), buffers.kp.end(), 0.0);
    std::fill(buffers.kd.begin(), buffers.kd.end(), 0.0);
    std::fill(buffers.torque.begin(), buffers.torque.end(), 0.0);
}

void PrintMotorStates(
    tita_robot &robot,
    CommandMode mode,
    const CommandBuffers &commands,
    std::mutex &io_mutex,
    int motor_index = -1)
{
    auto q = robot.get_joint_q();
    auto dq = robot.get_joint_v();
    auto tau = robot.get_joint_t();

    const std::size_t motor_count = std::min({q.size(), dq.size(), tau.size(), commands.torque.size()});

    std::lock_guard<std::mutex> lock(io_mutex);
    std::cout << std::fixed << std::setprecision(4);

    if (motor_index >= 0)
    {
        if (static_cast<std::size_t>(motor_index) >= motor_count)
        {
            std::cout << "Motor index " << motor_index << " is out of range (" << motor_count << " motors reported)." << std::endl;
            return;
        }

        std::cout << "Motor " << motor_index << " state (mode: " << ModeToString(mode) << ")" << std::endl;
        std::cout << "  q: " << q[motor_index] << " rad" << std::endl;
        std::cout << "  dq: " << dq[motor_index] << " rad/s" << std::endl;
        std::cout << "  tau_est: " << tau[motor_index] << " Nm" << std::endl;

        if (mode == CommandMode::Torque)
        {
            std::cout << "  cmd_tau: " << commands.torque[motor_index] << " Nm" << std::endl;
        }
        else
        {
            std::cout << "  cmd_q: " << commands.position[motor_index] << " rad" << std::endl;
            std::cout << "  cmd_dq: " << commands.velocity[motor_index] << " rad/s" << std::endl;
            std::cout << "  cmd_kp: " << commands.kp[motor_index] << std::endl;
            std::cout << "  cmd_kd: " << commands.kd[motor_index] << std::endl;
            std::cout << "  cmd_tau: " << commands.torque[motor_index] << " Nm" << std::endl;
        }
        return;
    }

    std::cout << "Motor states (mode: " << ModeToString(mode) << ")" << std::endl;
    std::cout << " idx |     q(rad) |  dq(rad/s) |  tau_est |   cmd_tau";
    if (mode == CommandMode::MIT)
    {
        std::cout << " |    cmd_q |  cmd_dq |  cmd_kp |  cmd_kd";
    }
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------------";
    if (mode == CommandMode::MIT)
    {
        std::cout << "-----------------------------";
    }
    std::cout << std::endl;

    for (std::size_t i = 0; i < motor_count; ++i)
    {
        std::cout << std::setw(4) << i << " | " << std::setw(9) << q[i] << " | " << std::setw(10) << dq[i] << " | "
                  << std::setw(8) << tau[i] << " | " << std::setw(8) << commands.torque[i];
        if (mode == CommandMode::MIT)
        {
            std::cout << " | " << std::setw(8) << commands.position[i] << " | " << std::setw(7) << commands.velocity[i]
                      << " | " << std::setw(7) << commands.kp[i] << " | " << std::setw(7) << commands.kd[i];
        }
        std::cout << std::endl;
    }
}

} // namespace

int main(int argc, char *argv[])
{
    std::size_t motor_count = 16U;
    std::atomic<CommandMode> mode(CommandMode::Torque);
    bool monitor_enabled = false;
    int monitor_period_ms = 500;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--motors" && i + 1 < argc)
        {
            try
            {
                motor_count = static_cast<std::size_t>(std::stoul(argv[++i]));
            }
            catch (const std::exception &)
            {
                std::cerr << "Invalid value for --motors: " << argv[i] << std::endl;
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--mode" && i + 1 < argc)
        {
            const std::string value = ToLower(argv[++i]);
            if (value == "torque")
            {
                mode = CommandMode::Torque;
            }
            else if (value == "mit")
            {
                mode = CommandMode::MIT;
            }
            else
            {
                std::cerr << "Unknown mode: " << value << std::endl;
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--monitor")
        {
            monitor_enabled = true;
            if (i + 1 < argc)
            {
                char *end_ptr = nullptr;
                const long candidate = std::strtol(argv[i + 1], &end_ptr, 10);
                if (end_ptr != nullptr && *end_ptr == '\0' && candidate > 0)
                {
                    monitor_period_ms = static_cast<int>(candidate);
                    ++i;
                }
            }
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [--motors N] [--mode torque|mit] [--monitor [period_ms]]" << std::endl;
            return EXIT_SUCCESS;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (motor_count == 0U)
    {
        std::cerr << "Motor count must be greater than zero." << std::endl;
        return EXIT_FAILURE;
    }

    tita_robot robot(motor_count);

    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "Failed to switch the Titati MCU into SDK (direct torque) mode." << std::endl;
    }

    CommandBuffers command_buffers;
    command_buffers.position.resize(motor_count, 0.0);
    command_buffers.velocity.resize(motor_count, 0.0);
    command_buffers.kp.resize(motor_count, 0.0);
    command_buffers.kd.resize(motor_count, 0.0);
    command_buffers.torque.resize(motor_count, 0.0);

    std::mutex command_mutex;
    std::mutex io_mutex;

    std::atomic<bool> running(true);
    g_running_flag = &running;
    std::signal(SIGINT, SignalHandler);

    std::thread command_thread([&] () {
        while (running.load())
        {
            CommandMode active_mode = mode.load();
            CommandBuffers local_buffers;
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                local_buffers = command_buffers;
            }

            bool success = false;
            try
            {
                if (active_mode == CommandMode::Torque)
                {
                    success = robot.set_target_joint_t(local_buffers.torque);
                }
                else
                {
                    success = robot.set_target_joint_mit(
                        local_buffers.position, local_buffers.velocity, local_buffers.kp, local_buffers.kd, local_buffers.torque);
                }
            }
            catch (const std::exception &ex)
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Exception while sending command: " << ex.what() << std::endl;
            }

            if (!success)
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Failed to send command over CAN." << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // Send a final zero command on exit
        CommandBuffers zero_buffers;
        zero_buffers.position.resize(motor_count, 0.0);
        zero_buffers.velocity.resize(motor_count, 0.0);
        zero_buffers.kp.resize(motor_count, 0.0);
        zero_buffers.kd.resize(motor_count, 0.0);
        zero_buffers.torque.resize(motor_count, 0.0);

        try
        {
            if (mode.load() == CommandMode::Torque)
            {
                robot.set_target_joint_t(zero_buffers.torque);
            }
            else
            {
                robot.set_target_joint_mit(
                    zero_buffers.position, zero_buffers.velocity, zero_buffers.kp, zero_buffers.kd, zero_buffers.torque);
            }
        }
        catch (const std::exception &ex)
        {
            std::lock_guard<std::mutex> lock(io_mutex);
            std::cerr << "Exception while clearing command: " << ex.what() << std::endl;
        }
    });

    std::atomic<bool> monitor_toggle(monitor_enabled);
    std::atomic<int> monitor_period(monitor_period_ms);

    std::thread monitor_thread([&] () {
        while (running.load())
        {
            if (monitor_toggle.load())
            {
                CommandMode active_mode = mode.load();
                CommandBuffers local_buffers;
                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    local_buffers = command_buffers;
                }

                PrintMotorStates(robot, active_mode, local_buffers, io_mutex);
                std::this_thread::sleep_for(std::chrono::milliseconds(monitor_period.load()));
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    });

    PrintHelp(io_mutex);

    while (running.load())
    {
        {
            std::lock_guard<std::mutex> lock(io_mutex);
            std::cout << "> " << std::flush;
        }

        std::string line;
        if (!std::getline(std::cin, line))
        {
            running.store(false);
            break;
        }

        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command))
        {
            continue;
        }

        command = ToLower(command);

        if (command == "help" || command == "h")
        {
            PrintHelp(io_mutex);
        }
        else if (command == "exit" || command == "quit" || command == "q")
        {
            running.store(false);
        }
        else if (command == "status" || command == "s")
        {
            int requested_index = -1;
            CommandBuffers snapshot;
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                snapshot = command_buffers;
            }
            if (iss >> requested_index)
            {
                PrintMotorStates(robot, mode.load(), snapshot, io_mutex, requested_index);
            }
            else
            {
                PrintMotorStates(robot, mode.load(), snapshot, io_mutex);
            }
        }
        else if (command == "mode")
        {
            std::string value;
            if (!(iss >> value))
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Usage: mode torque|mit" << std::endl;
                continue;
            }

            value = ToLower(value);
            if (value == "torque")
            {
                mode.store(CommandMode::Torque);
            }
            else if (value == "mit")
            {
                mode.store(CommandMode::MIT);
            }
            else
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Unknown mode: " << value << std::endl;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(command_mutex);
                ResetCommands(command_buffers);
            }

            std::lock_guard<std::mutex> lock(io_mutex);
            std::cout << "Switched to " << ModeToString(mode.load()) << " mode and cleared all commands." << std::endl;
        }
        else if (command == "set")
        {
            int index_input = -1;
            if (!(iss >> index_input))
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Usage: set <index> <values...>" << std::endl;
                continue;
            }

            if (index_input < 0 || static_cast<std::size_t>(index_input) >= motor_count)
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Motor index must be within [0, " << motor_count - 1 << "]." << std::endl;
                continue;
            }

            std::vector<double> values;
            double value = 0.0;
            while (iss >> value)
            {
                values.push_back(value);
            }

            if (mode.load() == CommandMode::Torque)
            {
                if (values.empty())
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "Torque mode requires one value: set <index> <torque_Nm>." << std::endl;
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    command_buffers.torque[static_cast<std::size_t>(index_input)] = values[0];
                }

                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << "Motor " << index_input << " torque command set to " << values[0] << " Nm." << std::endl;
            }
            else
            {
                const double q_target = values.size() > 0 ? values[0] : 0.0;
                const double dq_target = values.size() > 1 ? values[1] : 0.0;
                const double kp_target = values.size() > 2 ? values[2] : 0.0;
                const double kd_target = values.size() > 3 ? values[3] : 0.0;
                const double tau_target = values.size() > 4 ? values[4] : 0.0;

                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    const std::size_t idx = static_cast<std::size_t>(index_input);
                    command_buffers.position[idx] = q_target;
                    command_buffers.velocity[idx] = dq_target;
                    command_buffers.kp[idx] = kp_target;
                    command_buffers.kd[idx] = kd_target;
                    command_buffers.torque[idx] = tau_target;
                }

                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << "Motor " << index_input << " MIT command set to (q=" << q_target << ", dq=" << dq_target
                          << ", kp=" << kp_target << ", kd=" << kd_target << ", tau=" << tau_target << ")." << std::endl;
            }
        }
        else if (command == "zero")
        {
            std::string target;
            if (!(iss >> target))
            {
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cerr << "Usage: zero <index|all>" << std::endl;
                continue;
            }

            if (target == "all")
            {
                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    ResetCommands(command_buffers);
                }

                std::lock_guard<std::mutex> io_lock(io_mutex);
                std::cout << "Cleared commands for all motors." << std::endl;
            }
            else
            {
                int index = 0;
                try
                {
                    index = std::stoi(target);
                }
                catch (const std::exception &)
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "Motor index must be an integer or 'all'." << std::endl;
                    continue;
                }

                if (index < 0 || static_cast<std::size_t>(index) >= motor_count)
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "Motor index must be within [0, " << motor_count - 1 << "]." << std::endl;
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    const std::size_t idx = static_cast<std::size_t>(index);
                    command_buffers.position[idx] = 0.0;
                    command_buffers.velocity[idx] = 0.0;
                    command_buffers.kp[idx] = 0.0;
                    command_buffers.kd[idx] = 0.0;
                    command_buffers.torque[idx] = 0.0;
                }

                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << "Cleared command for motor " << index << '.' << std::endl;
            }
        }
        else if (command == "monitor")
        {
            std::string token;
            if (!(iss >> token))
            {
                const bool new_state = !monitor_toggle.load();
                monitor_toggle.store(new_state);
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << (new_state ? "Enabled" : "Disabled") << " periodic state monitoring." << std::endl;
                continue;
            }

            token = ToLower(token);
            if (token == "off")
            {
                monitor_toggle.store(false);
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << "Disabled periodic state monitoring." << std::endl;
            }
            else
            {
                int period = 0;
                try
                {
                    period = std::stoi(token);
                }
                catch (const std::exception &)
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "Usage: monitor [period_ms|off]" << std::endl;
                    continue;
                }

                if (period <= 0)
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "Monitor period must be positive." << std::endl;
                    continue;
                }

                monitor_period.store(period);
                monitor_toggle.store(true);
                std::lock_guard<std::mutex> lock(io_mutex);
                std::cout << "Enabled periodic monitoring every " << period << " ms." << std::endl;
            }
        }
        else
        {
            std::lock_guard<std::mutex> lock(io_mutex);
            std::cerr << "Unknown command: " << command << std::endl;
        }
    }

    running.store(false);
    if (monitor_thread.joinable())
    {
        monitor_thread.join();
    }
    if (command_thread.joinable())
    {
        command_thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::cout << "Titati motor tester exited." << std::endl;
    }

    return EXIT_SUCCESS;
}
