/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct Options
{
    std::string mode = "torque"; // torque, mit, or read
    std::size_t motor_index = 0;
    double torque = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double duration = 2.0; // seconds
    double rate = 200.0;   // Hz
    std::size_t num_motors = 16;
};

bool parse_arguments(int argc, char **argv, Options &options)
{
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i)
    {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0)
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for argument: " << key << std::endl;
                return false;
            }
            args[key] = argv[++i];
        }
        else
        {
            std::cerr << "Unknown argument: " << key << std::endl;
            return false;
        }
    }

    auto get_double = [&args](const std::string &name, double default_value) {
        auto it = args.find(name);
        if (it == args.end()) return default_value;
        return std::stod(it->second);
    };

    auto get_size = [&args](const std::string &name, std::size_t default_value) {
        auto it = args.find(name);
        if (it == args.end()) return default_value;
        return static_cast<std::size_t>(std::stoul(it->second));
    };

    auto it_mode = args.find("--mode");
    if (it_mode != args.end())
    {
        options.mode = it_mode->second;
    }

    options.motor_index = get_size("--motor", options.motor_index);
    options.torque = get_double("--torque", options.torque);
    options.position = get_double("--position", options.position);
    options.velocity = get_double("--velocity", options.velocity);
    options.kp = get_double("--kp", options.kp);
    options.kd = get_double("--kd", options.kd);
    options.duration = get_double("--duration", options.duration);
    options.rate = get_double("--rate", options.rate);
    options.num_motors = get_size("--num-motors", options.num_motors);

    if (options.mode != "torque" && options.mode != "mit" && options.mode != "read")
    {
        std::cerr << "Unsupported mode: " << options.mode << ". Use 'torque', 'mit', or 'read'." << std::endl;
        return false;
    }

    return true;
}

void print_usage(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [options]\n"
              << "  --mode [torque|mit|read]    Control mode (default: torque). 'read' only streams state.\n"
              << "  --motor <index>             Motor index to command (0-based)\n"
              << "  --torque <Nm>               Torque command (torque mode, default: 0.0)\n"
              << "  --position <rad>            MIT target position (default: 0.0)\n"
              << "  --velocity <rad/s>          MIT target velocity (default: 0.0)\n"
              << "  --kp <gain>                 MIT proportional gain (default: 0.0)\n"
              << "  --kd <gain>                 MIT derivative gain (default: 0.0)\n"
              << "  --duration <seconds>        Command duration (default: 2.0)\n"
              << "  --rate <Hz>                 Command update rate (default: 200.0)\n"
              << "  --num-motors <count>        Total number of motors (default: 16)\n"
              << std::endl;
}

void print_joint_state(const std::vector<double> &q,
                       const std::vector<double> &dq,
                       const std::vector<double> &tau)
{
    std::cout << "Current joint state:" << std::endl;
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        std::cout << "  [" << i << "] q=" << q[i]
                  << " rad, dq=" << dq[i]
                  << " rad/s, tau=" << tau[i] << " Nm" << std::endl;
    }
}
}

int main(int argc, char **argv)
{
    Options options;
    if (!parse_arguments(argc, argv, options))
    {
        print_usage(argv[0]);
        return 1;
    }

    if (options.mode != "read" && options.motor_index >= options.num_motors)
    {
        std::cerr << "Motor index " << options.motor_index
                  << " is out of range for " << options.num_motors << " motors." << std::endl;
        return 1;
    }

    tita_robot robot(options.num_motors);
    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "Failed to switch Titati to SDK control mode. Ensure CAN interface is ready." << std::endl;
    }

    auto q = robot.get_joint_q();
    auto dq = robot.get_joint_v();
    auto tau = robot.get_joint_t();
    print_joint_state(q, dq, tau);

    const std::chrono::duration<double> interval(1.0 / options.rate);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_print = start;

    std::vector<double> torques(options.num_motors, 0.0);
    std::vector<double> positions(options.num_motors, 0.0);
    std::vector<double> velocities(options.num_motors, 0.0);
    std::vector<double> kp(options.num_motors, 0.0);
    std::vector<double> kd(options.num_motors, 0.0);

    if (options.mode != "read")
    {
        torques[options.motor_index] = options.torque;
        positions[options.motor_index] = options.position;
        velocities[options.motor_index] = options.velocity;
        kp[options.motor_index] = options.kp;
        kd[options.motor_index] = options.kd;
    }

    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(options.duration))
    {
        if (options.mode == "torque")
        {
            robot.set_target_joint_t(torques);
        }
        else if (options.mode == "mit")
        {
            robot.set_target_joint_mit(positions, velocities, kp, kd, torques);
        }
        else
        {
            std::this_thread::sleep_for(interval);
            q = robot.get_joint_q();
            dq = robot.get_joint_v();
            tau = robot.get_joint_t();
            std::cout << "--- sample ---" << std::endl;
            print_joint_state(q, dq, tau);
            continue;
        }

        std::this_thread::sleep_for(interval);

        // Print state at 10 Hz to avoid flooding
        auto now = std::chrono::steady_clock::now();
        if (now - last_print >= std::chrono::milliseconds(100))
        {
            q = robot.get_joint_q();
            dq = robot.get_joint_v();
            tau = robot.get_joint_t();
            std::cout << "--- sample ---" << std::endl;
            print_joint_state(q, dq, tau);
            last_print = now;
        }
    }

    std::vector<double> zero(options.num_motors, 0.0);
    if (options.mode != "read")
    {
        robot.set_target_joint_t(zero);
    }
    robot.set_motors_sdk(false);

    std::cout << "Motor test completed." << std::endl;
    return 0;
}
