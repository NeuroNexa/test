/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "titati_hw/tita_robot.hpp"
#include "rl_sdk.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct Options
{
    std::string mode = "torque";  // torque, mit, direct
    int motor_count = 16;
    double amplitude = 2.0;        // Nm for torque, rad for mit offset
    double duration = 1.0;         // seconds per joint
    double kp_leg = 70.0;
    double kd_leg = 5.0;
    double kp_wheel = 0.0;
    double kd_wheel = 0.5;
    bool enable_sdk = true;
};

void PrintUsage()
{
    std::cout << "titati_motor_test --mode <torque|mit|direct> [--motors N] [--amplitude A] [--duration D]" << std::endl;
    std::cout << "  torque : apply sequential torque pulses to each motor" << std::endl;
    std::cout << "  mit    : command MIT position offsets for each motor" << std::endl;
    std::cout << "  direct : switch MCU control mode (use --amplitude 0 to switch back to MCU)" << std::endl;
}

Options ParseOptions(int argc, char **argv)
{
    Options opts;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if ((arg == "--mode" || arg == "-m") && i + 1 < argc)
        {
            opts.mode = argv[++i];
        }
        else if (arg == "--motors" && i + 1 < argc)
        {
            opts.motor_count = std::stoi(argv[++i]);
        }
        else if (arg == "--amplitude" && i + 1 < argc)
        {
            opts.amplitude = std::stod(argv[++i]);
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            opts.duration = std::stod(argv[++i]);
        }
        else if (arg == "--disable-sdk")
        {
            opts.enable_sdk = false;
        }
        else if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            std::exit(0);
        }
    }
    return opts;
}

std::vector<double> DefaultPose(int motor_count)
{
    std::vector<double> base = {
        0.00, 0.40, -0.917,
        0.00, 0.40, -0.917,
        0.00, -0.40, 0.917,
        0.00, -0.40, 0.917,
        0.00, 0.00, 0.00, 0.00
    };
    if (motor_count <= static_cast<int>(base.size()))
    {
        base.resize(motor_count);
    }
    else
    {
        base.resize(motor_count, 0.0);
    }
    return base;
}

std::vector<double> DefaultGain(double leg_gain, double wheel_gain, int motor_count)
{
    std::vector<double> gains(motor_count, leg_gain);
    if (motor_count >= 16)
    {
        for (int i = 12; i < motor_count; ++i)
        {
            gains[i] = wheel_gain;
        }
    }
    return gains;
}

void RunTorqueTest(titati::hardware::TitatiRobot &robot, const Options &opts)
{
    std::vector<double> tau(opts.motor_count, 0.0);
    auto rest = tau;
    const auto period = std::chrono::milliseconds(5);

    for (int i = 0; i < opts.motor_count; ++i)
    {
        std::cout << LOGGER::INFO << "Torque test joint " << i << " -> " << opts.amplitude << " Nm" << std::endl;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(opts.duration))
        {
            std::fill(tau.begin(), tau.end(), 0.0);
            tau[i] = opts.amplitude;
            robot.set_target_joint_t(tau);
            std::this_thread::sleep_for(period);
        }
        robot.set_target_joint_t(rest);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    robot.set_target_joint_t(rest);
}

void RunMitTest(titati::hardware::TitatiRobot &robot, const Options &opts)
{
    const auto base_pos = DefaultPose(opts.motor_count);
    const auto base_kp = DefaultGain(opts.kp_leg, opts.kp_wheel, opts.motor_count);
    const auto base_kd = DefaultGain(opts.kd_leg, opts.kd_wheel, opts.motor_count);
    std::vector<double> command_pos = base_pos;
    std::vector<double> command_vel(opts.motor_count, 0.0);
    std::vector<double> command_tau(opts.motor_count, 0.0);

    const auto period = std::chrono::milliseconds(5);

    for (int i = 0; i < opts.motor_count; ++i)
    {
        std::cout << LOGGER::INFO << "MIT test joint " << i << " -> offset " << opts.amplitude << " rad" << std::endl;
        command_pos = base_pos;
        command_pos[i] += opts.amplitude;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(opts.duration))
        {
            robot.set_target_joint_mit(command_pos, command_vel, base_kp, base_kd, command_tau);
            std::this_thread::sleep_for(period);
        }
        robot.set_target_joint_mit(base_pos, command_vel, base_kp, base_kd, command_tau);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    robot.set_target_joint_mit(base_pos, command_vel, base_kp, base_kd, command_tau);
}

}  // namespace

int main(int argc, char **argv)
{
    auto opts = ParseOptions(argc, argv);

    if (opts.mode != "torque" && opts.mode != "mit" && opts.mode != "direct")
    {
        std::cerr << LOGGER::ERROR << "Unknown mode: " << opts.mode << std::endl;
        PrintUsage();
        return 1;
    }

    titati::hardware::TitatiRobot robot(opts.motor_count);

    if (opts.mode == "direct")
    {
        bool ok = robot.set_motors_sdk(opts.enable_sdk);
        std::cout << LOGGER::INFO << (opts.enable_sdk ? "Enable" : "Disable") << " direct SDK: " << (ok ? "OK" : "FAILED") << std::endl;
        return ok ? 0 : 1;
    }

    robot.set_motors_sdk(true);

    if (opts.mode == "torque")
    {
        RunTorqueTest(robot, opts);
    }
    else if (opts.mode == "mit")
    {
        RunMitTest(robot, opts);
    }

    robot.set_target_joint_t(std::vector<double>(opts.motor_count, 0.0));
    std::cout << std::endl << LOGGER::INFO << "Test finished." << std::endl;
    return 0;
}
