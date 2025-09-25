/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tita_robot/tita_robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr size_t kTitatiDofs = 16;
constexpr std::chrono::milliseconds kControlPeriod{5};
constexpr std::chrono::milliseconds kPrintPeriod{500};
}

int main()
{
    tita_robot robot(kTitatiDofs);
    if (!robot.set_motors_sdk(true))
    {
        std::cout << "[WARNING] Failed to enable SDK control. Commands may not take effect." << std::endl;
    }

    std::vector<double> torque_command(kTitatiDofs, 0.0);
    std::vector<double> mit_q(kTitatiDofs, 0.0);
    std::vector<double> mit_dq(kTitatiDofs, 0.0);
    std::vector<double> mit_kp(kTitatiDofs, 0.0);
    std::vector<double> mit_kd(kTitatiDofs, 0.0);
    std::vector<double> mit_tau(kTitatiDofs, 0.0);

    std::mutex command_mutex;
    std::atomic<bool> running{true};
    std::atomic<bool> use_mit{false};
    std::atomic<bool> print_states{true};

    std::thread control_thread([&]() {
        while (running.load())
        {
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                if (use_mit.load())
                {
                    robot.set_target_joint_mit(mit_q, mit_dq, mit_kp, mit_kd, mit_tau);
                }
                else
                {
                    robot.set_target_joint_t(torque_command);
                }
            }
            std::this_thread::sleep_for(kControlPeriod);
        }
        std::vector<double> zero(kTitatiDofs, 0.0);
        robot.set_target_joint_t(zero);
        robot.set_motors_sdk(false);
    });

    std::thread print_thread([&]() {
        while (running.load())
        {
            if (print_states.load())
            {
                auto q = robot.get_joint_q();
                auto v = robot.get_joint_v();
                auto tau = robot.get_joint_t();
                auto imu_q = robot.get_imu_quaternion();
                auto imu_gyro = robot.get_imu_angular_velocity();
                auto imu_acc = robot.get_imu_acceleration();

                std::cout << "\n================ Titati Motor Status ================\n";
                for (size_t i = 0; i < q.size(); ++i)
                {
                    std::cout << std::fixed << std::setprecision(3)
                              << "Motor " << std::setw(2) << i << ": q=" << std::setw(8) << q[i]
                              << " dq=" << std::setw(8) << v[i]
                              << " tau=" << std::setw(8) << tau[i] << std::endl;
                }
                std::cout << "IMU quat (x y z w): " << imu_q[0] << " " << imu_q[1] << " " << imu_q[2] << " " << imu_q[3] << std::endl;
                std::cout << "IMU gyro (rad/s):  " << imu_gyro[0] << " " << imu_gyro[1] << " " << imu_gyro[2] << std::endl;
                std::cout << "IMU acc (m/s^2):   " << imu_acc[0] << " " << imu_acc[1] << " " << imu_acc[2] << std::endl;
            }
            std::this_thread::sleep_for(kPrintPeriod);
        }
    });

    std::cout << "Simple Titati motor test started." << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  help                           - Show command list" << std::endl;
    std::cout << "  torque <id> <tau>              - Apply torque command to motor <id> (others zero)" << std::endl;
    std::cout << "  mit <id> <q> <dq> <kp> <kd> <tau> - Apply MIT command to motor <id> (others zero)" << std::endl;
    std::cout << "  hold                           - Keep current command without zeroing" << std::endl;
    std::cout << "  zero                           - Zero all commands" << std::endl;
    std::cout << "  print on/off                   - Toggle periodic state printing" << std::endl;
    std::cout << "  status                         - Print one-shot state snapshot" << std::endl;
    std::cout << "  quit                           - Exit test" << std::endl;

    std::string line;
    while (std::cout << "\n> " && std::getline(std::cin, line))
    {
        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd))
        {
            continue;
        }

        if (cmd == "help")
        {
            std::cout << "Commands:" << std::endl;
            std::cout << "  torque <id> <tau>" << std::endl;
            std::cout << "  mit <id> <q> <dq> <kp> <kd> <tau>" << std::endl;
            std::cout << "  hold" << std::endl;
            std::cout << "  zero" << std::endl;
            std::cout << "  print on/off" << std::endl;
            std::cout << "  status" << std::endl;
            std::cout << "  quit" << std::endl;
        }
        else if (cmd == "torque")
        {
            size_t id;
            double tau;
            if (!(iss >> id >> tau) || id >= kTitatiDofs)
            {
                std::cout << "Invalid torque command. Usage: torque <id> <tau>" << std::endl;
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                std::fill(torque_command.begin(), torque_command.end(), 0.0);
                torque_command[id] = tau;
                use_mit.store(false);
            }
            std::cout << "Torque command sent to motor " << id << std::endl;
        }
        else if (cmd == "mit")
        {
            size_t id;
            double q, dq, kp, kd, tau;
            if (!(iss >> id >> q >> dq >> kp >> kd >> tau) || id >= kTitatiDofs)
            {
                std::cout << "Invalid MIT command. Usage: mit <id> <q> <dq> <kp> <kd> <tau>" << std::endl;
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                std::fill(mit_q.begin(), mit_q.end(), 0.0);
                std::fill(mit_dq.begin(), mit_dq.end(), 0.0);
                std::fill(mit_kp.begin(), mit_kp.end(), 0.0);
                std::fill(mit_kd.begin(), mit_kd.end(), 0.0);
                std::fill(mit_tau.begin(), mit_tau.end(), 0.0);
                mit_q[id] = q;
                mit_dq[id] = dq;
                mit_kp[id] = kp;
                mit_kd[id] = kd;
                mit_tau[id] = tau;
                use_mit.store(true);
            }
            std::cout << "MIT command sent to motor " << id << std::endl;
        }
        else if (cmd == "zero")
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            std::fill(torque_command.begin(), torque_command.end(), 0.0);
            std::fill(mit_q.begin(), mit_q.end(), 0.0);
            std::fill(mit_dq.begin(), mit_dq.end(), 0.0);
            std::fill(mit_kp.begin(), mit_kp.end(), 0.0);
            std::fill(mit_kd.begin(), mit_kd.end(), 0.0);
            std::fill(mit_tau.begin(), mit_tau.end(), 0.0);
            std::cout << "Commands zeroed." << std::endl;
        }
        else if (cmd == "hold")
        {
            std::cout << "Holding current command output." << std::endl;
        }
        else if (cmd == "print")
        {
            std::string state;
            if (!(iss >> state))
            {
                std::cout << "Usage: print on/off" << std::endl;
                continue;
            }
            if (state == "on")
            {
                print_states.store(true);
            }
            else if (state == "off")
            {
                print_states.store(false);
            }
            else
            {
                std::cout << "Usage: print on/off" << std::endl;
            }
        }
        else if (cmd == "status")
        {
            auto q = robot.get_joint_q();
            auto v = robot.get_joint_v();
            auto tau = robot.get_joint_t();
            std::cout << "Instant snapshot:" << std::endl;
            for (size_t i = 0; i < q.size(); ++i)
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "Motor " << std::setw(2) << i << ": q=" << std::setw(8) << q[i]
                          << " dq=" << std::setw(8) << v[i]
                          << " tau=" << std::setw(8) << tau[i] << std::endl;
            }
        }
        else if (cmd == "quit" || cmd == "exit")
        {
            break;
        }
        else
        {
            std::cout << "Unknown command. Type 'help' for options." << std::endl;
        }
    }

    running.store(false);
    if (control_thread.joinable()) control_thread.join();
    if (print_thread.joinable()) print_thread.join();

    std::cout << "Titati motor test finished." << std::endl;
    return 0;
}

