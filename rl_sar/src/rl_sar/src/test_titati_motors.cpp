#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "tita_hardware/tita_robot.hpp"

namespace
{
constexpr std::size_t kNumMotors = 16;
constexpr double kHoldKp = 20.0;
constexpr double kHoldKd = 1.0;
constexpr std::chrono::milliseconds kCommandPeriod{5};
void PrintState(const tita_robot &robot)
{
    auto positions = robot.get_joint_q();
    auto velocities = robot.get_joint_v();
    auto torques = robot.get_joint_t();

    std::cout << "Current joint state:\n";
    for (std::size_t i = 0; i < positions.size(); ++i)
    {
        std::cout << "  Motor " << i
                  << ": q=" << positions[i]
                  << " rad, dq=" << velocities[i]
                  << " rad/s, tau=" << torques[i]
                  << " Nm" << std::endl;
    }
}
} // namespace

int main()
{
    tita_robot robot(kNumMotors);
    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "[ERROR] Failed to enable SDK control mode. Abort." << std::endl;
        return 1;
    }

    std::cout << "Titati motor test utility" << std::endl;
    std::cout << "Commands:\n"
              << "  <index> <torque>        - set torque (Nm) on motor index 0-15\n"
              << "  mit <index> <pos> [kp] [kd] [vel] [tau] [hold_ms]" << std::endl
              << "                        - single joint MIT test (rad, rad/s, Nm, ms)\n"
              << "  read                    - print current joint states\n"
              << "  zero                    - zero all joint torques\n"
              << "  exit                    - stop test and restore MCU control\n";

    std::vector<double> torques(kNumMotors, 0.0);
    std::string line;
    bool running = true;
    while (running && std::getline(std::cin, line))
    {
        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command))
        {
            continue;
        }

        if (command == "read")
        {
            PrintState(robot);
            continue;
        }

        if (command == "zero")
        {
            std::fill(torques.begin(), torques.end(), 0.0);
            robot.set_target_joint_t(torques);
            std::cout << "All torques cleared." << std::endl;
            continue;
        }

        if (command == "mit")
        {
            std::size_t motor_index;
            double target_position;
            if (!(iss >> motor_index >> target_position))
            {
                std::cerr << "[WARNING] Usage: mit <index> <pos> [kp] [kd] [vel] [tau]" << std::endl;
                continue;
            }

            if (motor_index >= kNumMotors)
            {
                std::cerr << "[WARNING] Motor index out of range. Expected 0-" << kNumMotors - 1 << std::endl;
                continue;
            }

            double kp_value = 50.0;
            double kd_value = 5.0;
            double velocity_value = 0.0;
            double torque_value = 0.0;
            int hold_duration_ms = 1500;

            std::vector<double> optional_values;
            double parsed_optional = 0.0;
            while (iss >> parsed_optional)
            {
                optional_values.push_back(parsed_optional);
            }

            if (!optional_values.empty())
            {
                kp_value = optional_values[0];
            }
            if (optional_values.size() > 1)
            {
                kd_value = optional_values[1];
            }
            if (optional_values.size() > 2)
            {
                velocity_value = optional_values[2];
            }
            if (optional_values.size() > 3)
            {
                torque_value = optional_values[3];
            }
            if (optional_values.size() > 4)
            {
                hold_duration_ms = static_cast<int>(optional_values[4]);
            }

            if (hold_duration_ms < 0)
            {
                hold_duration_ms = 0;
            }

            auto measured_positions = robot.get_joint_q();

            auto q_targets = measured_positions;
            auto v_targets = std::vector<double>(kNumMotors, 0.0);
            auto kp_targets = std::vector<double>(kNumMotors, kHoldKp);
            auto kd_targets = std::vector<double>(kNumMotors, kHoldKd);
            auto tau_targets = std::vector<double>(kNumMotors, 0.0);

            q_targets[motor_index] = target_position;
            v_targets[motor_index] = velocity_value;
            kp_targets[motor_index] = kp_value;
            kd_targets[motor_index] = kd_value;
            tau_targets[motor_index] = torque_value;

            const auto start = std::chrono::steady_clock::now();
            std::size_t successful_frames = 0;
            std::size_t failed_frames = 0;

            while (true)
            {
                const bool ok = robot.set_target_joint_mit(q_targets, v_targets, kp_targets, kd_targets, tau_targets);
                if (!ok)
                {
                    ++failed_frames;
                    std::cerr << "[ERROR] Failed to send MIT command frame." << std::endl;
                    break;
                }

                ++successful_frames;

                if (hold_duration_ms == 0)
                {
                    break;
                }

                const auto elapsed = std::chrono::steady_clock::now() - start;
                if (elapsed >= std::chrono::milliseconds(hold_duration_ms))
                {
                    break;
                }

                std::this_thread::sleep_for(kCommandPeriod);
            }

            auto final_positions = robot.get_joint_q();
            const double initial_position = measured_positions[motor_index];
            const double final_position = final_positions[motor_index];

            std::cout << std::fixed << std::setprecision(4)
                      << "Applied MIT target to motor " << motor_index
                      << ": q_target=" << target_position
                      << " rad, kp=" << kp_value
                      << ", kd=" << kd_value
                      << ", vel=" << velocity_value
                      << " rad/s, tau=" << torque_value
                      << " Nm, hold=" << hold_duration_ms << " ms" << std::endl;
            std::cout << "  Position feedback: initial=" << initial_position
                      << " rad, final=" << final_position
                      << " rad, delta=" << (final_position - initial_position) << " rad" << std::endl;
            std::cout << "  MIT frames sent: success=" << successful_frames
                      << ", failed=" << failed_frames << std::endl;

            continue;
        }

        if (command == "exit" || command == "quit")
        {
            running = false;
            break;
        }

        std::size_t motor_index;
        double torque_value;
        try
        {
            motor_index = static_cast<std::size_t>(std::stoul(command));
        }
        catch (const std::exception &)
        {
            std::cerr << "[WARNING] Invalid command: " << command << std::endl;
            continue;
        }

        if (!(iss >> torque_value))
        {
            std::cerr << "[WARNING] Torque value required after motor index." << std::endl;
            continue;
        }

        if (motor_index >= torques.size())
        {
            std::cerr << "[WARNING] Motor index out of range. Expected 0-" << torques.size() - 1 << std::endl;
            continue;
        }

        torques.assign(torques.size(), 0.0);
        torques[motor_index] = torque_value;
        if (!robot.set_target_joint_t(torques))
        {
            std::cerr << "[ERROR] Failed to send torque command." << std::endl;
        }
        else
        {
            std::cout << "Applied torque " << torque_value << " Nm to motor " << motor_index << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    robot.set_target_joint_t(std::vector<double>(kNumMotors, 0.0));
    robot.set_motors_sdk(false);
    std::cout << "Test finished." << std::endl;
    return 0;
}
