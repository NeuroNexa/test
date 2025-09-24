#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "tita_hardware/tita_robot.hpp"

namespace
{
constexpr std::size_t kNumMotors = 16;
volatile std::sig_atomic_t g_shutdown_requested = 0;

void HandleSignal(int)
{
    g_shutdown_requested = 1;
}
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

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "Titati motor test utility" << std::endl;
    std::cout << "Commands:\n"
              << "  <index> <torque>        - set torque (Nm) on motor index 0-15\n"
              << "  mit <index> <pos> [kp] [kd] [vel] [tau]" << std::endl
              << "                        - single joint MIT test (rad, rad/s, Nm)\n"
              << "  read                    - print current joint states\n"
              << "  zero                    - zero all joint torques\n"
              << "  exit                    - stop test and restore MCU control\n";

    std::vector<double> torques(kNumMotors, 0.0);
    std::string line;
    bool running = true;
    while (running)
    {
        if (g_shutdown_requested)
        {
            break;
        }

        if (!std::getline(std::cin, line))
        {
            break;
        }

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
            if (iss >> kp_value)
            {
                if (iss >> kd_value)
                {
                    if (iss >> velocity_value)
                    {
                        if (iss >> torque_value)
                        {
                            // nothing else to parse
                        }
                    }
                }
            }

            auto q_targets = robot.get_joint_q();
            auto v_targets = std::vector<double>(kNumMotors, 0.0);
            auto kp_targets = std::vector<double>(kNumMotors, 0.0);
            auto kd_targets = std::vector<double>(kNumMotors, 0.0);
            auto tau_targets = std::vector<double>(kNumMotors, 0.0);

            q_targets[motor_index] = target_position;
            v_targets[motor_index] = velocity_value;
            kp_targets[motor_index] = kp_value;
            kd_targets[motor_index] = kd_value;
            tau_targets[motor_index] = torque_value;

            if (!robot.set_target_joint_mit(q_targets, v_targets, kp_targets, kd_targets, tau_targets))
            {
                std::cerr << "[ERROR] Failed to send MIT command." << std::endl;
            }
            else
            {
                std::cout << "Applied MIT target to motor " << motor_index
                          << ": q=" << target_position << " rad, kp=" << kp_value
                          << ", kd=" << kd_value << ", vel=" << velocity_value
                          << " rad/s, tau=" << torque_value << " Nm" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
