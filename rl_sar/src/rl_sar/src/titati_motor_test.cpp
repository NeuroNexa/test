#include "titati_sdk/tita_robot.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
void PrintState(const titati::TitatiRobot &robot)
{
    auto q = robot.get_joint_q();
    auto dq = robot.get_joint_v();
    auto tau = robot.get_joint_t();

    std::cout << "\n=== Titati Motor State ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        std::cout << "Motor " << std::setw(2) << i
                  << " | q: " << std::setw(9) << q[i]
                  << " rad | dq: " << std::setw(9) << dq[i]
                  << " rad/s | tau: " << std::setw(9) << tau[i] << " Nm" << std::endl;
    }
    std::cout << std::endl;
}

void SendTorque(titati::TitatiRobot &robot, std::size_t index, double torque)
{
    std::vector<double> torques(robot.motor_count(), 0.0);
    if (index < torques.size())
    {
        torques[index] = torque;
    }
    robot.set_target_joint_t(torques);
}

void SendMIT(titati::TitatiRobot &robot, std::size_t index, double q, double dq, double kp, double kd, double tau)
{
    std::size_t count = robot.motor_count();
    std::vector<double> target_q(count, 0.0);
    std::vector<double> target_dq(count, 0.0);
    std::vector<double> target_kp(count, 0.0);
    std::vector<double> target_kd(count, 0.0);
    std::vector<double> target_tau(count, 0.0);

    if (index < count)
    {
        target_q[index] = q;
        target_dq[index] = dq;
        target_kp[index] = kp;
        target_kd[index] = kd;
        target_tau[index] = tau;
    }
    robot.set_target_joint_mit(target_q, target_dq, target_kp, target_kd, target_tau);
}
}

int main(int argc, char **argv)
{
    std::size_t motor_count = 16;
    if (argc > 1)
    {
        motor_count = static_cast<std::size_t>(std::stoul(argv[1]));
    }

    titati::TitatiRobot robot(motor_count);
    if (!robot.set_motors_sdk(true))
    {
        std::cerr << "[WARNING] Failed to switch Titati into SDK control mode. Commands may not take effect." << std::endl;
    }

    std::cout << "Titati motor test utility" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  status                -> print current joint state" << std::endl;
    std::cout << "  torque <id> <tau>     -> apply feed-forward torque (Nm) to a motor" << std::endl;
    std::cout << "  mit <id> <q> <dq> <kp> <kd> <tau> -> send MIT command to a motor" << std::endl;
    std::cout << "  zero                  -> zero torque on all motors" << std::endl;
    std::cout << "  watch <seconds>       -> continuously print state" << std::endl;
    std::cout << "  help                  -> print this help" << std::endl;
    std::cout << "  exit / quit           -> leave test program" << std::endl;

    std::string line;
    while (true)
    {
        std::cout << "tester> " << std::flush;
        if (!std::getline(std::cin, line))
        {
            break;
        }

        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd))
        {
            continue;
        }

        if (cmd == "status")
        {
            PrintState(robot);
        }
        else if (cmd == "watch")
        {
            double seconds = 5.0;
            iss >> seconds;
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds)
            {
                PrintState(robot);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
        else if (cmd == "torque")
        {
            std::size_t id;
            double tau;
            if (iss >> id >> tau)
            {
                if (id >= robot.motor_count())
                {
                    std::cout << "[ERROR] Motor id out of range" << std::endl;
                    continue;
                }
                SendTorque(robot, id, tau);
                std::cout << "Applied torque command to motor " << id << std::endl;
            }
            else
            {
                std::cout << "Usage: torque <id> <tau>" << std::endl;
            }
        }
        else if (cmd == "mit")
        {
            std::size_t id;
            double q, dq, kp, kd, tau;
            if (iss >> id >> q >> dq >> kp >> kd >> tau)
            {
                if (id >= robot.motor_count())
                {
                    std::cout << "[ERROR] Motor id out of range" << std::endl;
                    continue;
                }
                SendMIT(robot, id, q, dq, kp, kd, tau);
                std::cout << "Sent MIT command to motor " << id << std::endl;
            }
            else
            {
                std::cout << "Usage: mit <id> <q> <dq> <kp> <kd> <tau>" << std::endl;
            }
        }
        else if (cmd == "zero")
        {
            std::vector<double> torques(robot.motor_count(), 0.0);
            robot.set_target_joint_t(torques);
            std::cout << "All torques cleared" << std::endl;
        }
        else if (cmd == "help")
        {
            std::cout << "Commands:" << std::endl;
            std::cout << "  status                -> print current joint state" << std::endl;
            std::cout << "  torque <id> <tau>     -> apply feed-forward torque (Nm) to a motor" << std::endl;
            std::cout << "  mit <id> <q> <dq> <kp> <kd> <tau> -> send MIT command to a motor" << std::endl;
            std::cout << "  zero                  -> zero torque on all motors" << std::endl;
            std::cout << "  watch <seconds>       -> continuously print state" << std::endl;
            std::cout << "  help                  -> print this help" << std::endl;
            std::cout << "  exit / quit           -> leave test program" << std::endl;
        }
        else if (cmd == "exit" || cmd == "quit")
        {
            break;
        }
        else
        {
            std::cout << "Unknown command. Type 'help' for usage." << std::endl;
        }
    }

    std::vector<double> torques(robot.motor_count(), 0.0);
    robot.set_target_joint_t(torques);
    robot.set_motors_sdk(false);
    std::cout << "Tester exit" << std::endl;
    return 0;
}

