/*
 * Titati motor diagnostic tool
 */

#include "titati_sdk/titati_robot.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct CommandOptions
{
    bool read_once{false};
    bool monitor{false};
    bool use_mit{false};
    bool send_command{false};
    int motor_id{-1};
    double torque{0.0};
    double position{0.0};
    double velocity{0.0};
    double kp{0.0};
    double kd{0.0};
    double duration{1.0};
    double interval{0.01};
};

void PrintUsage()
{
    std::cout << "Titati Motor Test Utility" << std::endl;
    std::cout << "Usage:\n"
              << "  titati_motor_test --read" << std::endl
              << "  titati_motor_test --monitor" << std::endl
              << "  titati_motor_test --mode torque --id <0-15> --tau <Nm> [--duration s]" << std::endl
              << "  titati_motor_test --mode mit --id <0-15> --pos <rad> --vel <rad/s> --kp <Nm/rad> --kd <Nm*s/rad> --tau <Nm>"
              << " [--duration s]" << std::endl
              << "Common optional arguments:" << std::endl
              << "  --interval <s>    Command resend interval (default 0.01)" << std::endl
              << "  --help            Show this help message" << std::endl;
}

bool ParseArguments(int argc, char **argv, CommandOptions &options)
{
    if (argc == 1)
    {
        PrintUsage();
        return false;
    }

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--help")
        {
            PrintUsage();
            return false;
        }
        else if (arg == "--read")
        {
            options.read_once = true;
        }
        else if (arg == "--monitor")
        {
            options.monitor = true;
        }
        else if (arg == "--mode" && i + 1 < argc)
        {
            std::string mode = argv[++i];
            if (mode == "torque")
            {
                options.use_mit = false;
                options.send_command = true;
            }
            else if (mode == "mit")
            {
                options.use_mit = true;
                options.send_command = true;
            }
            else
            {
                std::cerr << "Unknown mode: " << mode << std::endl;
                return false;
            }
        }
        else if (arg == "--id" && i + 1 < argc)
        {
            options.motor_id = std::stoi(argv[++i]);
        }
        else if ((arg == "--tau" || arg == "--torque") && i + 1 < argc)
        {
            options.torque = std::stod(argv[++i]);
        }
        else if (arg == "--pos" && i + 1 < argc)
        {
            options.position = std::stod(argv[++i]);
        }
        else if (arg == "--vel" && i + 1 < argc)
        {
            options.velocity = std::stod(argv[++i]);
        }
        else if (arg == "--kp" && i + 1 < argc)
        {
            options.kp = std::stod(argv[++i]);
        }
        else if (arg == "--kd" && i + 1 < argc)
        {
            options.kd = std::stod(argv[++i]);
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            options.duration = std::stod(argv[++i]);
        }
        else if (arg == "--interval" && i + 1 < argc)
        {
            options.interval = std::stod(argv[++i]);
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return false;
        }
    }

    if (options.send_command && (options.motor_id < 0 || options.motor_id >= 16))
    {
        std::cerr << "Motor id must be between 0 and 15." << std::endl;
        return false;
    }
    if (options.interval <= 0.0)
    {
        options.interval = 0.01;
    }
    if (options.duration < 0.0)
    {
        options.duration = 0.0;
    }
    return true;
}

void PrintState(const tita_robot &robot)
{
    auto q = robot.get_joint_q();
    auto dq = robot.get_joint_v();
    auto tau = robot.get_joint_t();
    auto status = robot.get_joint_status();

    std::cout << "Index | Position(rad) | Velocity(rad/s) | Torque(Nm) | Status" << std::endl;
    for (size_t i = 0; i < q.size(); ++i)
    {
        int st = (i < status.size()) ? status[i] : 0;
        std::cout << std::setw(5) << i << " | "
                  << std::setw(12) << std::fixed << std::setprecision(4) << q[i] << " | "
                  << std::setw(15) << std::fixed << std::setprecision(4) << dq[i] << " | "
                  << std::setw(11) << std::fixed << std::setprecision(4) << tau[i] << " | "
                  << st << std::endl;
    }
}

} // namespace

int main(int argc, char **argv)
{
    CommandOptions options;
    if (!ParseArguments(argc, argv, options))
    {
        return 0;
    }

    tita_robot robot(16);
    robot.set_motors_sdk(true);

    if (!robot.wait_for_feedback(std::chrono::milliseconds(500)))
    {
        std::cerr << "Warning: timed out waiting for Titati motor feedback. Check CAN routing and power." << std::endl;
        auto missing = robot.get_missing_feedback_indices();
        if (!missing.empty())
        {
            std::cerr << "Missing motor indexes:";
            for (auto idx : missing)
            {
                std::cerr << ' ' << idx;
            }
            std::cerr << std::endl;
        }
    }

    if (options.read_once)
    {
        PrintState(robot);
    }

    if (options.monitor)
    {
        while (true)
        {
            PrintState(robot);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    if (!options.send_command)
    {
        return 0;
    }

    robot.wait_for_feedback(std::chrono::milliseconds(200));

    std::vector<double> q(16, 0.0);
    std::vector<double> dq(16, 0.0);
    std::vector<double> kp(16, 0.0);
    std::vector<double> kd(16, 0.0);
    std::vector<double> tau(16, 0.0);

    if (options.use_mit)
    {
        q[options.motor_id] = options.position;
        dq[options.motor_id] = options.velocity;
        kp[options.motor_id] = options.kp;
        kd[options.motor_id] = options.kd;
        tau[options.motor_id] = options.torque;
    }
    else
    {
        tau[options.motor_id] = options.torque;
    }

    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        if (options.use_mit)
        {
            robot.set_target_joint_mit(q, dq, kp, kd, tau);
        }
        else
        {
            robot.set_target_joint_t(tau);
        }

        if (options.duration == 0.0)
        {
            break;
        }

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= options.duration)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(options.interval));
    }

    std::vector<double> zeros(16, 0.0);
    if (options.use_mit)
    {
        robot.set_target_joint_mit(zeros, zeros, zeros, zeros, zeros);
    }
    else
    {
        robot.set_target_joint_t(zeros);
    }

    PrintState(robot);
    return 0;
}
