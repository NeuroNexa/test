/*
 * Simple Titati motor exerciser.
 */

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "titati/force_direct_manager.hpp"
#include "titati/tita_robot/tita_robot.hpp"

namespace
{
volatile std::sig_atomic_t g_running = 1;

void signal_handler(int)
{
    g_running = 0;
}

struct Options
{
    size_t joint = 0;
    std::string mode = "torque";
    double value = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double duration = 2.0;
};

void print_usage()
{
    std::cout << "Usage: titati_motor_test [--joint <index>] [--mode torque|mit] [--value <v>] [--kp <kp>] [--kd <kd>] [--duration <s>]" << std::endl;
    std::cout << "  --joint     Joint index [0-15] (default: 0)" << std::endl;
    std::cout << "  --mode      Control mode: torque (Nm) or mit (rad) (default: torque)" << std::endl;
    std::cout << "  --value     Torque (Nm) in torque mode, position (rad) in MIT mode" << std::endl;
    std::cout << "  --kp        MIT proportional gain (used only in MIT mode)" << std::endl;
    std::cout << "  --kd        MIT derivative gain (used only in MIT mode)" << std::endl;
    std::cout << "  --duration  Command duration in seconds (default: 2.0)" << std::endl;
}

bool parse_arguments(int argc, char **argv, Options &opts)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--joint" && i + 1 < argc)
        {
            opts.joint = static_cast<size_t>(std::stoul(argv[++i]));
        }
        else if (arg == "--mode" && i + 1 < argc)
        {
            opts.mode = argv[++i];
        }
        else if (arg == "--value" && i + 1 < argc)
        {
            opts.value = std::stod(argv[++i]);
        }
        else if (arg == "--kp" && i + 1 < argc)
        {
            opts.kp = std::stod(argv[++i]);
        }
        else if (arg == "--kd" && i + 1 < argc)
        {
            opts.kd = std::stod(argv[++i]);
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            opts.duration = std::stod(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h")
        {
            print_usage();
            return false;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage();
            return false;
        }
    }

    if (opts.mode != "torque" && opts.mode != "mit")
    {
        std::cerr << "Invalid mode: " << opts.mode << std::endl;
        return false;
    }
    if (opts.joint >= 16)
    {
        std::cerr << "Joint index out of range: " << opts.joint << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    Options opts;
    if (!parse_arguments(argc, argv, opts))
    {
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    titati::ForceDirectManager manager;
    manager.start();

    tita_robot robot(16);
    robot.set_motors_sdk(true);

    std::cout << "Starting Titati motor test on joint " << opts.joint
              << " using mode " << opts.mode << "." << std::endl;

    auto start = std::chrono::steady_clock::now();
    auto next_print = start;

    while (g_running)
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed <= opts.duration)
        {
            if (opts.mode == "torque")
            {
                std::vector<double> torques(16, 0.0);
                torques[opts.joint] = opts.value;
                robot.set_target_joint_t(torques);
            }
            else
            {
                std::vector<double> q(16, 0.0), dq(16, 0.0), kp(16, 0.0), kd(16, 0.0), tau(16, 0.0);
                q[opts.joint] = opts.value;
                kp[opts.joint] = opts.kp;
                kd[opts.joint] = opts.kd;
                robot.set_target_joint_mit(q, dq, kp, kd, tau);
            }
        }
        else
        {
            std::vector<double> zero(16, 0.0);
            robot.set_target_joint_t(zero);
            g_running = 0;
        }

        if (now >= next_print)
        {
            auto positions = robot.get_joint_q();
            auto velocities = robot.get_joint_v();
            auto torques = robot.get_joint_t();
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "q[" << opts.joint << "]=" << positions[opts.joint]
                      << " rad, dq=" << velocities[opts.joint]
                      << " rad/s, tau=" << torques[opts.joint] << " Nm" << std::endl;
            next_print = now + std::chrono::milliseconds(200);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    robot.set_target_joint_t(std::vector<double>(16, 0.0));
    robot.set_motors_sdk(false);
    manager.stop();

    std::cout << "Test finished." << std::endl;
    return 0;
}
