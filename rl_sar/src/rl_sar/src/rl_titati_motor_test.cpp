#include "tita_robot/tita_robot.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cctype>
#include <exception>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr const char *LOG_INFO = "\033[0;37m[INFO]\033[0m ";
constexpr const char *LOG_WARNING = "\033[0;33m[WARNING]\033[0m ";
constexpr const char *LOG_ERROR = "\033[0;31m[ERROR]\033[0m ";

std::atomic<bool> g_running{true};

void handle_signal(int)
{
    g_running.store(false);
}

void print_usage(const char *program)
{
    std::cout << "Usage: " << program
              << " [--interface canX] [--config path] [--mode mit|torque]" << std::endl;
    std::cout << "             [--amplitude value] [--hold seconds] [--settle seconds]" << std::endl;
    std::cout << "             [--router|--no-router]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --interface, -i   CAN interface name (default: use config or can0)" << std::endl;
    std::cout << "  --config, -c      Path to Titati base.yaml (default: policy/titati/base.yaml)" << std::endl;
    std::cout << "  --mode, -m        Command mode: 'mit' (default) or 'torque'" << std::endl;
    std::cout << "  --amplitude, -a   Command amplitude (radians for MIT, Nm for torque)" << std::endl;
    std::cout << "  --hold            Time in seconds to hold the command (default: 1.0)" << std::endl;
    std::cout << "  --settle          Time in seconds to settle back to neutral (default: 0.5)" << std::endl;
    std::cout << "  --router          Force enabling the CAN-FD router handshake" << std::endl;
    std::cout << "  --no-router       Disable the CAN-FD router handshake" << std::endl;
    std::cout << "  --help, -h        Show this message" << std::endl;
}

enum class TestMode
{
    MIT,
    Torque
};

struct ProgramOptions
{
    std::string interface = "";
    std::string config_path = "policy/titati/base.yaml";
    std::string mode = "mit";
    double amplitude = 0.05;
    double hold_seconds = 1.0;
    double settle_seconds = 0.5;
    int router_override = -1; // -1 = use config, 0 = disabled, 1 = enabled
    bool interface_overridden = false;
};

bool parse_options(int argc, char **argv, ProgramOptions &options, bool &show_usage)
{
    for (int idx = 1; idx < argc; ++idx)
    {
        const std::string arg = argv[idx];
        if ((arg == "--interface" || arg == "-i") && idx + 1 < argc)
        {
            options.interface = argv[++idx];
            options.interface_overridden = true;
        }
        else if ((arg == "--config" || arg == "-c") && idx + 1 < argc)
        {
            options.config_path = argv[++idx];
        }
        else if ((arg == "--mode" || arg == "-m") && idx + 1 < argc)
        {
            options.mode = argv[++idx];
        }
        else if ((arg == "--amplitude" || arg == "-a") && idx + 1 < argc)
        {
            try
            {
                options.amplitude = std::stod(argv[++idx]);
            }
            catch (const std::exception &ex)
            {
                std::cerr << LOG_ERROR << "Invalid amplitude value: " << ex.what() << std::endl;
                return false;
            }
        }
        else if (arg == "--hold" && idx + 1 < argc)
        {
            try
            {
                options.hold_seconds = std::stod(argv[++idx]);
            }
            catch (const std::exception &ex)
            {
                std::cerr << LOG_ERROR << "Invalid hold duration: " << ex.what() << std::endl;
                return false;
            }
        }
        else if (arg == "--settle" && idx + 1 < argc)
        {
            try
            {
                options.settle_seconds = std::stod(argv[++idx]);
            }
            catch (const std::exception &ex)
            {
                std::cerr << LOG_ERROR << "Invalid settle duration: " << ex.what() << std::endl;
                return false;
            }
        }
        else if (arg == "--router")
        {
            options.router_override = 1;
        }
        else if (arg == "--no-router")
        {
            options.router_override = 0;
        }
        else if (arg == "--help" || arg == "-h")
        {
            show_usage = true;
            return true;
        }
        else
        {
            std::cerr << LOG_ERROR << "Unknown argument '" << arg << "'." << std::endl;
            return false;
        }
    }

    return true;
}

template <typename T>
std::vector<T> reorder_to_hardware(const std::vector<T> &rl_values,
                                   const std::vector<int> &mapping,
                                   std::size_t motor_count,
                                   const T &default_value)
{
    std::vector<T> result(motor_count, default_value);
    for (std::size_t rl_index = 0; rl_index < rl_values.size() && rl_index < mapping.size(); ++rl_index)
    {
        const int hw_index = mapping[rl_index];
        if (hw_index >= 0 && hw_index < static_cast<int>(motor_count))
        {
            result[static_cast<std::size_t>(hw_index)] = rl_values[rl_index];
        }
    }
    return result;
}

std::vector<std::string> reorder_names_to_hardware(const std::vector<std::string> &rl_names,
                                                   const std::vector<int> &mapping,
                                                   std::size_t motor_count)
{
    std::vector<std::string> result(motor_count);
    for (std::size_t idx = 0; idx < motor_count; ++idx)
    {
        result[idx] = "motor_" + std::to_string(idx);
    }
    for (std::size_t rl_index = 0; rl_index < rl_names.size() && rl_index < mapping.size(); ++rl_index)
    {
        const int hw_index = mapping[rl_index];
        if (hw_index >= 0 && hw_index < static_cast<int>(motor_count))
        {
            result[static_cast<std::size_t>(hw_index)] = rl_names[rl_index];
        }
    }
    return result;
}

struct MotorTestConfig
{
    int num_dofs = 0;
    std::vector<int> joint_mapping;
    std::vector<std::string> joint_names_hw;
    std::vector<double> fixed_kp_hw;
    std::vector<double> fixed_kd_hw;
    std::vector<double> torque_limits_hw;
    bool use_canfd_router = true;
    std::string can_interface = "can0";
};

MotorTestConfig load_motor_config(const std::string &path)
{
    YAML::Node root = YAML::LoadFile(path);
    if (!root["titati"])
    {
        throw std::runtime_error("Missing 'titati' namespace in config: " + path);
    }

    YAML::Node node = root["titati"];

    MotorTestConfig config;
    config.num_dofs = node["num_of_dofs"].as<int>();
    config.joint_mapping = node["joint_mapping"].as<std::vector<int>>();
    const auto joint_names = node["joint_names"].as<std::vector<std::string>>();
    const auto fixed_kp = node["fixed_kp"].as<std::vector<double>>();
    const auto fixed_kd = node["fixed_kd"].as<std::vector<double>>();
    const auto torque_limits = node["torque_limits"].as<std::vector<double>>();

    config.joint_names_hw = reorder_names_to_hardware(joint_names, config.joint_mapping, config.num_dofs);
    config.fixed_kp_hw = reorder_to_hardware(fixed_kp, config.joint_mapping, config.num_dofs, 0.0);
    config.fixed_kd_hw = reorder_to_hardware(fixed_kd, config.joint_mapping, config.num_dofs, 0.0);
    config.torque_limits_hw = reorder_to_hardware(torque_limits, config.joint_mapping, config.num_dofs, 0.0);
    if (node["use_canfd_router"])
    {
        config.use_canfd_router = node["use_canfd_router"].as<bool>();
    }
    else
    {
        config.use_canfd_router = config.num_dofs > 8;
    }

    if (node["can_interface"])
    {
        config.can_interface = node["can_interface"].as<std::string>();
    }

    return config;
}

bool enable_sdk_with_retry(tita_robot &robot,
                           int motor_count,
                           const std::vector<double> &kp,
                           const std::vector<double> &kd)
{
    std::vector<double> dq(motor_count, 0.0);
    std::vector<double> tau(motor_count, 0.0);

    constexpr int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        if (robot.set_motors_sdk(true))
        {
            auto q = robot.get_joint_q();
            if (static_cast<int>(q.size()) != motor_count)
            {
                q.resize(static_cast<std::size_t>(motor_count), 0.0);
            }

            if (!robot.set_target_joint_mit(q, dq, kp, kd, tau))
            {
                std::cerr << LOG_WARNING << "Failed to send hold command after enabling SDK control." << std::endl;
            }
            else
            {
                std::cout << LOG_INFO << "Applied hold command after enabling SDK control." << std::endl;
            }

            std::cout << LOG_INFO << "Motors switched to SDK control mode." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }

        std::cerr << LOG_WARNING << "Failed to switch motors to SDK control (attempt "
                  << attempt << "/" << max_attempts << ")." << std::endl;
        robot.set_motors_sdk(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << LOG_ERROR << "Unable to switch motors to SDK control after "
              << max_attempts << " attempts." << std::endl;
    return false;
}

struct MotorScope
{
    tita_robot *robot = nullptr;
    int motor_count = 0;
    const std::vector<double> *kp = nullptr;
    const std::vector<double> *kd = nullptr;

    ~MotorScope()
    {
        if (!robot || motor_count <= 0)
        {
            return;
        }

        std::vector<double> dq(static_cast<std::size_t>(motor_count), 0.0);
        std::vector<double> tau(static_cast<std::size_t>(motor_count), 0.0);

        try
        {
            auto q = robot->get_joint_q();
            if (static_cast<int>(q.size()) != motor_count)
            {
                q.resize(static_cast<std::size_t>(motor_count), 0.0);
            }

            if (kp && kd && kp->size() == dq.size() && kd->size() == dq.size())
            {
                robot->set_target_joint_mit(q, dq, *kp, *kd, tau);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            robot->set_target_joint_t(tau);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        catch (...)
        {
            // Ignore cleanup errors.
        }

        robot->set_motors_sdk(false);
    }
};

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    ProgramOptions options;
    bool show_usage = false;
    if (!parse_options(argc, argv, options, show_usage))
    {
        if (show_usage)
        {
            print_usage(argv[0]);
            return 0;
        }
        return 1;
    }

    if (show_usage)
    {
        print_usage(argv[0]);
        return 0;
    }

    std::string mode_lower = options.mode;
    std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    TestMode mode = TestMode::MIT;
    if (mode_lower == "mit")
    {
        mode = TestMode::MIT;
    }
    else if (mode_lower == "torque")
    {
        mode = TestMode::Torque;
        if (std::abs(options.amplitude) < 1e-6)
        {
            options.amplitude = 2.0; // provide a sensible default torque pulse
        }
    }
    else
    {
        std::cerr << LOG_ERROR << "Unsupported mode '" << options.mode << "'." << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    if (options.hold_seconds <= 0.0)
    {
        std::cerr << LOG_WARNING << "Hold duration must be positive. Using 1.0 second." << std::endl;
        options.hold_seconds = 1.0;
    }
    if (options.settle_seconds <= 0.0)
    {
        std::cerr << LOG_WARNING << "Settle duration must be positive. Using 0.5 second." << std::endl;
        options.settle_seconds = 0.5;
    }

    MotorTestConfig config;
    try
    {
        config = load_motor_config(options.config_path);
    }
    catch (const std::exception &ex)
    {
        std::cerr << LOG_ERROR << "Failed to load Titati config: " << ex.what() << std::endl;
        return 1;
    }

    if (!options.interface_overridden)
    {
        options.interface = config.can_interface;
    }

    const bool use_router = (options.router_override == -1)
                                ? config.use_canfd_router
                                : (options.router_override == 1);

    std::cout << LOG_INFO << "Opening interface '" << options.interface << "'"
              << " (router " << (use_router ? "enabled" : "disabled") << ")" << std::endl;

    tita_robot robot(static_cast<std::size_t>(config.num_dofs), options.interface, use_router);
    MotorScope guard{&robot, config.num_dofs, &config.fixed_kp_hw, &config.fixed_kd_hw};

    if (!enable_sdk_with_retry(robot, config.num_dofs, config.fixed_kp_hw, config.fixed_kd_hw))
    {
        std::cerr << LOG_ERROR << "Cannot continue without SDK control." << std::endl;
        return 1;
    }

    if (std::abs(options.amplitude) < 1e-6)
    {
        std::cout << LOG_WARNING << "Amplitude is near zero; commands will keep the robot stationary." << std::endl;
    }

    std::cout << LOG_INFO << "Testing " << config.num_dofs << " motors in "
              << (mode == TestMode::MIT ? "MIT" : "torque") << " mode with amplitude "
              << options.amplitude << (mode == TestMode::MIT ? " rad" : " Nm") << "." << std::endl;
    std::cout << LOG_INFO << "Hold duration: " << options.hold_seconds
              << " s, settle duration: " << options.settle_seconds << " s." << std::endl;
    std::cout << LOG_INFO << "Press Ctrl+C to abort early." << std::endl;

    const auto hold_duration = std::chrono::duration<double>(options.hold_seconds);
    const auto settle_duration = std::chrono::duration<double>(options.settle_seconds);
    const auto command_period = std::chrono::milliseconds(5);
    const auto log_interval = std::chrono::milliseconds(200);

    std::cout << std::fixed << std::setprecision(3);

    for (int hw_index = 0; hw_index < config.num_dofs && g_running.load(); ++hw_index)
    {
        const std::string &name = config.joint_names_hw[static_cast<std::size_t>(hw_index)];
        std::cout << std::endl
                  << LOG_INFO << "Testing motor " << hw_index << " (" << name << ")" << std::endl;

        auto baseline_q = robot.get_joint_q();
        if (static_cast<int>(baseline_q.size()) != config.num_dofs)
        {
            baseline_q.resize(static_cast<std::size_t>(config.num_dofs), 0.0);
        }

        std::vector<double> q_cmd = baseline_q;
        std::vector<double> dq_cmd(static_cast<std::size_t>(config.num_dofs), 0.0);
        std::vector<double> tau_cmd(static_cast<std::size_t>(config.num_dofs), 0.0);

        double torque_command = options.amplitude;
        if (mode == TestMode::Torque && hw_index < static_cast<int>(config.torque_limits_hw.size()))
        {
            const double limit = config.torque_limits_hw[static_cast<std::size_t>(hw_index)];
            if (limit > 0.0 && std::abs(torque_command) > limit)
            {
                std::cout << LOG_WARNING << "Requested torque " << torque_command
                          << " Nm exceeds limit " << limit << " Nm; clipping." << std::endl;
                torque_command = std::max(-limit, std::min(limit, torque_command));
            }
        }

        auto next_log = std::chrono::steady_clock::now();
        const auto command_end = std::chrono::steady_clock::now() + hold_duration;
        bool command_error_reported = false;

        while (g_running.load() && std::chrono::steady_clock::now() < command_end)
        {
            bool sent = true;
            if (mode == TestMode::MIT)
            {
                q_cmd = baseline_q;
                q_cmd[static_cast<std::size_t>(hw_index)] = baseline_q[static_cast<std::size_t>(hw_index)] + options.amplitude;
                sent = robot.set_target_joint_mit(q_cmd, dq_cmd, config.fixed_kp_hw, config.fixed_kd_hw, tau_cmd);
            }
            else
            {
                tau_cmd.assign(tau_cmd.size(), 0.0);
                tau_cmd[static_cast<std::size_t>(hw_index)] = torque_command;
                sent = robot.set_target_joint_t(tau_cmd);
            }

            if (!sent && !command_error_reported)
            {
                std::cout << LOG_WARNING << "Failed to transmit command frame." << std::endl;
                command_error_reported = true;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= next_log)
            {
                auto q_feedback = robot.get_joint_q();
                auto dq_feedback = robot.get_joint_v();
                auto tau_feedback = robot.get_joint_t();
                if (static_cast<int>(q_feedback.size()) != config.num_dofs)
                {
                    q_feedback.resize(static_cast<std::size_t>(config.num_dofs), 0.0);
                }
                if (static_cast<int>(dq_feedback.size()) != config.num_dofs)
                {
                    dq_feedback.resize(static_cast<std::size_t>(config.num_dofs), 0.0);
                }
                if (static_cast<int>(tau_feedback.size()) != config.num_dofs)
                {
                    tau_feedback.resize(static_cast<std::size_t>(config.num_dofs), 0.0);
                }

                std::cout << LOG_INFO << "  feedback q=" << q_feedback[static_cast<std::size_t>(hw_index)]
                          << " rad, dq=" << dq_feedback[static_cast<std::size_t>(hw_index)]
                          << " rad/s, tau=" << tau_feedback[static_cast<std::size_t>(hw_index)] << " Nm" << std::endl;
                next_log = now + log_interval;
            }

            std::this_thread::sleep_for(command_period);
        }

        if (!g_running.load())
        {
            break;
        }

        baseline_q = robot.get_joint_q();
        if (static_cast<int>(baseline_q.size()) != config.num_dofs)
        {
            baseline_q.resize(static_cast<std::size_t>(config.num_dofs), 0.0);
        }

        const auto settle_end = std::chrono::steady_clock::now() + settle_duration;
        while (g_running.load() && std::chrono::steady_clock::now() < settle_end)
        {
            bool sent = true;
            if (mode == TestMode::MIT)
            {
                q_cmd = baseline_q;
                sent = robot.set_target_joint_mit(q_cmd, dq_cmd, config.fixed_kp_hw, config.fixed_kd_hw, tau_cmd);
            }
            else
            {
                tau_cmd.assign(tau_cmd.size(), 0.0);
                sent = robot.set_target_joint_t(tau_cmd);
            }

            if (!sent && !command_error_reported)
            {
                std::cout << LOG_WARNING << "Failed to transmit settle command." << std::endl;
                command_error_reported = true;
            }

            std::this_thread::sleep_for(command_period);
        }
    }

    std::cout << std::endl << LOG_INFO << "Motor test finished." << std::endl;
    return 0;
}
