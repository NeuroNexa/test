#include "titati_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
constexpr double kTwoPi = 6.28318530717958647692;
std::atomic<bool> g_keep_running{true};

void SignalHandler(int)
{
    g_keep_running.store(false);
}

struct Options
{
    std::string can_interface{"can0"};
    std::size_t motor_count{16};
    std::chrono::seconds duration{10};
    std::chrono::milliseconds command_period{10};
    std::optional<std::size_t> jog_motor{};
    double jog_torque{0.4};
    std::chrono::milliseconds jog_period{2000};
};

Options ParseOptions(int argc, char** argv)
{
    Options opts;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [--can-interface NAME] [--duration SEC]"
                      << " [--command-period MS] [--motor-count N] [--jog-motor INDEX]"
                      << " [--jog-torque NM] [--jog-period MS]" << std::endl;
            std::exit(EXIT_SUCCESS);
        }
        else if (arg == "--can-interface" && i + 1 < argc)
        {
            opts.can_interface = argv[++i];
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            opts.duration = std::chrono::seconds(std::stoll(argv[++i]));
        }
        else if (arg == "--command-period" && i + 1 < argc)
        {
            opts.command_period = std::chrono::milliseconds(std::stoll(argv[++i]));
        }
        else if (arg == "--motor-count" && i + 1 < argc)
        {
            opts.motor_count = static_cast<std::size_t>(std::stoul(argv[++i]));
        }
        else if (arg == "--jog-motor" && i + 1 < argc)
        {
            opts.jog_motor = static_cast<std::size_t>(std::stoul(argv[++i]));
        }
        else if (arg == "--jog-torque" && i + 1 < argc)
        {
            opts.jog_torque = std::stod(argv[++i]);
        }
        else if (arg == "--jog-period" && i + 1 < argc)
        {
            opts.jog_period = std::chrono::milliseconds(std::stoll(argv[++i]));
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
    if (opts.motor_count == 0)
    {
        std::cerr << "motor-count must be greater than zero" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (opts.duration.count() <= 0)
    {
        std::cerr << "duration must be positive" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (opts.command_period.count() <= 0)
    {
        std::cerr << "command-period must be positive" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (opts.jog_period.count() <= 0)
    {
        std::cerr << "jog-period must be positive" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (opts.jog_motor.has_value() && *opts.jog_motor >= opts.motor_count)
    {
        std::cerr << "jog-motor index must be in [0, " << opts.motor_count - 1 << ']'
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (opts.jog_torque < 0.0)
    {
        std::cerr << "jog-torque must be non-negative" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return opts;
}

void PrintStatus(const char* prefix,
                 const Clock::time_point& start,
                 std::size_t ready_count,
                 std::size_t expected,
                 std::uint64_t sequence,
                 std::uint32_t imu_ts)
{
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
    std::cout << '[' << std::setw(4) << std::setfill('0') << elapsed.count() / 1000 << '.'
              << std::setw(1) << (elapsed.count() / 100) % 10 << "s] " << prefix
              << " (" << ready_count << '/' << expected << " ready";
    if (sequence > 0)
    {
        std::cout << ", seq=" << sequence;
    }
    if (imu_ts > 0)
    {
        std::cout << ", imu_ts=" << imu_ts;
    }
    std::cout << ')' << std::setfill(' ') << std::endl;
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const Options options = ParseOptions(argc, argv);

    titati::hardware::TitatiHardware hardware(options.can_interface, options.motor_count);
    if (!hardware.Initialize())
    {
        std::cerr << "[TitatiSelfTest] Failed to initialise CAN interface on " << options.can_interface << std::endl;
        return EXIT_FAILURE;
    }

    if (!hardware.SetDirectControlMode(true))
    {
        std::cerr << "[TitatiSelfTest] Warning: unable to confirm FORCE_DIRECT handshake" << std::endl;
    }

    std::vector<double> zero(options.motor_count, 0.0);
    std::vector<double> torque_command(options.motor_count, 0.0);
    const double jog_period_seconds = static_cast<double>(options.jog_period.count()) / 1000.0;
    auto next_command_time = Clock::now();
    const auto deadline = Clock::now() + options.duration;
    const auto start_time = Clock::now();

    std::size_t ready_motors = 0;
    std::size_t peak_ready_motors = 0;
    std::size_t ready_frames = 0;
    std::size_t total_frames = 0;
    std::uint64_t last_sequence = 0;
    std::uint64_t longest_gap_us = 0;
    std::uint64_t last_timestamp_us = 0;

    bool sent_command_failure = false;
    bool jog_announced = false;

    while (g_keep_running.load() && Clock::now() < deadline)
    {
        const auto state = hardware.GetLatestState();

        ready_motors = 0;
        for (const auto& motor : state.motors)
        {
            if (motor.timestamp != 0)
            {
                ++ready_motors;
            }
        }
        peak_ready_motors = std::max(peak_ready_motors, ready_motors);

        if (state.sequence != 0 && state.sequence != last_sequence)
        {
            ++total_frames;
            if (ready_motors == options.motor_count)
            {
                ++ready_frames;
            }
            if (last_timestamp_us != 0)
            {
                const auto gap = (state.imu.timestamp > last_timestamp_us)
                                      ? static_cast<std::uint64_t>(state.imu.timestamp) - last_timestamp_us
                                      : 0U;
                longest_gap_us = std::max(longest_gap_us, gap);
            }
            last_sequence = state.sequence;
            last_timestamp_us = state.imu.timestamp;
        }

        if (ready_frames == 0 && (total_frames % 10 == 0))
        {
            PrintStatus("waiting for motor feedback", start_time, peak_ready_motors, options.motor_count,
                        state.sequence, state.imu.timestamp);
        }
        else if (total_frames % 100 == 0 && total_frames > 0)
        {
            PrintStatus("streaming feedback", start_time, ready_motors, options.motor_count, state.sequence,
                        state.imu.timestamp);
        }

        const auto now = Clock::now();
        if (!jog_announced && options.jog_motor.has_value() && ready_frames > 0)
        {
            std::cout << "[TitatiSelfTest] Jogging motor " << *options.jog_motor
                      << " with ±" << options.jog_torque << " Nm sine wave." << std::endl;
            jog_announced = true;
        }

        if (now >= next_command_time)
        {
            std::fill(torque_command.begin(), torque_command.end(), 0.0);
            if (options.jog_motor.has_value() && ready_frames > 0 && options.jog_torque > 0.0)
            {
                const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);
                const double phase = std::fmod(elapsed.count(), jog_period_seconds);
                const double omega = kTwoPi / jog_period_seconds;
                torque_command[*options.jog_motor] = options.jog_torque * std::sin(omega * phase);
            }

            if (!hardware.SendMitCommand(zero, zero, zero, zero, torque_command))
            {
                sent_command_failure = true;
                std::cerr << "[TitatiSelfTest] Failed to broadcast zero MIT command" << std::endl;
                break;
            }
            next_command_time = now + options.command_period;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    hardware.SetDirectControlMode(false);
    hardware.Shutdown();

    if (sent_command_failure)
    {
        return 3;
    }

    if (total_frames == 0)
    {
        std::cerr << "[TitatiSelfTest] FAIL: no feedback frames received. Check CAN wiring and router power." << std::endl;
        return 1;
    }

    if (peak_ready_motors < options.motor_count)
    {
        std::cerr << "[TitatiSelfTest] FAIL: only " << peak_ready_motors << " motors reported timestamps." << std::endl;
        return 2;
    }

    std::cout << "[TitatiSelfTest] PASS: received " << total_frames << " frames with all "
              << options.motor_count << " motors updating." << std::endl;
    if (longest_gap_us > 0)
    {
        std::cout << "[TitatiSelfTest] Longest IMU gap " << longest_gap_us << " us" << std::endl;
    }
    std::cout << "[TitatiSelfTest] Zero-torque commands broadcast every " << options.command_period.count()
              << " ms" << std::endl;
    return EXIT_SUCCESS;
}
