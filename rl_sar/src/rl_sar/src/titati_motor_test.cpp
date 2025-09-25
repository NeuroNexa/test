#include "titati/titati_hardware.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace
{
void PrintJointState(const rl_sar::TitatiJointState &state)
{
    std::cout << "\n[Joint State] index | position(rad) | velocity(rad/s) | torque(Nm)" << std::endl;
    for (std::size_t i = 0; i < state.position.size(); ++i)
    {
        std::cout << std::setw(4) << i << " | "
                  << std::setw(12) << std::setprecision(6) << state.position[i] << " | "
                  << std::setw(13) << std::setprecision(6) << state.velocity[i] << " | "
                  << std::setw(11) << std::setprecision(6) << state.torque[i] << std::endl;
    }
}

void PrintImuState(const rl_sar::TitatiImuState &imu)
{
    std::cout << "\n[IMU] quaternion (w,x,y,z): "
              << imu.quaternion_wxyz[0] << ", "
              << imu.quaternion_wxyz[1] << ", "
              << imu.quaternion_wxyz[2] << ", "
              << imu.quaternion_wxyz[3] << std::endl;
    std::cout << "[IMU] angular velocity (rad/s): "
              << imu.angular_velocity[0] << ", "
              << imu.angular_velocity[1] << ", "
              << imu.angular_velocity[2] << std::endl;
    std::cout << "[IMU] linear acceleration (m/s^2): "
              << imu.linear_acceleration[0] << ", "
              << imu.linear_acceleration[1] << ", "
              << imu.linear_acceleration[2] << std::endl;
}
}

int main(int argc, char **argv)
{
    bool torque_mode = false;
    bool mit_mode = false;
    bool read_only = true;
    int target_motor = -1;
    double torque_value = 0.0;
    double duration = 2.0;
    double target_position = 0.0;
    double target_velocity = 0.0;
    double target_kp = 0.0;
    double target_kd = 0.0;
    double target_tau_ff = 0.0;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc)
        {
            if (std::strcmp(argv[i + 1], "torque") == 0)
            {
                torque_mode = true;
                read_only = false;
            }
            else if (std::strcmp(argv[i + 1], "mit") == 0)
            {
                mit_mode = true;
                read_only = false;
            }
            ++i;
        }
        else if (std::strcmp(argv[i], "--motor") == 0 && i + 1 < argc)
        {
            target_motor = std::atoi(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--value") == 0 && i + 1 < argc)
        {
            torque_value = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc)
        {
            duration = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--position") == 0 && i + 1 < argc)
        {
            target_position = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--velocity") == 0 && i + 1 < argc)
        {
            target_velocity = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc)
        {
            target_kp = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc)
        {
            target_kd = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--tau") == 0 && i + 1 < argc)
        {
            target_tau_ff = std::atof(argv[++i]);
        }
    }

    rl_sar::TitatiHardware hardware(16);
    if (!hardware.EnableDirectControl())
    {
        std::cerr << "[WARN] Failed to enable direct control. Commands may be ignored." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto joint_state = hardware.ReadJointState();
    const auto imu_state = hardware.ReadImuState();
    PrintJointState(joint_state);
    PrintImuState(imu_state);

    if (read_only)
    {
        std::cout << "\nUsage examples:" << std::endl;
        std::cout << "  ./titati_motor_test --mode torque --motor 3 --value 5.0 --duration 1.0" << std::endl;
        std::cout << "  ./titati_motor_test --mode mit --motor 7 --position 0.2 --kp 40 --kd 2 --tau 0.0" << std::endl;
        return 0;
    }

    if (target_motor < 0 || target_motor >= static_cast<int>(hardware.MotorCount()))
    {
        std::cerr << "[ERROR] Invalid motor index. Valid range: 0-" << hardware.MotorCount() - 1 << std::endl;
        return 1;
    }

    if (torque_mode)
    {
        std::vector<double> torque(hardware.MotorCount(), 0.0);
        torque[target_motor] = torque_value;
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration)
        {
            hardware.SendTorqueCommand(torque);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        hardware.SendTorqueCommand(std::vector<double>(hardware.MotorCount(), 0.0));
        std::cout << "Applied torque command to motor " << target_motor << " for " << duration << " s." << std::endl;
    }
    else if (mit_mode)
    {
        std::vector<double> q(hardware.MotorCount(), 0.0);
        std::vector<double> dq(hardware.MotorCount(), 0.0);
        std::vector<double> kp(hardware.MotorCount(), 0.0);
        std::vector<double> kd(hardware.MotorCount(), 0.0);
        std::vector<double> tau(hardware.MotorCount(), 0.0);
        q[target_motor] = target_position;
        dq[target_motor] = target_velocity;
        kp[target_motor] = target_kp;
        kd[target_motor] = target_kd;
        tau[target_motor] = target_tau_ff;
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration)
        {
            hardware.SendMITCommand(q, dq, kp, kd, tau);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        hardware.SendTorqueCommand(std::vector<double>(hardware.MotorCount(), 0.0));
        std::cout << "Applied MIT command to motor " << target_motor << " for " << duration << " s." << std::endl;
    }

    const auto joint_state_after = hardware.ReadJointState();
    PrintJointState(joint_state_after);
    return 0;
}
