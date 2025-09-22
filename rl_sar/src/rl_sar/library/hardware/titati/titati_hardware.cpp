#include "titati_hardware.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <cerrno>

namespace titati::hardware
{
namespace
{
constexpr std::uint32_t kMotorFeedbackBaseId = 0x108U;
constexpr std::uint32_t kMotorFeedbackMask = 0x7E0U;
constexpr std::uint32_t kMotorCommandBaseId = 0x120U;
constexpr std::uint32_t kImuFeedbackId = 0x118U;
constexpr std::uint32_t kRouterStatusId = 0x09FU;
constexpr std::uint32_t kRpcCommandId = 0x170U;
constexpr float kGravity = 9.81f;
constexpr std::uint16_t kRpcKeyReadyNext = 0x200U;
constexpr std::uint32_t kReadyWaiting = 0x00U;
constexpr std::uint32_t kAutoLocomotion = 0x01U;
constexpr std::uint32_t kForceDirect = 0x03U;

struct __attribute__((packed)) MotorFeedbackPacket
{
    std::uint32_t timestamp;
    float position;
    float velocity;
    float torque;
};

struct __attribute__((packed)) ImuPacket
{
    std::uint32_t timestamp;
    float acceleration[3];
    float gyroscope[3];
    float quaternion[4];  // x, y, z, w
    float temperature;
};

}  // namespace

TitatiHardware::TitatiHardware(std::string can_interface, std::size_t motor_count)
    : interface_(std::move(can_interface)),
      motor_count_(motor_count)
{
    if (motor_count_ == 0)
    {
        throw std::invalid_argument("motor_count must be greater than zero");
    }

    if (motor_count_ % 8 == 0)
    {
        dof_per_leg_ = 4;
    }
    else if (motor_count_ % 6 == 0)
    {
        dof_per_leg_ = 3;
    }
    else if (motor_count_ % 4 == 0)
    {
        dof_per_leg_ = 4;
    }
    else
    {
        dof_per_leg_ = motor_count_;
    }

    if (dof_per_leg_ == 0)
    {
        dof_per_leg_ = motor_count_;
    }

    if (dof_per_leg_ > 0)
    {
        leg_count_ = motor_count_ / dof_per_leg_;
        if (motor_count_ % dof_per_leg_ != 0)
        {
            ++leg_count_;
        }
    }

    if (leg_count_ == 0)
    {
        leg_count_ = 1;
        dof_per_leg_ = motor_count_;
    }
    motor_state_.resize(motor_count_);
}

TitatiHardware::~TitatiHardware()
{
    Shutdown();
}

bool TitatiHardware::Initialize()
{
    if (initialized_.load())
    {
        return true;
    }

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        std::perror("titati_can socket");
        return false;
    }

    int canfd_on = 1;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) < 0)
    {
        std::perror("setsockopt CAN_RAW_FD_FRAMES");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    int recv_own_msgs = 0;
    (void)::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    struct ifreq ifr
    {
    };
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        std::perror("ioctl SIOCGIFINDEX");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    const int ifindex = ifr.ifr_ifindex;

    if (ioctl(socket_fd_, SIOCGIFMTU, &ifr) < 0)
    {
        std::perror("ioctl SIOCGIFMTU");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    if (ifr.ifr_mtu != CANFD_MTU)
    {
        std::cerr << "[TitatiHardware] CAN interface " << interface_
                  << " is not configured for CAN-FD (expected MTU " << CANFD_MTU
                  << ", got " << ifr.ifr_mtu << ')' << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr
    {
    };
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifindex;
    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        std::perror("bind can interface");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct can_filter filters[3];
    filters[0].can_id = kMotorFeedbackBaseId;
    filters[0].can_mask = kMotorFeedbackMask;
    filters[1].can_id = kImuFeedbackId;
    filters[1].can_mask = CAN_SFF_MASK;
    filters[2].can_id = kRouterStatusId;
    filters[2].can_mask = CAN_SFF_MASK;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0)
    {
        std::perror("setsockopt CAN_RAW_FILTER");
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_.store(true);
    receiver_thread_ = std::thread(&TitatiHardware::ReceiverLoop, this);
    initialized_.store(true);
    return true;
}

void TitatiHardware::Shutdown()
{
    if (initialized_.load())
    {
        SetDirectControlMode(false);
    }

    running_.store(false);
    if (receiver_thread_.joinable())
    {
        receiver_thread_.join();
    }

    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

    initialized_.store(false);
}

CombinedState TitatiHardware::GetLatestState() const
{
    CombinedState output;
    output.motors.resize(motor_state_.size());

    {
        std::scoped_lock<std::mutex> lock(state_mutex_);
        output.motors = motor_state_;
        output.imu = imu_state_;
    }
    output.sequence = state_sequence_.load(std::memory_order_relaxed);
    return output;
}

bool TitatiHardware::SendMitCommand(const std::vector<double>& position,
                                    const std::vector<double>& velocity,
                                    const std::vector<double>& kp,
                                    const std::vector<double>& kd,
                                    const std::vector<double>& torque)
{
    if (!initialized_.load())
    {
        return false;
    }

    if (position.size() != motor_count_ || velocity.size() != motor_count_ ||
        kp.size() != motor_count_ || kd.size() != motor_count_ || torque.size() != motor_count_)
    {
        std::cerr << "[TitatiHardware] Invalid command vector size" << std::endl;
        return false;
    }

    if (motor_count_ == 0)
    {
        return false;
    }

    const std::vector<TitatiHardware::MotorCommandPacket> command_packets =
        BuildMotorCommandPackets(position, velocity, kp, kd, torque);
    if (command_packets.empty())
    {
        return false;
    }

    if ((command_packets.size() % 2U) != 0U)
    {
        return false;
    }

    bool success = true;
    const std::size_t command_pairs = command_packets.size() / 2U;

    for (std::size_t pair = 0; pair < command_pairs; ++pair)
    {
        struct canfd_frame frame{};
        frame.can_id = kMotorCommandBaseId + pair;
        frame.len = sizeof(TitatiHardware::MotorCommandPacket) * 2U;
        frame.flags = CANFD_BRS | CANFD_FDF;
        std::memcpy(frame.data, &command_packets[pair * 2U], sizeof(TitatiHardware::MotorCommandPacket));
        std::memcpy(frame.data + sizeof(TitatiHardware::MotorCommandPacket),
                    &command_packets[pair * 2U + 1U], sizeof(TitatiHardware::MotorCommandPacket));

        if (!SendFrame(&frame, CANFD_MTU))
        {
            success = false;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(150));
    }

    return success;
}

bool TitatiHardware::SendTorqueCommand(const std::vector<double>& torque)
{
    std::vector<double> zeros(motor_count_, 0.0);
    return SendMitCommand(zeros, zeros, zeros, zeros, torque);
}

bool TitatiHardware::SetDirectControlMode(bool enable)
{
    if (!initialized_.load())
    {
        return false;
    }

    direct_mode_requested_.store(enable);
    if (!enable)
    {
        last_force_direct_request_us_.store(0U, std::memory_order_relaxed);
    }

    bool ok = SendRpcCommand(kRpcKeyReadyNext, kReadyWaiting);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    ok &= SendRpcCommand(kRpcKeyReadyNext, enable ? kForceDirect : kAutoLocomotion);

    if (enable)
    {
        last_force_direct_request_us_.store(AcquireSteadyTimestampUs(), std::memory_order_relaxed);
    }
    return ok;
}

void TitatiHardware::ReceiverLoop()
{
    struct pollfd poll_fd
    {
        socket_fd_, POLLIN, 0
    };

    while (running_.load())
    {
        const int poll_ret = ::poll(&poll_fd, 1, 100);
        if (!running_.load())
        {
            break;
        }

        if (poll_ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            std::perror("poll can socket");
            continue;
        }

        if (poll_ret == 0)
        {
            continue;
        }

        struct canfd_frame frame
        {
        };
        const ssize_t bytes_read = ::read(socket_fd_, &frame, sizeof(frame));
        if (bytes_read < 0)
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                continue;
            }
            std::perror("read can frame");
            continue;
        }

        const std::uint32_t can_id = frame.can_id & CAN_SFF_MASK;
        if (can_id >= kMotorFeedbackBaseId && can_id < kMotorFeedbackBaseId + leg_count_)
        {
            HandleMotorFeedback(can_id, frame.data, frame.len);
        }
        else if (can_id == kImuFeedbackId)
        {
            HandleImuFeedback(frame.data, frame.len);
        }
        else if (can_id == kRouterStatusId)
        {
            HandleRouterFeedback(frame.data, frame.len);
        }
        else
        {
            // Ignore other frames
        }
    }
}

void TitatiHardware::HandleMotorFeedback(std::uint32_t can_id, const std::uint8_t* data, std::uint8_t dlc)
{
    const std::size_t leg_index = can_id - kMotorFeedbackBaseId;
    const std::size_t base_index = leg_index * dof_per_leg_;

    if (leg_index >= leg_count_)
    {
        return;
    }

    std::scoped_lock<std::mutex> lock(state_mutex_);
    const std::size_t motors_in_frame =
        std::min<std::size_t>(dof_per_leg_, dlc / sizeof(MotorFeedbackPacket));
    for (std::size_t i = 0; i < motors_in_frame; ++i)
    {
        MotorFeedbackPacket packet{};
        std::memcpy(&packet, data + i * sizeof(MotorFeedbackPacket), sizeof(MotorFeedbackPacket));

        const std::size_t motor_index = base_index + i;
        if (motor_index >= motor_state_.size())
        {
            continue;
        }

        motor_state_[motor_index].position = static_cast<double>(packet.position);
        motor_state_[motor_index].velocity = static_cast<double>(packet.velocity);
        motor_state_[motor_index].torque = static_cast<double>(packet.torque);
        motor_state_[motor_index].timestamp = packet.timestamp;
    }
    state_sequence_.fetch_add(1, std::memory_order_relaxed);
}

void TitatiHardware::HandleImuFeedback(const std::uint8_t* data, std::uint8_t dlc)
{
    if (dlc < sizeof(ImuPacket))
    {
        return;
    }

    ImuPacket packet{};
    std::memcpy(&packet, data, sizeof(ImuPacket));

    std::scoped_lock<std::mutex> lock(state_mutex_);
    imu_state_.timestamp = packet.timestamp;
    imu_state_.acceleration = {packet.acceleration[0] * kGravity,
                               packet.acceleration[1] * kGravity,
                               packet.acceleration[2] * kGravity};
    imu_state_.gyroscope = {packet.gyroscope[0], packet.gyroscope[1], packet.gyroscope[2]};
    imu_state_.quaternion = {packet.quaternion[3], packet.quaternion[0], packet.quaternion[1], packet.quaternion[2]};
    state_sequence_.fetch_add(1, std::memory_order_relaxed);
}

void TitatiHardware::HandleRouterFeedback(const std::uint8_t* data, std::uint8_t dlc)
{
    if (dlc < 12)
    {
        return;
    }

    std::uint32_t mode = 0;
    std::memcpy(&mode, data + 4, sizeof(mode));

    if (!direct_mode_requested_.load(std::memory_order_relaxed))
    {
        return;
    }

    if (mode == kForceDirect)
    {
        return;
    }

    const auto now = AcquireSteadyTimestampUs();
    const auto last = last_force_direct_request_us_.load(std::memory_order_relaxed);
    if (last != 0U && now - last < 100000U)
    {
        return;
    }

    last_force_direct_request_us_.store(now, std::memory_order_relaxed);
    SendRpcCommand(kRpcKeyReadyNext, kReadyWaiting);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    SendRpcCommand(kRpcKeyReadyNext, kForceDirect);
    last_force_direct_request_us_.store(AcquireSteadyTimestampUs(), std::memory_order_relaxed);
}

bool TitatiHardware::SendRpcCommand(std::uint16_t key, std::uint32_t value)
{
    if (socket_fd_ < 0)
    {
        return false;
    }

    struct canfd_frame frame{};
    frame.can_id = kRpcCommandId;
    frame.len = 10U;
    frame.flags = CANFD_BRS | CANFD_FDF;
    std::memset(frame.data, 0, sizeof(frame.data));

    const std::uint32_t timestamp = AcquireTimestampUs();
    std::memcpy(frame.data, &timestamp, sizeof(timestamp));
    std::memcpy(frame.data + 4, &key, sizeof(key));
    std::memcpy(frame.data + 6, &value, sizeof(value));

    return SendFrame(&frame, CANFD_MTU);
}

std::vector<TitatiHardware::MotorCommandPacket> TitatiHardware::BuildMotorCommandPackets(
    const std::vector<double>& position,
    const std::vector<double>& velocity,
    const std::vector<double>& kp,
    const std::vector<double>& kd,
    const std::vector<double>& torque) const
{
    std::vector<TitatiHardware::MotorCommandPacket> packets;
    packets.reserve(motor_count_);

    const std::uint32_t timestamp = AcquireTimestampUs();

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        TitatiHardware::MotorCommandPacket packet{};
        packet.timestamp = timestamp;
        packet.position = static_cast<float>(position[i]);
        packet.velocity = static_cast<float>(velocity[i]);
        packet.kp = static_cast<float>(kp[i]);
        packet.kd = static_cast<float>(kd[i]);
        packet.torque = static_cast<float>(torque[i]);
        packets.emplace_back(packet);
    }

    if (dof_per_leg_ == 3 && leg_count_ > 0)
    {
        std::vector<TitatiHardware::MotorCommandPacket> padded;
        padded.reserve(leg_count_ * 4U);

        for (std::size_t leg = 0; leg < leg_count_; ++leg)
        {
            const std::size_t base = leg * dof_per_leg_;
            for (std::size_t joint = 0; joint < dof_per_leg_; ++joint)
            {
                const std::size_t idx = base + joint;
                if (idx < packets.size())
                {
                    padded.emplace_back(packets[idx]);
                }
            }

            // Titati controllers expect a 4-slot leg group on the CAN bus even for 3-DOF legs.
            padded.emplace_back();
        }

        if (padded.size() % 2U != 0U)
        {
            padded.emplace_back();
        }

        return padded;
    }

    if (packets.size() % 2U != 0U)
    {
        packets.emplace_back();
    }

    return packets;
}

bool TitatiHardware::SendFrame(const void* frame, std::size_t length) const
{
    if (socket_fd_ < 0)
    {
        return false;
    }

    const ssize_t written = ::write(socket_fd_, frame, length);
    if (written < 0)
    {
        std::perror("write can frame");
        return false;
    }
    return true;
}

std::uint32_t TitatiHardware::AcquireTimestampUs() const
{
    struct timeval tv
    {
    };
    ::gettimeofday(&tv, nullptr);
    const std::uint64_t micros = static_cast<std::uint64_t>(tv.tv_sec) * 1000000ULL +
                                 static_cast<std::uint64_t>(tv.tv_usec);
    return static_cast<std::uint32_t>(micros & 0xFFFFFFFFU);
}

std::uint64_t TitatiHardware::AcquireSteadyTimestampUs() const
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

}  // namespace titati::hardware
