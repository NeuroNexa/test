// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "titati_controller/titati_controller_node.hpp"

#include "pluginlib/class_list_macros.hpp"
namespace titati_locomotion
{
TitatiController::TitatiController() {}

controller_interface::CallbackReturn TitatiController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  param_listener_ = std::make_shared<titati_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  setup_controller();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitatiController::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  } 

  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }
  cmd_vel_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::Twist>(
      tita_topic::manager_twist_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&TitatiController::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      tita_topic::manager_pose_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&TitatiController::posestamped_cb, this, std::placeholders::_1));      
  fsm_goal_subscription_ =
    get_node()->create_subscription<std_msgs::msg::String>(
      tita_topic::manager_key_command, rclcpp::SensorDataQoS().reliable(),
      std::bind(&TitatiController::fsm_goal_cb, this, std::placeholders::_1));
  left_battery_info_subscription_ =
    get_node()->create_subscription<sensor_msgs::msg::BatteryState>(
      tita_topic::kleft_battery_topic, rclcpp::SensorDataQoS().reliable(),
      std::bind(&TitatiController::left_battery_cb, this, std::placeholders::_1));
  right_battery_info_subscription_ =
    get_node()->create_subscription<sensor_msgs::msg::BatteryState>(
      tita_topic::kright_battery_topic, rclcpp::SensorDataQoS().reliable(),
      std::bind(&TitatiController::right_battery_cb, this, std::placeholders::_1));

  odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  odom_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TitatiController::odom_cb, this));
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());    
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TitatiController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
      RCLCPP_DEBUG(get_node()->get_logger(), "Get joint cmd : %s", joint->name.c_str());
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration TitatiController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type TitatiController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    update_control_parameters();
  }
  // controlData_->params->dt_ = period.seconds();

  mainLoopThread();
  // TODO:
  // if (!lqr_thread_running_) {
  //   lqr_thread_running_ = true;
  //   lqr_thread_ = std::thread([this]() {
  //     while (lqr_thread_running_) {
  //       lqrLoopThread();
  //     }
  //   });
  // }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TitatiController::on_activate(const rclcpp_lifecycle::State &)
{
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    // const auto position_command_handle = std::find_if(
    //   command_interfaces_.begin(), command_interfaces_.end(),
    //   [&joint](const auto & interface)
    //   {
    //     return interface.get_prefix_name() == joint->name &&
    //            interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    //   });
    // if (position_command_handle == command_interfaces_.end()) {
    //   RCLCPP_ERROR(
    //     get_node()->get_logger(), "Unable to obtain joint command handle for %s",
    //     joint->name.c_str());
    //   return controller_interface::CallbackReturn::FAILURE;
    // }
    // joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    // const auto velocity_command_handle = std::find_if(
    //   command_interfaces_.begin(), command_interfaces_.end(),
    //   [&joint](const auto & interface)
    //   {
    //     return interface.get_prefix_name() == joint->name &&
    //            interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    //   });
    // if (velocity_command_handle == command_interfaces_.end()) {
    //   RCLCPP_ERROR(
    //     get_node()->get_logger(), "Unable to obtain joint command handle for %s",
    //     joint->name.c_str());
    //   return controller_interface::CallbackReturn::FAILURE;
    // }
    // joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);
    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitatiController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitatiController::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitatiController::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitatiController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}

TitatiController::~TitatiController() {}

// TODO:
void TitatiController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
  // (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  vel_gait_cmd_->x_vel_cmd = msg->linear.x;
  vel_gait_cmd_->yaw_turn_rate = msg->angular.z;
  joy_count_ = 0;
}

void TitatiController::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  // cmd->pose_position_[Y] = msg->pose.position.y;
  // cmd->pose_position_[Z] = msg->pose.position.z;
  // cmd->pose_orientation_[QX] = msg->pose.orientation.x;
  // cmd->pose_orientation_[QY] = msg->pose.orientation.y;
  // cmd->pose_orientation_[QZ] = msg->pose.orientation.z;
  // cmd->pose_orientation_[QW] = msg->pose.orientation.w;
}

void TitatiController::fsm_goal_cb(
  const std_msgs::msg::String::SharedPtr msg)
{
  // (void)msg;
  // auto cmd = controlData_->state_command->rc_command_;
  // cmd->fsm_name_ = msg->data;
  if(msg->data == "idle" || msg->data == "transform_down")
    mode_ = 3;
  else if(msg->data == "transform_up")
    mode_ = 1;
  else if(msg->data == "balance_stand")
    mode_ = 4;
  else if(msg->data == "jump")
    mode_ = 5;
  else if(msg->data == "joint_pd")
    mode_ = 6;
  else
    mode_ = 0;
}

void TitatiController::odom_cb()
{
  std::string frame_prefix = get_node()->get_parameter("frame_prefix").as_string();

  // 发布TransformStamped消息
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = get_node()->now();
  odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = frame_prefix + controlData_->params->base_name_;
  // odom_trans.transform.translation.x = controlData_->state_estimate->position(X);
  // odom_trans.transform.translation.y = controlData_->state_estimate->position(Y);
  // odom_trans.transform.translation.z = controlData_->state_estimate->position(Z);
  // odom_trans.transform.rotation = tf2::toMsg(tf2::Quaternion(
  //   controlData_->state_estimate->orientation(QX), controlData_->state_estimate->orientation(QY), controlData_->state_estimate->orientation(QZ),
  //   controlData_->state_estimate->orientation(QW)));
  odom_broadcaster_->sendTransform(odom_trans);

  // 发布Odometry消息
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = "odom";
  // odom_msg.child_frame_id = frame_prefix + controlData_->params->base_name_;
  // odom_msg.pose.pose.position.x = controlData_->state_estimate->position(X);
  // odom_msg.pose.pose.position.y = controlData_->state_estimate->position(Y);
  // odom_msg.pose.pose.position.z = controlData_->state_estimate->position(Z);
  // odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
  //   controlData_->state_estimate->orientation(QX), controlData_->state_estimate->orientation(QY), controlData_->state_estimate->orientation(QZ),
  //   controlData_->state_estimate->orientation(QW)));
  // odom_msg.twist.twist.linear.x = controlData_->state_estimate->vWorld(X);
  // odom_msg.twist.twist.linear.y = controlData_->state_estimate->vWorld(Y);
  // odom_msg.twist.twist.linear.z = controlData_->state_estimate->vWorld(Z);
  // odom_msg.twist.twist.angular.x = controlData_->state_estimate->omegaWorld(X);
  // odom_msg.twist.twist.angular.y = controlData_->state_estimate->omegaWorld(Y);
  // odom_msg.twist.twist.angular.z = controlData_->state_estimate->omegaWorld(Z);
  odom_msg.twist.covariance = {};
  odom_publisher_->publish(odom_msg);
}

void TitatiController::left_battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_msg)
{
  if(battery_msg->location != "online" || static_cast<int>(battery_msg->percentage) < 20)
    left_battery_flag_ = true;
  else
    left_battery_flag_ = false;
}

void TitatiController::right_battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_msg)
{
  if(battery_msg->location != "online" || static_cast<int>(battery_msg->percentage) < 20)
    right_battery_flag_ = true;
  else
    right_battery_flag_ = false;
}


void TitatiController::setup_controller(){
  vel_gait_cmd_ = std::make_shared<VelGaitCmd>();
  get_node()->get_parameter<float>("slope_commmand_gain_x", vel_gait_cmd_->slope_x_yaw_gain[0]);
  get_node()->get_parameter<float>("slope_commmand_bias_x", vel_gait_cmd_->slope_x_yaw_bias[0]);
  get_node()->get_parameter<float>("slope_commmand_gain_yaw", vel_gait_cmd_->slope_x_yaw_gain[1]);
  get_node()->get_parameter<float>("slope_commmand_bias_yaw", vel_gait_cmd_->slope_x_yaw_bias[1]);
  get_node()->get_parameter<float>("stair_commmand_gain_x", vel_gait_cmd_->stair_x_yaw_gain[0]);
  get_node()->get_parameter<float>("stair_commmand_bias_x", vel_gait_cmd_->stair_x_yaw_bias[0]);
  get_node()->get_parameter<float>("stair_commmand_gain_yaw", vel_gait_cmd_->stair_x_yaw_gain[1]);
  get_node()->get_parameter<float>("stair_commmand_bias_yaw", vel_gait_cmd_->stair_x_yaw_bias[1]);
  get_node()->get_parameter<float>("high_commmand_gain_x", vel_gait_cmd_->high_x_yaw_gain[0]);
  get_node()->get_parameter<float>("high_commmand_bias_x", vel_gait_cmd_->high_x_yaw_bias[0]);
  get_node()->get_parameter<float>("high_commmand_gain_yaw", vel_gait_cmd_->high_x_yaw_gain[1]);
  get_node()->get_parameter<float>("high_commmand_bias_yaw", vel_gait_cmd_->high_x_yaw_bias[1]);
  robot_task_ = std::make_shared<WheelQuadruped_Task>(torque_, vel_gait_cmd_);
  // controlData_ = std::make_shared<ControlFSMData>(joint_names_.size());
  setup_control_parameters();
  setup_state_estimate();
  // FSMController_ = std::make_shared<FSM>(controlData_);
}
void TitatiController::mainLoopThread()
{
  // wbcTimer_.startTimer();
  // State Update
  // int id = 0;
  // auto state = controlData_->low_state;
  // for (std::shared_ptr<Joint> joint : joints_) {
  //   state->q(id) = joint->position_handle->get().get_value();
  //   state->dq(id) = joint->velocity_handle->get().get_value();
  //   state->tau_est(id) = joint->effort_handle->get().get_value();
  //   id++;
  // }
  // state->accelerometer = vector3_t(
  //   imu_sensor_->get_linear_acceleration()[0], imu_sensor_->get_linear_acceleration()[1],
  //   imu_sensor_->get_linear_acceleration()[2]);
  // state->gyro = vector3_t(
  //   imu_sensor_->get_angular_velocity()[0], imu_sensor_->get_angular_velocity()[1],
  //   imu_sensor_->get_angular_velocity()[2]);
  // state->quat = quaternion_t(
  //   imu_sensor_->get_orientation()[3], imu_sensor_->get_orientation()[0],
  //   imu_sensor_->get_orientation()[1], imu_sensor_->get_orientation()[2]);
  // // Control Update
  // if (init_estimator_count_ > 100)
  //   controlData_->state_estimator->run();
  // else
  //   init_estimator_count_++;
  // controlData_->pinocchio_model->updateState();
  // controlData_->state_command->convertToStateCommands();
  // FSMController_->run();
  size_t id = 0;
  for (std::shared_ptr<Joint> joint : joints_) {
    // if(id == 1)
    //   motor_pos_[id] = joint->position_handle->get().get_value() + 0.7;
    // else if(id == 5)
    //   motor_pos_[id] = joint->position_handle->get().get_value() + 0.57;
    // else
    motor_pos_[id] = joint->position_handle->get().get_value();
    motor_vel_[id] = joint->velocity_handle->get().get_value();
    id++;
  }
  float pos_tmp[4] = {0.}, vel_tmp[4] = {0.};
  for(int i = 0; i < 4; i++) {
    pos_tmp[i] = motor_pos_[8+i];
    vel_tmp[i] = motor_vel_[8+i];
    motor_pos_[8+i] = motor_pos_[12+i];
    motor_vel_[8+i] = motor_vel_[12+i];
    motor_pos_[12+i] = pos_tmp[i];
    motor_vel_[12+i] = vel_tmp[i];
  }
  quat_[0] = imu_sensor_->get_orientation()[3];
  for(size_t id = 0; id < 3; id++) {
    quat_[id + 1] = imu_sensor_->get_orientation()[id];
    gyro_[id] = imu_sensor_->get_angular_velocity()[id];
    accl_[id] = imu_sensor_->get_linear_acceleration()[id];
  }
  if (!init_flag_) {
    init_flag_ = true;
    robot_task_->Run_Init();
  }
  
  joy_count_++;
  if((left_battery_flag_ && right_battery_flag_) || joy_count_ > 250)
    mode_ = 3;

  robot_task_->Run_Update(quat_, gyro_, accl_, motor_pos_, motor_vel_);
  robot_task_->Run_Ctrl(mode_);

  float t_tmp[4] = {0.};
  for(int i = 0; i < 4; i++) {
    t_tmp[i] = torque_[8+i];
    torque_[8+i] = torque_[12+i];
    torque_[12+i] = t_tmp[i];
  }

  for (int i = 0; i < 16; i++) {
    if (i % 4 != 3)
      torque_[i] *= 1.25;
  }
  for (int i = 0; i < 16; i++) {
    if (i % 4 != 3) {
      if (torque_[i] < -65.0)
        torque_[i] = -65.0;
      else if (torque_[i] > 65.0)
        torque_[i] = 65.0;
    } else {
      if (torque_[i] < -10.0)
        torque_[i] = -10.0;
      else if (torque_[i] > 10.0)
        torque_[i] = 10.0;
    }
  }

  // Update torque
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(torque_[id]);
  }
  
  // wbcTimer_.endTimer();

  // TODO：debug infomation
  static int count = 0;
  if(count % 2500 == 0){
    for (uint id = 0; id < joints_.size(); id++){
      RCLCPP_ERROR(get_node()->get_logger(), "%d joint state %f",id,motor_pos_[id]);
      RCLCPP_ERROR(get_node()->get_logger(), "%d joint t %f",id,torque_[id]);
    }
    RCLCPP_ERROR(get_node()->get_logger(), "acc %f",imu_sensor_->get_linear_acceleration()[0]);
    RCLCPP_ERROR(get_node()->get_logger(), "acc %f",imu_sensor_->get_linear_acceleration()[1]);
    RCLCPP_ERROR(get_node()->get_logger(), "acc %f",imu_sensor_->get_linear_acceleration()[2]);
    RCLCPP_ERROR(get_node()->get_logger(), "gyro %f",imu_sensor_->get_angular_velocity()[0]);
    RCLCPP_ERROR(get_node()->get_logger(), "gyro %f",imu_sensor_->get_angular_velocity()[1]);
    RCLCPP_ERROR(get_node()->get_logger(), "gyro %f",imu_sensor_->get_angular_velocity()[2]);
    RCLCPP_ERROR(get_node()->get_logger(), "quat %f",imu_sensor_->get_orientation()[0]);
    RCLCPP_ERROR(get_node()->get_logger(), "quat %f",imu_sensor_->get_orientation()[1]);
    RCLCPP_ERROR(get_node()->get_logger(), "quat %f",imu_sensor_->get_orientation()[2]);
    RCLCPP_ERROR(get_node()->get_logger(), "quat %f",imu_sensor_->get_orientation()[3]);
    RCLCPP_ERROR(get_node()->get_logger(), "mode %d",mode_);
  }
  if(count % 400 == 0){
    RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n### MPC Benchmarking");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].");
  // RCLCPP_DEBUG(get_node()->get_logger(), "########################################################################");
    RCLCPP_DEBUG(get_node()->get_logger(), "\n### WBC Benchmarking");
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Maximum : %f [ms].", wbcTimer_.getMaxIntervalInMilliseconds());
  // RCLCPP_DEBUG(get_node()->get_logger(), "\n###   Average : %f [ms].", wbcTimer_.getAverageInMilliseconds());
  }
  count++;
}


void TitatiController::setup_control_parameters()
{

}

void TitatiController::update_control_parameters()
{

  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
}


void TitatiController::setup_state_estimate()
{

}

void CheaterTitatiController::setup_state_estimate()
{

}

}  // namespace tita_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(titati_locomotion::TitatiController, controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(
//   tita_locomotion::CheaterTitatiController, controller_interface::ControllerInterface)
