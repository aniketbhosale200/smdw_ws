// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"

#include <limits>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
using ControllerReferenceMsg =
  mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg;
using ControllerReferenceMsgUnstamped =
  mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsgUnstamped;

void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}
}

namespace mecanum_drive_controller
{
MecanumDriveController::MecanumDriveController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<mecanum_drive_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  auto prepare_lists_with_joint_names =
    [&command_joints = this->command_joint_names_, &state_joints = this->state_joint_names_](
      const std::size_t index, const std::string & command_joint_name,
      const std::string & state_joint_name)
  {
    command_joints[index] = command_joint_name;
    state_joints[index] = state_joint_name.empty() ? command_joint_name : state_joint_name;
  };

  command_joint_names_.resize(4);
  state_joint_names_.resize(4);

  prepare_lists_with_joint_names(
    FRONT_LEFT, params_.front_left_wheel_command_joint_name,
    params_.front_left_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    FRONT_RIGHT, params_.front_right_wheel_command_joint_name,
    params_.front_right_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    REAR_RIGHT, params_.rear_right_wheel_command_joint_name,
    params_.rear_right_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    REAR_LEFT, params_.rear_left_wheel_command_joint_name,
    params_.rear_left_wheel_state_joint_name);

  // Initialize geometry parameters for semi-circular wheels
  circular_radius_ = params_.kinematics.circular_radius;
  semi_major_axis_ = params_.kinematics.semi_major_axis;
  semi_minor_axis_ = params_.kinematics.semi_minor_axis;

  // Initialize odometry with updated parameters
  std::array<double, PLANAR_POINT_DIM> base_frame_offset = {
    {params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y,
     params_.kinematics.base_frame_offset.theta}};
  odometry_.init(get_node()->now(), base_frame_offset);
  odometry_.setWheelsParams(
    params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis,
    circular_radius_, semi_major_axis_, semi_minor_axis_);

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  use_stamped_vel_ = params_.use_stamped_vel;
  if (use_stamped_vel_)
  {
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
      "~/reference", subscribers_qos,
      std::bind(&MecanumDriveController::reference_callback, this, std::placeholders::_1));
  }
  else
  {
    ref_unstamped_subscriber_ = get_node()->create_subscription<ControllerReferenceMsgUnstamped>(
      "~/reference_unstamped", subscribers_qos,
      std::bind(&MecanumDriveController::reference_unstamped_callback, this, std::placeholders::_1));
  }

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);

  try
  {
    odom_s_publisher_ =
      get_node()->create_publisher<OdomStateMsg>("~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during publisher creation: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;
  auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }
  rt_odom_state_publisher_->unlock();

  try
  {
    tf_odom_s_publisher_ =
      get_node()->create_publisher<TfStateMsg>("~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during publisher creation: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  try
  {
    controller_s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during publisher creation: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "MecanumDriveController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumDriveController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }

  const auto age_of_last_command = get_node()->now() - msg->header.stamp;
  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older by %.10f than allowed timeout (%.4f).",
      rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
    reset_controller_reference_msg(msg, get_node());
  }
}

void MecanumDriveController::reference_unstamped_callback(
  const std::shared_ptr<ControllerReferenceMsgUnstamped> msg)
{
  auto twist_stamped = *(input_ref_.readFromNonRT());
  twist_stamped->twist = *msg;
  twist_stamped->header.stamp = get_node()->now();
  input_ref_.writeFromNonRT(twist_stamped);
}

controller_interface::InterfaceConfiguration
MecanumDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(command_joint_names_.size());
  for (const auto & joint : command_joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(state_joint_names_.size() * 2); // Velocity and position
  for (const auto & joint : state_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
MecanumDriveController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(NR_REF_ITFS, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());
  std::vector<std::string> reference_interface_names = {
    "linear/x/velocity", "linear/y/velocity", "angular/z/velocity"};
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names[i], &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

bool MecanumDriveController::on_set_chained_mode(bool chained_mode)
{
  return true || chained_mode;
}

controller_interface::CallbackReturn MecanumDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < NR_CMD_ITFS; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumDriveController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type MecanumDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Forward kinematics (odometry)
  const double wheel_front_left_vel = state_interfaces_[FRONT_LEFT * 2].get_value();  // Velocity
  const double wheel_front_left_pos = state_interfaces_[FRONT_LEFT * 2 + 1].get_value(); // Position
  const double wheel_front_right_vel = state_interfaces_[FRONT_RIGHT * 2].get_value();
  const double wheel_front_right_pos = state_interfaces_[FRONT_RIGHT * 2 + 1].get_value();
  const double wheel_rear_right_vel = state_interfaces_[REAR_RIGHT * 2].get_value();
  const double wheel_rear_right_pos = state_interfaces_[REAR_RIGHT * 2 + 1].get_value();
  const double wheel_rear_left_vel = state_interfaces_[REAR_LEFT * 2].get_value();
  const double wheel_rear_left_pos = state_interfaces_[REAR_LEFT * 2 + 1].get_value();

  if (!std::isnan(wheel_front_left_vel) && !std::isnan(wheel_rear_left_vel) &&
      !std::isnan(wheel_rear_right_vel) && !std::isnan(wheel_front_right_vel) &&
      !std::isnan(wheel_front_left_pos) && !std::isnan(wheel_front_right_pos) &&
      !std::isnan(wheel_rear_right_pos) && !std::isnan(wheel_rear_left_pos))
  {
    odometry_.update(
      wheel_front_left_pos, wheel_front_left_vel,
      wheel_rear_left_pos, wheel_rear_left_vel,
      wheel_rear_right_pos, wheel_rear_right_vel,
      wheel_front_right_pos, wheel_front_right_vel,
      period.seconds());
  }

  auto current_ref = *(input_ref_.readFromRT());
  const auto age_of_last_command = time - current_ref->header.stamp;

  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
  {
    if (!std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.linear.y) &&
        !std::isnan(current_ref->twist.angular.z))
    {
      reference_interfaces_[0] = current_ref->twist.linear.x;
      reference_interfaces_[1] = current_ref->twist.linear.y;
      reference_interfaces_[2] = current_ref->twist.angular.z;

      if (ref_timeout_ == rclcpp::Duration::from_seconds(0))
      {
        current_ref->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        current_ref->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        current_ref->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  else
  {
    if (!std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.linear.y) &&
        !std::isnan(current_ref->twist.angular.z))
    {
      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      reference_interfaces_[2] = 0.0;
      current_ref->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Inverse kinematics (move robot)
  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) &&
      !std::isnan(reference_interfaces_[2]))
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, params_.kinematics.base_frame_offset.theta);
    tf2::Matrix3x3 rotation_from_base_to_center = tf2::Matrix3x3(quaternion);
    tf2::Vector3 velocity_in_base_frame_w_r_t_center_frame_ =
      rotation_from_base_to_center *
      tf2::Vector3(reference_interfaces_[0], reference_interfaces_[1], 0.0);
    tf2::Vector3 linear_trans_from_base_to_center = tf2::Vector3(
      params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y, 0.0);

    velocity_in_center_frame_linear_x_ =
      velocity_in_base_frame_w_r_t_center_frame_.x() +
      linear_trans_from_base_to_center.y() * reference_interfaces_[2];
    velocity_in_center_frame_linear_y_ =
      velocity_in_base_frame_w_r_t_center_frame_.y() -
      linear_trans_from_base_to_center.x() * reference_interfaces_[2];
    velocity_in_center_frame_angular_z_ = reference_interfaces_[2];

    // Compute effective radii based on wheel positions
    const double r_fl = compute_effective_radius(wheel_front_left_pos);
    const double r_fr = compute_effective_radius(wheel_front_right_pos);
    const double r_rr = compute_effective_radius(wheel_rear_right_pos);
    const double r_rl = compute_effective_radius(wheel_rear_left_pos);

    // Compute wheel velocities using dynamic radii
    const double wheel_front_left_vel =
      (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ -
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_) / r_fl;
    const double wheel_front_right_vel =
      (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ +
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_) / r_fr;
    const double wheel_rear_right_vel =
      (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ +
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_) / r_rr;
    const double wheel_rear_left_vel =
      (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_) / r_rl;

    command_interfaces_[FRONT_LEFT].set_value(wheel_front_left_vel);
    command_interfaces_[FRONT_RIGHT].set_value(wheel_front_right_vel);
    command_interfaces_[REAR_RIGHT].set_value(wheel_rear_right_vel);
    command_interfaces_[REAR_LEFT].set_value(wheel_rear_left_vel);
  }
  else
  {
    command_interfaces_[FRONT_LEFT].set_value(0.0);
    command_interfaces_[FRONT_RIGHT].set_value(0.0);
    command_interfaces_[REAR_RIGHT].set_value(0.0);
    command_interfaces_[REAR_LEFT].set_value(0.0);
  }

  // Publish odometry message
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getRz());
  if (rt_odom_state_publisher_->trylock())
  {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.getX();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.getY();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.getVx();
    rt_odom_state_publisher_->msg_.twist.twist.linear.y = odometry_.getVy();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.getWz();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x = odometry_.getX();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y = odometry_.getY();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation = tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = get_node()->now();
    controller_state_publisher_->msg_.front_left_wheel_velocity = wheel_front_left_vel;
    controller_state_publisher_->msg_.front_right_wheel_velocity = wheel_front_right_vel;
    controller_state_publisher_->msg_.back_right_wheel_velocity = wheel_rear_right_vel;
    controller_state_publisher_->msg_.back_left_wheel_velocity = wheel_rear_left_vel;
    controller_state_publisher_->msg_.reference_velocity.linear.x = reference_interfaces_[0];
    controller_state_publisher_->msg_.reference_velocity.linear.y = reference_interfaces_[1];
    controller_state_publisher_->msg_.reference_velocity.angular.z = reference_interfaces_[2];
    controller_state_publisher_->unlockAndPublish();
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

double MecanumDriveController::compute_effective_radius(double theta) const
{
  // Normalize theta to [0, 2π)
  theta = std::fmod(theta, 2 * M_PI);
  if (theta < 0) theta += 2 * M_PI;

  // Circular part: 0 to π, Elliptical part: π to 2π
  if (theta < M_PI)
  {
    return circular_radius_;  // Constant radius for circular half
  }
  else
  {
    double phi = theta - M_PI;  // Adjust angle for elliptical part
    return (semi_major_axis_ * semi_minor_axis_) /
           std::sqrt(
             std::pow(semi_minor_axis_ * std::cos(phi), 2) +
             std::pow(semi_major_axis_ * std::sin(phi), 2));  // Ellipse radius formula
  }
}

}  // namespace mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecanum_drive_controller::MecanumDriveController,
  controller_interface::ChainableControllerInterface)
