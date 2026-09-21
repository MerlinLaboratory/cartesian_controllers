////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_motion_controller/cartesian_motion_controller.h>

#include <algorithm>
#include <cstdint>
#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_motion_controller
{
CartesianMotionController::CartesianMotionController() : Base::CartesianControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("reference", "pose");
  auto_declare<double>("reference_timeout", 0.5);
  auto_declare<double>("twist_filter_bandwidth", 10.0);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  m_reference = get_node()->get_parameter("reference").as_string();
  m_reference_timeout = get_node()->get_parameter("reference_timeout").as_double();
  m_twist_filter_bandwidth = get_node()->get_parameter("twist_filter_bandwidth").as_double();
  if (m_reference != "pose" && m_reference != "twist")
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'reference' must be either 'pose' or 'twist'.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (m_reference_timeout <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'reference_timeout' must be greater than zero.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (m_twist_filter_bandwidth < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'twist_filter_bandwidth' must be non-negative.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (m_reference == "twist" && !Base::m_ik_solver->supportsDifferentialIK())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Twist reference requires an IK solver with differential IK support. "
                 "Use ik_solver: damped_least_squares.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (m_reference == "pose")
  {
    m_target_frame_subscr = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      get_node()->get_name() + std::string("/target_frame"), 3,
      std::bind(&CartesianMotionController::targetFrameCallback, this, std::placeholders::_1));
  }
  else
  {
    m_target_twist_subscr = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      get_node()->get_name() + std::string("/target_twist"), 3,
      std::bind(&CartesianMotionController::targetTwistCallback, this, std::placeholders::_1));
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // Reset simulation with real joint state
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();

  // Start where we are
  m_target_frame = m_current_frame;
  m_filtered_twist.setZero();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionController::update(const rclcpp::Time & time,
                                                                    const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  if (m_reference == "twist")
  {
    ctrl::Vector6D reference_twist = ctrl::Vector6D::Zero();
    bool has_fresh_command = false;
    const auto command = m_target_twist_buffer.readFromRT();
    // Controller-manager update time and the lifecycle node clock can have
    // different clock types in ROS 2. Compare timestamps from this node's
    // clock only; rclcpp::Time subtraction otherwise throws at runtime.
    const auto command_age_ns = get_node()->get_clock()->now().nanoseconds() -
      command->received_at.nanoseconds();
    if (command->valid && command_age_ns >= 0 &&
        command_age_ns <= static_cast<int64_t>(m_reference_timeout * 1e9))
    {
      reference_twist << command->message.linear.x, command->message.linear.y,
        command->message.linear.z, command->message.angular.x,
        command->message.angular.y, command->message.angular.z;
      has_fresh_command = true;
    }

    if (has_fresh_command)
    {
      // First-order low-pass filter. The bandwidth is the -3 dB cutoff in Hz;
      // setting it to zero disables filtering.
      constexpr double two_pi = 6.28318530717958647692;
      const double alpha = m_twist_filter_bandwidth == 0.0 ? 1.0 :
        1.0 - std::exp(-two_pi * m_twist_filter_bandwidth * period.seconds());
      m_filtered_twist += alpha * (reference_twist - m_filtered_twist);
    }
    else
    {
      // A timeout is a safety stop, not a command to be filtered gradually.
      m_filtered_twist.setZero();
    }

    if (!Base::computeJointVelocityCmds(m_filtered_twist, period))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The configured IK solver does not support twist control.");
      return controller_interface::return_type::ERROR;
    }
    Base::writeJointControlCmds();
    return controller_interface::return_type::OK;
  }

  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the motion error = target - current.
    ctrl::Vector6D error = computeMotionError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianMotionController::computeMotionError()
{
  // Compute motion error wrt robot_base_link
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis);  // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, -max_distance, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}

void CartesianMotionController::targetFrameCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr target)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(target->pose.position.x) || std::isnan(target->pose.position.y) ||
      std::isnan(target->pose.position.z) || std::isnan(target->pose.orientation.x) ||
      std::isnan(target->pose.orientation.y) || std::isnan(target->pose.orientation.z) ||
      std::isnan(target->pose.orientation.w))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target pose. Ignoring input.");
    return;
  }

  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 3000,
                         "Got target pose in wrong reference frame. Expected: %s but got %s",
                         Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  m_target_frame = KDL::Frame(
    KDL::Rotation::Quaternion(target->pose.orientation.x, target->pose.orientation.y,
                              target->pose.orientation.z, target->pose.orientation.w),
    KDL::Vector(target->pose.position.x, target->pose.position.y, target->pose.position.z));
}

void CartesianMotionController::targetTwistCallback(
  const geometry_msgs::msg::Twist::SharedPtr target)
{
  if (!this->isActive())
  {
    return;
  }

  const auto & twist = *target;
  if (std::isnan(twist.linear.x) || std::isnan(twist.linear.y) || std::isnan(twist.linear.z) ||
      std::isnan(twist.angular.x) || std::isnan(twist.angular.y) || std::isnan(twist.angular.z))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 3000,
                         "NaN detected in target twist. Ignoring input.");
    return;
  }
  TwistCommand command;
  command.message = *target;
  command.received_at = get_node()->get_clock()->now();
  command.valid = true;
  m_target_twist_buffer.writeFromNonRT(command);
}

}  // namespace cartesian_motion_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController,
                       controller_interface::ControllerInterface)
