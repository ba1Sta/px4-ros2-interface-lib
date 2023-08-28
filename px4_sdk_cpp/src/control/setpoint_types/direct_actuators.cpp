/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_sdk/control/setpoint_types/direct_actuators.hpp>


namespace px4_sdk
{

DirectActuatorsSetpointType::DirectActuatorsSetpointType(
  rclcpp::Node & node,
  const std::string & topic_namespace_prefix)
: _node(node)
{
  _actuator_motors_pub = node.create_publisher<px4_msgs::msg::ActuatorMotors>(
    topic_namespace_prefix + "/fmu/in/actuator_motors", 1);
  _actuator_servos_pub = node.create_publisher<px4_msgs::msg::ActuatorServos>(
    topic_namespace_prefix + "/fmu/in/actuator_servos", 1);
}

void DirectActuatorsSetpointType::updateMotors(
  const Eigen::Vector<float,
  kMaxNumMotors> & motor_commands)
{
  onUpdate();

  px4_msgs::msg::ActuatorMotors sp_motors{};
  for (int i = 0; i < kMaxNumMotors; ++i) {
    sp_motors.control[i] = motor_commands(i);
  }
  sp_motors.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _actuator_motors_pub->publish(sp_motors);
}

void DirectActuatorsSetpointType::updateServos(
  const Eigen::Vector<float,
  kMaxNumServos> & servo_commands)
{
  onUpdate();

  px4_msgs::msg::ActuatorServos sp_servos{};
  for (int i = 0; i < kMaxNumServos; ++i) {
    sp_servos.control[i] = servo_commands(i);
  }
  sp_servos.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _actuator_servos_pub->publish(sp_servos);
}

SetpointBase::Configuration DirectActuatorsSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = false;
  config.rates_enabled = false;
  config.attitude_enabled = false;
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_sdk