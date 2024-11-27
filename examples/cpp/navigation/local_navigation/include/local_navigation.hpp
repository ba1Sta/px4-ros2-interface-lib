/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <nav_msgs/msg/odometry.hpp> // Include for odometry messages

using namespace std::chrono_literals; // NOLINT

class LocalNavigationTest : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  explicit LocalNavigationTest(rclcpp::Node & node)
  : LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalFRD,
      px4_ros2::VelocityFrame::LocalFRD)
  {
    _timer =
      node.create_wall_timer(10ms, [this] {updateLocalPosition();});

    // Create the odometry subscriber
    _odometry_subscriber = _node.create_subscription<nav_msgs::msg::Odometry>(
      "/robot1/odometry/imu", 10, std::bind(&LocalNavigationTest::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node.get_logger(), "example_local_navigation_node running!");
  }

  void updateLocalPosition()
  {
    std::lock_guard<std::mutex> lock(_mutex); // Ensure thread safety
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = _node.get_clock()->now();

    local_position_measurement.position_xy = Eigen::Vector2f {_last_odometry.pose.pose.position.x, -_last_odometry.pose.pose.position.y};
    // local_position_measurement.position_xy_variance = Eigen::Vector2f {_last_odometry.pose.covariance[0], _last_odometry.pose.covariance[7]};
    local_position_measurement.position_xy_variance = Eigen::Vector2f {0.1f, 0.1f};
    // local_position_measurement.velocity_xy = Eigen::Vector2f {_last_odometry.twist.twist.linear.x, -_last_odometry.twist.twist.linear.y};
    // local_position_measurement.velocity_xy_variance = Eigen::Vector2f {_last_odometry.twist.covariance[0], _last_odometry.twist.covariance[7]};
    // local_position_measurement.velocity_xy_variance = Eigen::Vector2f {0.1f, 0.1f};

    local_position_measurement.position_z = -_last_odometry.pose.pose.position.z;
    // local_position_measurement.velocity_z = -_last_odometry.twist.twist.linear.z;
    // local_position_measurement.position_z_variance = _last_odometry.pose.covariance[14];
    local_position_measurement.position_z_variance = 0.1f;
    // local_position_measurement.velocity_z_variance = 0.1f;

    // Transform orientation from FLU to FRD
    // Eigen::Quaternionf orientation_flu(
    //     _last_odometry.pose.pose.orientation.w,
    //     _last_odometry.pose.pose.orientation.x,
    //     _last_odometry.pose.pose.orientation.y,
    //     _last_odometry.pose.pose.orientation.z
    // );
    // Eigen::Quaternionf rotation_180_x(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    // Transform the quaternion to FRD frame
    // Eigen::Quaternionf frd_quat = rotation_180_x * orientation_flu;
    // local_position_measurement.attitude_quaternion = frd_quat;

    local_position_measurement.attitude_quaternion = Eigen::Quaternionf(
                                                                        _last_odometry.pose.pose.orientation.w,
                                                                        _last_odometry.pose.pose.orientation.x,
                                                                        -_last_odometry.pose.pose.orientation.y,
                                                                        -_last_odometry.pose.pose.orientation.z
                                                                      );

    // local_position_measurement.attitude_variance = Eigen::Vector3f {
    //     static_cast<float>(_last_odometry.pose.covariance[21]),
    //     static_cast<float>(_last_odometry.pose.covariance[28]),
    //     static_cast<float>(_last_odometry.pose.covariance[35])
    // };
    local_position_measurement.attitude_variance = Eigen::Vector3f {0.1f, 0.1f, 0.1f};

    try {
      update(local_position_measurement);
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent position update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(_mutex); // Ensure thread safety
    _last_odometry = *msg;
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
  std::mutex _mutex; // Mutex for thread safety
  nav_msgs::msg::Odometry _last_odometry;
};

class ExampleLocalNavigationNode : public rclcpp::Node
{
public:
  ExampleLocalNavigationNode()
  : Node("example_local_navigation_node")
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _interface = std::make_unique<LocalNavigationTest>(*this);

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<LocalNavigationTest> _interface;
};