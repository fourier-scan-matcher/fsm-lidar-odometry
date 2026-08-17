// Copyright 2022 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "fsm_lidar_odometry/fsm_lidar_odometry_interface.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace fsm_lidar_odometry
{
namespace
{
bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);

  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

rclcpp::QoS scanQos(const std::string& reliability, int depth)
{
  rclcpp::QoS qos(static_cast<std::size_t>(depth));

  if (reliability == "best_effort")
    qos.best_effort();
  else
    qos.reliable();

  return qos;
}

std::optional<double> yawOf(const geometry_msgs::msg::Quaternion& orientation)
{
  if (!isFinite(orientation.x) || !isFinite(orientation.y) ||
    !isFinite(orientation.z) || !isFinite(orientation.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  constexpr double kMinimumQuaternionNormSquared = 1e-9;

  if (norm_squared < kMinimumQuaternionNormSquared)
    return std::nullopt;

  const tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  return FSM::Utils::wrapAngle(yaw);
}

}

Interface::Interface(const rclcpp::NodeOptions& options)
: rclcpp::Node("fsm_lidar_odometry", options)
{
  declareParameters();

  const Parameters parameters = readParameters();

  if (const std::string problem = validate(parameters); !problem.empty())
  {
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  setDiagnosticSink(
    [this](const std::string& message)
    {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    });

  matcher_ = std::make_unique<Matcher>(parameters);

  global_frame_id_ = get_parameter("global_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  lo_frame_id_ = get_parameter("lo_frame_id").as_string();
  initial_pose_topic_ = get_parameter("initial_pose_topic").as_string();

  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    get_parameter("lo_topic").as_string(), 1);
  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    get_parameter("pose_estimate_topic").as_string(), 1);
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    get_parameter("path_estimate_topic").as_string(), 1);

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    get_parameter("scan_topic").as_string(),
    scanQos(get_parameter("scan_qos_reliability").as_string(),
      static_cast<int>(get_parameter("scan_qos_depth").as_int())),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
    {
      scanCallback(std::move(scan));
    });

  service_group_ =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  clear_trajectory_service_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_estimated_trajectory",
    std::bind(&Interface::clearTrajectory, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  set_initial_pose_service_ = create_service<std_srvs::srv::Trigger>(
    "~/set_initial_pose",
    std::bind(&Interface::setInitialPose, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "~/start",
    std::bind(&Interface::start, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "~/stop",
    std::bind(&Interface::stop, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_.header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Initialised.");
  RCLCPP_INFO(get_logger(),
    "To start production of lidar odometry issue"
    " ros2 service call %s/start std_srvs/srv/Trigger",
    get_name());
}

void Interface::clearTrajectory(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Clearing trajectory ...");

  const bool was_locked = locked_;
  locked_ = true;
  matcher_->clearTrajectory();
  path_.poses.clear();
  locked_ = was_locked;

  response->success = true;
  response->message = "estimated trajectory cleared";
}

void Interface::declareParameters()
{
  const std::string name = get_name();

  declare_parameter("scan_topic", "/base_scan");
  declare_parameter("initial_pose_topic", name + "/initial_pose");
  declare_parameter("pose_estimate_topic", name + "/pose_estimate");
  declare_parameter("path_estimate_topic", name + "/path_estimate");
  declare_parameter("lo_topic", name + "/lo");

  declare_parameter("global_frame_id", "map");
  declare_parameter("base_frame_id", "base_laser_link");
  declare_parameter("lo_frame_id", "lo");

  declare_parameter("size_scan", 0);
  declare_parameter("min_magnification_size", 0);
  declare_parameter("max_magnification_size", 3);
  declare_parameter("num_iterations", 2);
  declare_parameter("xy_bound", 0.2);
  declare_parameter("t_bound", M_PI / 4);
  declare_parameter("max_counter", 200);
  declare_parameter("max_recoveries", 10);
  declare_parameter("rng_seed", 0);
  declare_parameter("ray_search", "angular");

  declare_parameter("scan_qos_reliability", "reliable");
  declare_parameter("scan_qos_depth", 1);
}

void Interface::publishOdometry(const MatchResult& result,
  const rclcpp::Time& stamp, double interval)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = lo_frame_id_;
  message.child_frame_id = base_frame_id_;

  message.pose.pose = retypePose(result.increment);

  const std::array<double, 36> covariance{};
  message.pose.covariance = covariance;
  message.twist.covariance = covariance;

  if (interval > 0.0)
  {
    message.twist.twist.linear.x = result.increment.x / interval;
    message.twist.twist.linear.y = result.increment.y / interval;
    message.twist.twist.angular.z = result.increment.t / interval;
  }
  else
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Scans %.9f seconds apart carry no velocity between them, so this "
      "odometry message reports none. The displacement it reports stands.",
      interval);
  }

  odometry_publisher_->publish(message);
}

void Interface::publishPath(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = global_frame_id_;
  pose.pose = retypePose(result.accumulated);

  path_.header.stamp = stamp;
  path_.header.frame_id = global_frame_id_;
  path_.poses.push_back(pose);

  path_publisher_->publish(path_);
}

void Interface::publishPose(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  message.pose = retypePose(result.accumulated);

  pose_publisher_->publish(message);
}

void Interface::publishTransform(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = lo_frame_id_;
  message.child_frame_id = base_frame_id_;

  message.transform.translation.x = result.increment.x;
  message.transform.translation.y = result.increment.y;
  message.transform.translation.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, result.increment.t);
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();

  transform_broadcaster_->sendTransform(message);
}

Parameters Interface::readParameters()
{
  Parameters parameters;
  parameters.size_scan =
    static_cast<std::size_t>(get_parameter("size_scan").as_int());
  parameters.num_iterations =
    static_cast<unsigned int>(get_parameter("num_iterations").as_int());
  parameters.xy_bound = get_parameter("xy_bound").as_double();
  parameters.t_bound = get_parameter("t_bound").as_double();
  parameters.max_counter =
    static_cast<unsigned int>(get_parameter("max_counter").as_int());
  parameters.min_magnification_size =
    static_cast<unsigned int>(get_parameter("min_magnification_size").as_int());
  parameters.max_magnification_size =
    static_cast<unsigned int>(get_parameter("max_magnification_size").as_int());
  parameters.max_recoveries =
    static_cast<unsigned int>(get_parameter("max_recoveries").as_int());
  parameters.rng_seed =
    static_cast<unsigned int>(get_parameter("rng_seed").as_int());
  parameters.ray_search = get_parameter("ray_search").as_string();
  return parameters;
}

geometry_msgs::msg::Pose Interface::retypePose(const Pose& pose) const
{
  geometry_msgs::msg::Pose message;

  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, pose.t);
  quaternion.normalize();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();

  return message;
}

void Interface::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (locked_)
  {
    RCLCPP_WARN(get_logger(), "Will not process this scan");
    return;
  }

  const rclcpp::Time stamp(scan->header.stamp);

  const std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());

  const std::expected<MatchResult, MatchError> result =
    matcher_->process(ranges);

  if (!result.has_value())
  {
    if (result.error() == MatchError::scan_too_short)
    {
      RCLCPP_WARN(get_logger(),
        "Scan carries %zu ranges but size_scan is %zu; ignoring it",
        ranges.size(), matcher_->parameters().size_scan);
    }

    previous_stamp_ = stamp;
    return;
  }

  const double interval = (stamp - previous_stamp_).seconds();
  previous_stamp_ = stamp;

  RCLCPP_INFO(get_logger(), "FSM executed in %.1f ms",
    1000.0 * result->execution_time);

  if (result->execution_time > interval)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "A match takes %.0f ms but scans arrive every %.0f ms, so scans are "
      "being dropped. Matching is at %zu rays; setting size_scan below that "
      "matches fewer and takes less time over it.",
      1000.0 * result->execution_time, 1000.0 * interval,
      matcher_->matchSize());
  }

  if (result->num_recoveries > 0)
  {
    RCLCPP_WARN(get_logger(),
      "Match needed %u recovery attempts; this scan pair is not reproducible "
      "across builds", result->num_recoveries);
  }

  publishTransform(*result, stamp);
  publishOdometry(*result, stamp, interval);
  publishPose(*result, stamp);
  publishPath(*result, stamp);
}

void Interface::setInitialPose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;

  const auto listener = std::make_shared<rclcpp::Node>(
    std::string(get_name()) + "_initial_pose_listener");

  const bool arrived =
    rclcpp::wait_for_message(message, listener, initial_pose_topic_);

  if (!arrived)
  {
    response->success = false;
    response->message = "no message on " + initial_pose_topic_;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!isFinite(message.pose.pose.position.x) ||
    !isFinite(message.pose.pose.position.y))
  {
    response->success = false;
    response->message = "initial pose position is not a finite point";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::optional<double> yaw = yawOf(message.pose.pose.orientation);

  if (!yaw.has_value())
  {
    response->success = false;
    response->message =
      "initial pose orientation carries no heading: a quaternion must be "
      "finite and must have a length to be normalised by, and four zeros "
      "have neither";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const Pose pose{
    message.pose.pose.position.x,
    message.pose.pose.position.y,
    *yaw};

  matcher_->setInitialPose(pose);

  response->success = true;
  response->message = "initial pose set";
  RCLCPP_INFO(get_logger(), "Setting initial pose to (%.2f,%.2f,%.2f)",
    pose.x, pose.y, pose.t);
}

void Interface::start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "lidar odometry is now available.");
  locked_ = false;

  response->success = true;
  response->message = "lidar odometry started";
}

void Interface::stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "lidar odometry is shut down.");
  locked_ = true;

  response->success = true;
  response->message = "lidar odometry stopped";
}

}
