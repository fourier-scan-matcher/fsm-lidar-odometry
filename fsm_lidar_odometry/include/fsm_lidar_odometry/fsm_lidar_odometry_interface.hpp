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

#ifndef FSM_LIDAR_ODOMETRY__FSM_LIDAR_ODOMETRY_INTERFACE_HPP_
#define FSM_LIDAR_ODOMETRY__FSM_LIDAR_ODOMETRY_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "fsm_lidar_odometry/fsm_lidar_odometry.hpp"

namespace fsm_lidar_odometry
{
/**
 * @brief Everything about this node that is a ROS concern.
 * Owns the parameters, the topics, the services and the transform broadcast,
 * and delegates every computation to Matcher.
 */
class Interface : public rclcpp::Node
{
public:
  explicit Interface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void clearTrajectory(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void declareParameters();

  void publishOdometry(const MatchResult& result, const rclcpp::Time& stamp,
    double interval);
  void publishPath(const MatchResult& result, const rclcpp::Time& stamp);
  void publishPose(const MatchResult& result, const rclcpp::Time& stamp);
  void publishTransform(const MatchResult& result, const rclcpp::Time& stamp);

  Parameters readParameters();

  geometry_msgs::msg::Pose retypePose(const Pose& pose) const;

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  void setInitialPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::unique_ptr<Matcher> matcher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_trajectory_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_initial_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::CallbackGroup::SharedPtr service_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  nav_msgs::msg::Path path_;

  std::string initial_pose_topic_;
  std::string global_frame_id_;
  std::string base_frame_id_;
  std::string lo_frame_id_;

  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  bool locked_{true};
};

}

#endif
