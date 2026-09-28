// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navigation_neupan_bridge
{
namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

double cross(const Point2D & origin, const Point2D & a, const Point2D & b)
{
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<Point2D> convexHull(std::vector<Point2D> points)
{
  std::sort(points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
    return a.x == b.x ? a.y < b.y : a.x < b.x;
  });
  points.erase(std::unique(points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
    return std::abs(a.x - b.x) < 1e-6 && std::abs(a.y - b.y) < 1e-6;
  }), points.end());
  if (points.size() < 3) {
    return {};
  }
  std::vector<Point2D> hull;
  for (const auto & point : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const size_t lower_size = hull.size();
  for (auto iterator = points.rbegin(); iterator != points.rend(); ++iterator) {
    while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*iterator);
  }
  hull.pop_back();
  return hull;
}

bool validJointState(const sensor_msgs::msg::JointState & message, const std::string & prefix)
{
  if (message.name.size() != message.position.size()) {
    return false;
  }
  int found = 0;
  for (size_t index = 0; index < message.name.size(); ++index) {
    if (message.name[index].rfind(prefix + "joint", 0) == 0) {
      if (!std::isfinite(message.position[index])) {
        return false;
      }
      ++found;
    }
  }
  return found == 7;
}
}  // namespace

class R1ArmFootprintNode : public rclcpp::Node
{
public:
  R1ArmFootprintNode()
  : Node("r1_arm_footprint"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 0.50);
    update_frequency_ = declare_parameter<double>("update_frequency", 10.0);
    link_radius_m_ = declare_parameter<double>("link_radius_m", 0.12);
    tool_radius_m_ = declare_parameter<double>("tool_radius_m", 0.16);
    projection_margin_m_ = declare_parameter<double>("projection_margin_m", 0.03);
    circle_samples_ = declare_parameter<int>("circle_samples", 12);
    base_footprint_ = declare_parameter<std::vector<double>>(
      "base_footprint_points", std::vector<double>{0.341, 0.266, 0.341, -0.266, -0.341, -0.266, -0.341, 0.266});
    fallback_footprint_ = declare_parameter<std::vector<double>>(
      "fallback_footprint_points", std::vector<double>{
        1.05, 1.30, 1.05, -1.30, -1.05, -1.30, -1.05, 1.30});
    if (state_timeout_sec_ <= 0.0 || update_frequency_ <= 0.0 || link_radius_m_ <= 0.0 ||
      tool_radius_m_ <= 0.0 || projection_margin_m_ < 0.0 || circle_samples_ < 6 ||
      base_footprint_.size() < 6 || base_footprint_.size() % 2 != 0 ||
      fallback_footprint_.size() < 6 || fallback_footprint_.size() % 2 != 0)
    {
      throw std::invalid_argument("invalid arm footprint parameters");
    }

    left_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/left_arm/joint_states", rclcpp::QoS(20),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        left_state_received_ = std::chrono::steady_clock::now();
        left_state_valid_ = validJointState(*message, "left_");
      });
    right_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/right_arm/joint_states", rclcpp::QoS(20),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        right_state_received_ = std::chrono::steady_clock::now();
        right_state_valid_ = validJointState(*message, "right_");
      });
    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/r1_arm_footprint", rclcpp::QoS(10));
    local_costmap_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/local_costmap/footprint", rclcpp::QoS(10));
    global_costmap_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/global_costmap/footprint", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/r1_arm_footprint/status", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency_),
      std::bind(&R1ArmFootprintNode::tick, this));
  }

private:
  void addDisc(std::vector<Point2D> & points, const Point2D & center, const double radius) const
  {
    for (int index = 0; index < circle_samples_; ++index) {
      const double angle = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(circle_samples_);
      points.push_back({center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)});
    }
  }

  void publishStatus(const std::string & state, const std::string & reason, const size_t points)
  {
    std_msgs::msg::String message;
    message.data = "{\"schema\":\"r1_arm_footprint_status_v1\",\"state\":\"" + state +
      "\",\"reason\":\"" + reason + "\",\"points\":" + std::to_string(points) + "}";
    status_pub_->publish(message);
  }

  void publishPolygon(const std::vector<Point2D> & points)
  {
    geometry_msgs::msg::Polygon polygon;
    for (const auto & point : points) {
      geometry_msgs::msg::Point32 output;
      output.x = static_cast<float>(point.x);
      output.y = static_cast<float>(point.y);
      output.z = 0.0F;
      polygon.points.push_back(output);
    }
    geometry_msgs::msg::PolygonStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = base_frame_;
    stamped.polygon = polygon;
    footprint_pub_->publish(stamped);
    local_costmap_pub_->publish(polygon);
    global_costmap_pub_->publish(polygon);
  }

  void publishFallback(const std::string & reason)
  {
    std::vector<Point2D> points;
    for (size_t index = 0; index < fallback_footprint_.size(); index += 2) {
      points.push_back({fallback_footprint_[index], fallback_footprint_[index + 1]});
    }
    const auto hull = convexHull(std::move(points));
    if (hull.size() < 3) {
      publishStatus("FAULT", "fallback_footprint_invalid", 0);
      return;
    }
    publishPolygon(hull);
    publishStatus("DEGRADED", reason, hull.size());
  }

  void tick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    bool states_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      states_ready = left_state_valid_ && right_state_valid_ &&
        std::chrono::duration<double>(steady_now - left_state_received_).count() <= state_timeout_sec_ &&
        std::chrono::duration<double>(steady_now - right_state_received_).count() <= state_timeout_sec_;
    }
    if (!states_ready) {
      publishFallback("arm_joint_state_fallback");
      return;
    }

    std::vector<Point2D> points;
    for (size_t index = 0; index < base_footprint_.size(); index += 2) {
      points.push_back({base_footprint_[index], base_footprint_[index + 1]});
    }
    const std::vector<std::pair<std::string, double>> frames = {
      {"left_base", link_radius_m_}, {"left_link1", link_radius_m_},
      {"left_link2", link_radius_m_}, {"left_link3", link_radius_m_},
      {"left_link4", link_radius_m_}, {"left_link5", link_radius_m_},
      {"left_link6", link_radius_m_}, {"left_link7", link_radius_m_},
      {"left_tcp", tool_radius_m_}, {"left_moduan_link", tool_radius_m_},
      {"L_omniPicker_base_link", tool_radius_m_},
      {"right_base", link_radius_m_}, {"right_link1", link_radius_m_},
      {"right_link2", link_radius_m_}, {"right_link3", link_radius_m_},
      {"right_link4", link_radius_m_}, {"right_link5", link_radius_m_},
      {"right_link6", link_radius_m_}, {"right_link7", link_radius_m_},
      {"right_tcp", tool_radius_m_}, {"right_moduan_link", tool_radius_m_},
      {"R_omniPicker_base_link", tool_radius_m_},
    };
    try {
      for (const auto & [frame, radius] : frames) {
        const auto transform = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
        addDisc(points, {transform.transform.translation.x, transform.transform.translation.y},
          radius + projection_margin_m_);
      }
    } catch (const tf2::TransformException &) {
      publishFallback("arm_tf_fallback");
      return;
    }
    const auto hull = convexHull(std::move(points));
    if (hull.size() < 3) {
      publishStatus("FAULT", "arm_projection_invalid", 0);
      return;
    }
    publishPolygon(hull);
    publishStatus("READY", "dynamic_projection_ready", hull.size());
  }

  std::string base_frame_;
  double state_timeout_sec_{0.5};
  double update_frequency_{10.0};
  double link_radius_m_{0.12};
  double tool_radius_m_{0.16};
  double projection_margin_m_{0.03};
  int circle_samples_{12};
  std::vector<double> base_footprint_;
  std::vector<double> fallback_footprint_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex mutex_;
  SteadyTime left_state_received_{};
  SteadyTime right_state_received_{};
  bool left_state_valid_{false};
  bool right_state_valid_{false};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation_neupan_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_neupan_bridge::R1ArmFootprintNode>());
  rclcpp::shutdown();
  return 0;
}
