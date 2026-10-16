// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometric_shapes/mesh_operations.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
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

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct MeshPart
{
  std::string frame;
  std::string resource;
  double scale{1.0};
  std::vector<Point3D> vertices;
};

double cross(const Point2D & origin, const Point2D & a, const Point2D & b)
{
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

double pointSegmentDistanceSquared(
  const Point2D & point, const Point2D & start,
  const Point2D & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1.0e-12) {
    return (point.x - start.x) * (point.x - start.x) +
           (point.y - start.y) * (point.y - start.y);
  }
  const double ratio = std::clamp(
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0);
  const double near_x = start.x + ratio * dx;
  const double near_y = start.y + ratio * dy;
  return (point.x - near_x) * (point.x - near_x) +
         (point.y - near_y) * (point.y - near_y);
}

bool insideOrOnPolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  bool inside = false;
  for (size_t index = 0, previous = polygon.size() - 1; index < polygon.size();
    previous = index++)
  {
    const auto & start = polygon[previous];
    const auto & end = polygon[index];
    if (pointSegmentDistanceSquared(point, start, end) <= 1.0e-10) {
      return true;
    }
    const bool crosses = (start.y > point.y) != (end.y > point.y);
    if (crosses && point.x < start.x + (point.y - start.y) * (end.x - start.x) /
      (end.y - start.y))
    {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<Point2D> convexHull(std::vector<Point2D> points)
{
  std::sort(
    points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
      return a.x == b.x ? a.y < b.y : a.x < b.x;
    });
  points.erase(
    std::unique(
      points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
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
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0)
    {
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
    mesh_projection_margin_m_ = declare_parameter<double>("mesh_projection_margin_m", 0.03);
    planner_footprint_margin_m_ = declare_parameter<double>("planner_footprint_margin_m", 0.15);
    mesh_filter_frequency_ = declare_parameter<double>("mesh_filter_frequency", 10.0);
    mesh_projection_frequency_ = declare_parameter<double>("mesh_projection_frequency", 10.0);
    mesh_reprojection_joint_delta_rad_ = declare_parameter<double>(
      "mesh_reprojection_joint_delta_rad", 0.01);
    circle_samples_ = declare_parameter<int>("circle_samples", 12);
    base_footprint_ = declare_parameter<std::vector<double>>(
      "base_footprint_points", std::vector<double>{0.341, 0.266, 0.341, -0.266, -0.341, -0.266,
        -0.341, 0.266});
    fallback_footprint_ = declare_parameter<std::vector<double>>(
      "fallback_footprint_points", std::vector<double>{
        1.05, 1.30, 1.05, -1.30, -1.05, -1.30, -1.05, 1.30});
    if (state_timeout_sec_ <= 0.0 || update_frequency_ <= 0.0 || link_radius_m_ <= 0.0 ||
      tool_radius_m_ <= 0.0 || projection_margin_m_ < 0.0 || mesh_projection_margin_m_ < 0.0 ||
      planner_footprint_margin_m_ < 0.0 ||
      mesh_filter_frequency_ <= 0.0 || mesh_projection_frequency_ <= 0.0 ||
      mesh_reprojection_joint_delta_rad_ < 0.0 ||
      circle_samples_ < 6 ||
      base_footprint_.size() < 6 || base_footprint_.size() % 2 != 0 ||
      fallback_footprint_.size() < 6 || fallback_footprint_.size() % 2 != 0)
    {
      throw std::invalid_argument("invalid arm footprint parameters");
    }
    loadCollisionMeshes();

    state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions state_subscription_options;
    state_subscription_options.callback_group = state_callback_group_;
    left_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/left_arm/joint_states", rclcpp::QoS(20),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (!validJointState(*message, "left_")) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        left_state_received_ = std::chrono::steady_clock::now();
        left_state_valid_ = true;
        for (size_t index = 0; index < message->name.size(); ++index) {
          if (message->name[index].rfind("left_joint", 0) == 0) {
            latest_mesh_joint_positions_[message->name[index]] = message->position[index];
          }
        }
        ++joint_state_generation_;
      }, state_subscription_options);
    right_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/right_arm/joint_states", rclcpp::QoS(20),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (!validJointState(*message, "right_")) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        right_state_received_ = std::chrono::steady_clock::now();
        right_state_valid_ = true;
        for (size_t index = 0; index < message->name.size(); ++index) {
          if (message->name[index].rfind("right_joint", 0) == 0) {
            latest_mesh_joint_positions_[message->name[index]] = message->position[index];
          }
        }
        ++joint_state_generation_;
      }, state_subscription_options);
    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/r1_arm_footprint", rclcpp::QoS(10));
    mesh_footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/r1_arm_mesh_footprint", rclcpp::QoS(10));
    mesh_components_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/r1_arm_mesh_components", rclcpp::QoS(10));
    guard_mesh_components_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/r1_arm_guard_mesh_components", rclcpp::QoS(10));
    mesh_shadow_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_neupan_mesh_shadow", rclcpp::SensorDataQoS());
    mesh_filter_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/r1_arm_mesh_filter/status", rclcpp::QoS(10));
    footprint_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mesh_projection_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_subscription_options;
    scan_subscription_options.callback_group = scan_callback_group_;
    mesh_shadow_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_neupan", rclcpp::SensorDataQoS(),
      std::bind(&R1ArmFootprintNode::meshShadowScanCallback, this, std::placeholders::_1),
      scan_subscription_options);
    local_costmap_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/local_costmap/footprint", rclcpp::QoS(10));
    global_costmap_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/global_costmap/footprint", rclcpp::QoS(10));
    planner_footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/r1_arm_footprint_planner", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/r1_arm_footprint/status", rclcpp::QoS(10));
    // The collision footprint is the real-time safety heartbeat. Mesh projection
    // is substantially more expensive and must never delay this timer.
    footprint_timer_ = rclcpp::create_wall_timer(
      std::chrono::duration<double>(1.0 / update_frequency_),
      std::bind(&R1ArmFootprintNode::footprintTick, this), footprint_callback_group_,
      get_node_base_interface().get(), get_node_timers_interface().get());
    mesh_projection_timer_ = rclcpp::create_wall_timer(
      std::chrono::duration<double>(1.0 / mesh_projection_frequency_),
      std::bind(&R1ArmFootprintNode::meshProjectionTick, this), mesh_projection_callback_group_,
      get_node_base_interface().get(), get_node_timers_interface().get());
  }

private:
  void addDisc(std::vector<Point2D> & points, const Point2D & center, const double radius) const
  {
    for (int index = 0; index < circle_samples_; ++index) {
      const double angle = 2.0 * M_PI * static_cast<double>(index) /
        static_cast<double>(circle_samples_);
      points.push_back({center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)});
    }
  }

  void publishStatus(const std::string & state, const std::string & reason, const size_t points)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    double publish_interval_sec = -1.0;
    double left_joint_age_sec = -1.0;
    double right_joint_age_sec = -1.0;
    double mesh_age_sec = -1.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_interval_sec = last_publish_interval_sec_;
      if (left_state_received_ != SteadyTime{}) {
        left_joint_age_sec =
          std::chrono::duration<double>(steady_now - left_state_received_).count();
      }
      if (right_state_received_ != SteadyTime{}) {
        right_joint_age_sec =
          std::chrono::duration<double>(steady_now - right_state_received_).count();
      }
      if (last_mesh_projection_ != SteadyTime{}) {
        mesh_age_sec = std::chrono::duration<double>(steady_now - last_mesh_projection_).count();
      }
    }
    std_msgs::msg::String message;
    message.data = "{\"schema\":\"r1_arm_footprint_status_v1\",\"state\":\"" + state +
      "\",\"reason\":\"" + reason + "\",\"points\":" + std::to_string(points) +
      ",\"target_update_hz\":" + std::to_string(update_frequency_) +
      ",\"publish_interval_sec\":" + std::to_string(publish_interval_sec) +
      ",\"left_joint_age_sec\":" + std::to_string(left_joint_age_sec) +
      ",\"right_joint_age_sec\":" + std::to_string(right_joint_age_sec) +
      ",\"mesh_age_sec\":" + std::to_string(mesh_age_sec) + "}";
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
    // The planning footprint carries the same lateral margin the NeuPAN
    // pipeline applies to arm-adjacent obstacles, so the global planner's
    // feasibility model matches the local optimizer's. The safety heartbeat
    // on /r1_arm_footprint stays unpadded for the collision guard.
    geometry_msgs::msg::Polygon planner_polygon;
    if (planner_footprint_margin_m_ > 0.0) {
      std::vector<Point2D> expanded;
      for (const auto & point : points) {
        addDisc(expanded, point, planner_footprint_margin_m_);
      }
      auto hull = convexHull(std::move(expanded));
      for (const auto & point : hull) {
        geometry_msgs::msg::Point32 output;
        output.x = static_cast<float>(point.x);
        output.y = static_cast<float>(point.y);
        output.z = 0.0F;
        planner_polygon.points.push_back(output);
      }
    } else {
      planner_polygon = polygon;
    }
    geometry_msgs::msg::PolygonStamped planner_stamped;
    planner_stamped.header.stamp = stamped.header.stamp;
    planner_stamped.header.frame_id = base_frame_;
    planner_stamped.polygon = planner_polygon;
    planner_footprint_pub_->publish(planner_stamped);
    local_costmap_pub_->publish(planner_polygon);
    global_costmap_pub_->publish(planner_polygon);
    const auto published_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_footprint_published_ != SteadyTime{}) {
      last_publish_interval_sec_ = std::chrono::duration<double>(
        published_at - last_footprint_published_).count();
    }
    last_footprint_published_ = published_at;
  }

  void publishMeshShadowPolygon(const std::vector<Point2D> & points)
  {
    geometry_msgs::msg::PolygonStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = base_frame_;
    for (const auto & point : points) {
      geometry_msgs::msg::Point32 output;
      output.x = static_cast<float>(point.x);
      output.y = static_cast<float>(point.y);
      output.z = 0.0F;
      stamped.polygon.points.push_back(output);
    }
    mesh_footprint_pub_->publish(stamped);
  }

  void publishMeshComponents(const std::vector<std::vector<Point2D>> & components)
  {
    // Float64MultiArray is a compact, dependency-free transport for variable
    // sized polygons: [vertex_count, x0, y0, ...] for each component.
    std_msgs::msg::Float64MultiArray message;
    std_msgs::msg::MultiArrayDimension dimension;
    dimension.label = "r1_arm_mesh_components_v1";
    dimension.size = components.size();
    for (const auto & component : components) {
      dimension.stride += 1U + component.size() * 2U;
    }
    message.layout.dim.push_back(dimension);
    message.data.reserve(dimension.stride);
    for (const auto & component : components) {
      message.data.push_back(static_cast<double>(component.size()));
      for (const auto & point : component) {
        message.data.push_back(point.x);
        message.data.push_back(point.y);
      }
    }
    mesh_components_pub_->publish(message);
  }

  void publishGuardMeshComponents(
    const std::vector<std::vector<Point2D>> & raw,
    const std::vector<std::vector<Point2D>> & expanded)
  {
    std_msgs::msg::Float64MultiArray message;
    std_msgs::msg::MultiArrayDimension dimension;
    dimension.label = "r1_arm_guard_mesh_components_v1";
    dimension.size = raw.size();
    message.data.push_back(static_cast<double>(raw.size()));
    for (const auto * group : {&raw, &expanded}) {
      for (const auto & polygon : *group) {
        message.data.push_back(static_cast<double>(polygon.size()));
        for (const auto & point : polygon) {
          message.data.push_back(point.x);
          message.data.push_back(point.y);
        }
      }
    }
    dimension.stride = message.data.size();
    message.layout.dim.push_back(dimension);
    guard_mesh_components_pub_->publish(message);
  }

  void addMeshPart(const std::string & frame, const std::string & resource, const double scale)
  {
    std::unique_ptr<shapes::Mesh> mesh(
      shapes::createMeshFromResource(resource, Eigen::Vector3d(scale, scale, scale)));
    if (!mesh || mesh->vertex_count == 0U) {
      throw std::runtime_error("mesh_load_failed:" + resource);
    }
    MeshPart part;
    part.frame = frame;
    part.resource = resource;
    part.scale = scale;
    part.vertices.reserve(mesh->vertex_count);
    for (unsigned int index = 0; index < mesh->vertex_count; ++index) {
      part.vertices.push_back(
        {
          mesh->vertices[index * 3U], mesh->vertices[index * 3U + 1U],
          mesh->vertices[index * 3U + 2U]});
    }
    mesh_parts_.push_back(std::move(part));
  }

  void loadCollisionMeshes()
  {
    try {
      const auto add_arm = [this](const std::string & prefix, const std::string & model) {
          const std::string root = "package://rokae_description/meshes/" + model + "/visual/";
          addMeshPart(prefix + "base", root + model + "_base.stl", 0.001);
          for (int index = 1; index <= 7; ++index) {
            addMeshPart(
              prefix + "link" + std::to_string(index),
              root + model + "_link" + std::to_string(index) + ".stl", 0.001);
          }
        };
      add_arm("left_", "AR5-5_08L-W4C1C5");
      add_arm("right_", "AR5-5_08R-W4C1C5");
      for (const auto & prefix : {std::string("left_"), std::string("right_")}) {
        addMeshPart(prefix + "moduan_link", "package://r1_description/meshes/moduan_link.STL", 1.0);
        const std::string gripper_prefix = prefix == "left_" ? "L_" : "R_";
        const std::string gripper_root =
          "package://rpp_components_description/meshes/gripper/agibot/omniPicker/";
        addMeshPart(gripper_prefix + "omniPicker_base_link", gripper_root + "base_link.STL", 1.0);
        for (const auto & link : {std::string("narrow1_Link"), std::string("narrow2_Link"),
            std::string("narrow3_Link"), std::string("narrow_loop_Link"),
            std::string("wide1_Link"), std::string("wide2_Link"),
            std::string("wide3_Link"), std::string("wide_loop_Link")})
        {
          addMeshPart(
            gripper_prefix + "omniPicker_hand_" + link,
            gripper_root + link + ".STL", 1.0);
        }
      }
      mesh_model_ready_ = true;
      RCLCPP_INFO(
        get_logger(), "loaded %zu URDF collision-mesh parts for arm projection",
        mesh_parts_.size());
    } catch (const std::exception & exception) {
      mesh_parts_.clear();
      mesh_model_ready_ = false;
      RCLCPP_ERROR(
        get_logger(), "URDF collision-mesh projection unavailable: %s",
        exception.what());
    }
  }

  bool projectCollisionMeshes(
    std::vector<std::vector<Point2D>> & component_polygons,
    std::vector<std::vector<Point2D>> & raw_polygons, std::vector<Point2D> & output)
  {
    if (!mesh_model_ready_) {
      return false;
    }
    std::vector<Point2D> all_points;
    for (const auto & part : mesh_parts_) {
      std::vector<Point2D> points;
      try {
        const auto transform = tf_buffer_.lookupTransform(
          base_frame_, part.frame,
          tf2::TimePointZero);
        const tf2::Quaternion rotation_q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
        const tf2::Matrix3x3 rotation(rotation_q);
        const tf2::Vector3 translation(
          transform.transform.translation.x, transform.transform.translation.y,
          transform.transform.translation.z);
        for (const auto & vertex : part.vertices) {
          const auto projected = rotation *
            tf2::Vector3(vertex.x, vertex.y, vertex.z) + translation;
          points.push_back({projected.x(), projected.y()});
        }
      } catch (const tf2::TransformException &) {
        return false;
      }
      auto hull = convexHull(std::move(points));
      if (hull.size() < 3) {
        return false;
      }
      raw_polygons.push_back(hull);
      std::vector<Point2D> expanded;
      for (const auto & point : hull) {
        addDisc(expanded, point, mesh_projection_margin_m_);
      }
      hull = convexHull(std::move(expanded));
      all_points.insert(all_points.end(), hull.begin(), hull.end());
      component_polygons.push_back(std::move(hull));
    }
    output = convexHull(std::move(all_points));
    if (output.size() < 3) {
      return false;
    }
    return output.size() >= 3;
  }

  void meshShadowScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (last_mesh_filter_started_ != SteadyTime{} &&
      std::chrono::duration<double>(steady_now - last_mesh_filter_started_).count() <
      1.0 / mesh_filter_frequency_)
    {
      return;
    }
    last_mesh_filter_started_ = steady_now;
    std::vector<std::vector<Point2D>> polygons;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      polygons = mesh_self_filter_polygons_;
    }
    if (message->header.frame_id != base_frame_ || polygons.empty()) {
      return;
    }
    sensor_msgs::msg::LaserScan filtered = *message;
    size_t removed = 0;
    double angle = message->angle_min;
    for (auto & range : filtered.ranges) {
      if (std::isfinite(range) && range >= message->range_min && range <= message->range_max) {
        const Point2D point{range * std::cos(angle), range * std::sin(angle)};
        if (std::any_of(
            polygons.begin(), polygons.end(), [&point](const auto & polygon) {
              return insideOrOnPolygon(point, polygon);
            }))
        {
          range = std::numeric_limits<float>::infinity();
          ++removed;
        }
      }
      angle += message->angle_increment;
    }
    mesh_shadow_scan_pub_->publish(filtered);
    std_msgs::msg::String status;
    status.data =
      "{\"schema\":\"r1_arm_mesh_filter_status_v1\",\"state\":\"READY\",\"input_points\":" +
      std::to_string(message->ranges.size()) + ",\"self_filtered_points\":" +
      std::to_string(removed) + ",\"component_polygons\":" + std::to_string(polygons.size()) + "}";
    mesh_filter_status_pub_->publish(status);
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

  void meshProjectionTick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    bool states_ready = false;
    bool geometry_changed = true;
    std::unordered_map<std::string, double> current_joint_positions;
    std::vector<std::vector<Point2D>> cached_components;
    std::vector<std::vector<Point2D>> cached_raw_components;
    std::vector<Point2D> cached_shadow;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      states_ready = left_state_valid_ && right_state_valid_ &&
        std::chrono::duration<double>(steady_now - left_state_received_).count() <=
        state_timeout_sec_ &&
        std::chrono::duration<double>(steady_now - right_state_received_).count() <=
        state_timeout_sec_;
      if (states_ready) {
        current_joint_positions = latest_mesh_joint_positions_;
        geometry_changed = current_joint_positions.size() != mesh_projected_joint_positions_.size();
        if (!geometry_changed) {
          for (const auto & [name, position] : current_joint_positions) {
            const auto previous = mesh_projected_joint_positions_.find(name);
            if (previous == mesh_projected_joint_positions_.end() ||
              std::abs(position - previous->second) >= mesh_reprojection_joint_delta_rad_)
            {
              geometry_changed = true;
              break;
            }
          }
        }
        if (!geometry_changed) {
          cached_components = mesh_cached_components_;
          cached_raw_components = mesh_cached_raw_components_;
          cached_shadow = mesh_cached_shadow_;
        }
      }
    }
    if (!states_ready) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!mesh_self_filter_polygons_.empty()) {
        mesh_self_filter_polygons_.clear();
        mesh_cached_components_.clear();
        mesh_cached_raw_components_.clear();
        mesh_cached_shadow_.clear();
        mesh_projected_joint_positions_.clear();
        publishMeshComponents({});
        publishGuardMeshComponents({}, {});
      }
      return;
    }

    // The exact mesh is expensive to project but joint messages continue while
    // the arms are stationary. Re-publish the validated cached geometry so
    // downstream freshness checks remain live without reprocessing meshes.
    if (!geometry_changed && !cached_components.empty() && !cached_shadow.empty()) {
      publishMeshShadowPolygon(cached_shadow);
      publishMeshComponents(cached_components);
      publishGuardMeshComponents(cached_raw_components, cached_components);
      std::lock_guard<std::mutex> lock(mutex_);
      last_mesh_projection_ = std::chrono::steady_clock::now();
      return;
    }

    std::vector<std::vector<Point2D>> mesh_components;
    std::vector<std::vector<Point2D>> raw_components;
    std::vector<Point2D> mesh_shadow;
    if (!projectCollisionMeshes(mesh_components, raw_components, mesh_shadow)) {
      return;
    }
    publishMeshShadowPolygon(mesh_shadow);
    publishMeshComponents(mesh_components);
    publishGuardMeshComponents(raw_components, mesh_components);
    std::lock_guard<std::mutex> lock(mutex_);
    mesh_self_filter_polygons_ = mesh_components;
    mesh_cached_components_ = std::move(mesh_components);
    mesh_cached_raw_components_ = std::move(raw_components);
    mesh_cached_shadow_ = std::move(mesh_shadow);
    mesh_projected_joint_positions_ = std::move(current_joint_positions);
    last_mesh_projection_ = std::chrono::steady_clock::now();
  }

  void footprintTick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    bool states_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      states_ready = left_state_valid_ && right_state_valid_ &&
        std::chrono::duration<double>(steady_now - left_state_received_).count() <=
        state_timeout_sec_ &&
        std::chrono::duration<double>(steady_now - right_state_received_).count() <=
        state_timeout_sec_;
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
        addDisc(
          points, {transform.transform.translation.x, transform.transform.translation.y},
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
  double mesh_projection_margin_m_{0.03};
  double planner_footprint_margin_m_{0.15};
  double mesh_filter_frequency_{10.0};
  double mesh_projection_frequency_{10.0};
  double mesh_reprojection_joint_delta_rad_{0.01};
  int circle_samples_{12};
  std::vector<double> base_footprint_;
  std::vector<double> fallback_footprint_;
  bool mesh_model_ready_{false};
  std::vector<MeshPart> mesh_parts_;
  std::vector<std::vector<Point2D>> mesh_self_filter_polygons_;
  std::vector<std::vector<Point2D>> mesh_cached_components_;
  std::vector<std::vector<Point2D>> mesh_cached_raw_components_;
  std::vector<Point2D> mesh_cached_shadow_;
  std::unordered_map<std::string, double> latest_mesh_joint_positions_;
  std::unordered_map<std::string, double> mesh_projected_joint_positions_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  SteadyTime last_mesh_filter_started_{};
  std::mutex mutex_;
  SteadyTime left_state_received_{};
  SteadyTime right_state_received_{};
  SteadyTime last_footprint_published_{};
  SteadyTime last_mesh_projection_{};
  double last_publish_interval_sec_{-1.0};
  std::uint64_t joint_state_generation_{0};
  std::uint64_t mesh_projected_state_generation_{0};
  bool left_state_valid_{false};
  bool right_state_valid_{false};
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr footprint_callback_group_;
  rclcpp::CallbackGroup::SharedPtr mesh_projection_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr mesh_footprint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mesh_components_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr guard_mesh_components_pub_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mesh_shadow_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mesh_shadow_scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mesh_filter_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr planner_footprint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr footprint_timer_;
  rclcpp::TimerBase::SharedPtr mesh_projection_timer_;
};
}  // namespace navigation_neupan_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_neupan_bridge::R1ArmFootprintNode>();
  // State callbacks must remain serviceable while a mesh projection and a
  // scan filter are each running.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
