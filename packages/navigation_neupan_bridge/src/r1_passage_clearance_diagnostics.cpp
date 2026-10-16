#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "navigation_neupan_bridge/passage_clearance_state.hpp"

namespace
{
constexpr double kEpsilon = 1.0e-6;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

bool finite(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

}  // namespace

class R1PassageClearanceDiagnostics : public rclcpp::Node
{
public:
  R1PassageClearanceDiagnostics()
  : Node("r1_passage_clearance_diagnostics"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_neupan_mesh_shadow");
    path_topic_ = declare_parameter<std::string>("path_topic", "/neupan_path");
    footprint_topic_ = declare_parameter<std::string>("footprint_topic", "/r1_arm_footprint");
    clearance_m_ = declare_parameter<double>("planning_clearance_m", 0.15);
    probe_distance_m_ = declare_parameter<double>("probe_distance_m", 0.45);
    probe_half_width_m_ = declare_parameter<double>("probe_half_width_m", 0.35);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    if (clearance_m_ < 0.0 || probe_distance_m_ < 0.0 || probe_half_width_m_ <= 0.0 ||
      publish_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("passage diagnostic parameters must be nonnegative and finite");
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_ = std::move(message);
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_ = std::move(message);
      });
    footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PolygonStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        footprint_ = std::move(message);
      });
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/r1_passage_clearance/markers", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/r1_passage_clearance/status", rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&R1PassageClearanceDiagnostics::publish, this));
  }

private:
  bool transformPoint(const geometry_msgs::msg::Point & source, const std::string & source_frame,
    Point2D * target) const
  {
    if (source_frame.empty() || source_frame == base_frame_) {
      target->x = source.x;
      target->y = source.y;
      return finite(*target);
    }
    try {
      geometry_msgs::msg::Point transformed;
      tf2::doTransform(source, transformed,
        tf_buffer_.lookupTransform(base_frame_, source_frame, tf2::TimePointZero));
      target->x = transformed.x;
      target->y = transformed.y;
      return finite(*target);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool pathDirection(const nav_msgs::msg::Path & path, Point2D * direction) const
  {
    if (path.poses.size() < 2U) {
      return false;
    }
    std::vector<Point2D> points;
    points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      Point2D point;
      if (!transformPoint(pose.pose.position, path.header.frame_id, &point)) {
        return false;
      }
      points.push_back(point);
    }
    size_t closest = 0U;
    double closest_sq = std::numeric_limits<double>::infinity();
    for (size_t index = 0U; index < points.size(); ++index) {
      const double distance_sq = points[index].x * points[index].x + points[index].y * points[index].y;
      if (distance_sq < closest_sq) {
        closest_sq = distance_sq;
        closest = index;
      }
    }
    const size_t start = closest == 0U ? 0U : closest - 1U;
    const size_t end = std::min(closest + 1U, points.size() - 1U);
    direction->x = points[end].x - points[start].x;
    direction->y = points[end].y - points[start].y;
    const double norm = std::hypot(direction->x, direction->y);
    if (norm < kEpsilon) {
      return false;
    }
    direction->x /= norm;
    direction->y /= norm;
    return true;
  }

  void addLine(visualization_msgs::msg::MarkerArray * markers, int id,
    const std::vector<Point2D> & points, float red, float green, float blue, float width)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "passage_clearance";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = width;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    for (const auto & point : points) {
      geometry_msgs::msg::Point output;
      output.x = point.x;
      output.y = point.y;
      output.z = 0.05;
      marker.points.push_back(output);
    }
    markers->markers.push_back(std::move(marker));
  }

  void addText(visualization_msgs::msg::MarkerArray * markers, const std::string & text,
    float red, float green, float blue)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "passage_clearance";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = probe_distance_m_;
    marker.pose.position.z = 0.18;
    marker.scale.z = 0.16;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    marker.text = text;
    markers->markers.push_back(std::move(marker));
  }

  void publish()
  {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    nav_msgs::msg::Path::SharedPtr path;
    geometry_msgs::msg::PolygonStamped::SharedPtr footprint;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scan = scan_;
      path = path_;
      footprint = footprint_;
    }
    if (!scan || !path || !footprint) {
      publishUnavailable("waiting_for_scan_path_or_footprint");
      return;
    }
    Point2D tangent;
    if (!pathDirection(*path, &tangent)) {
      publishUnavailable("path_transform_or_direction_unavailable");
      return;
    }
    if (footprint->polygon.points.size() < 3U) {
      publishUnavailable("footprint_invalid");
      return;
    }

    const Point2D normal{-tangent.y, tangent.x};
    double footprint_left = -std::numeric_limits<double>::infinity();
    double footprint_right = std::numeric_limits<double>::infinity();
    std::vector<Point2D> footprint_points;
    footprint_points.reserve(footprint->polygon.points.size() + 1U);
    for (const auto & point : footprint->polygon.points) {
      const Point2D local{point.x, point.y};
      footprint_points.push_back(local);
      const double lateral = local.x * normal.x + local.y * normal.y;
      footprint_left = std::max(footprint_left, lateral);
      footprint_right = std::min(footprint_right, lateral);
    }
    footprint_points.push_back(footprint_points.front());
    const double footprint_width = footprint_left - footprint_right;
    const double required_width = footprint_width + 2.0 * clearance_m_;

    double left_wall = std::numeric_limits<double>::infinity();
    double right_wall = -std::numeric_limits<double>::infinity();
    const std::string scan_frame = scan->header.frame_id;
    for (size_t index = 0U; index < scan->ranges.size(); ++index) {
      const double range = scan->ranges[index];
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        continue;
      }
      const double angle = scan->angle_min + static_cast<double>(index) * scan->angle_increment;
      geometry_msgs::msg::Point scan_point;
      scan_point.x = range * std::cos(angle);
      scan_point.y = range * std::sin(angle);
      Point2D local;
      if (!transformPoint(scan_point, scan_frame, &local)) {
        publishUnavailable("scan_transform_unavailable");
        return;
      }
      const double longitudinal = local.x * tangent.x + local.y * tangent.y;
      if (std::abs(longitudinal - probe_distance_m_) > probe_half_width_m_) {
        continue;
      }
      const double lateral = local.x * normal.x + local.y * normal.y;
      if (lateral > 0.0) {
        left_wall = std::min(left_wall, lateral);
      } else {
        right_wall = std::max(right_wall, lateral);
      }
    }

    const bool have_left = std::isfinite(left_wall);
    const bool have_right = std::isfinite(right_wall);
    const bool measured = have_left && have_right;
    const double available_width = measured ? left_wall - right_wall : 0.0;
    const double left_clearance = measured ? left_wall - footprint_left : 0.0;
    const double right_clearance = measured ? footprint_right - right_wall : 0.0;
    const std::string state = navigation_neupan_bridge::passageClearanceState(
      measured, available_width, required_width,
      left_clearance, right_clearance, clearance_m_);
    const bool clear = state == "CLEAR";
    const float red = clear ? 0.1F : 1.0F;
    const float green = clear ? 1.0F : 0.15F;
    const float blue = 0.1F;

    visualization_msgs::msg::MarkerArray markers;
    addLine(&markers, 0, footprint_points, 1.0F, 0.85F, 0.0F, 0.035F);
    if (measured) {
      const Point2D center{tangent.x * probe_distance_m_, tangent.y * probe_distance_m_};
      addLine(&markers, 1, {
          {center.x + normal.x * right_wall, center.y + normal.y * right_wall},
          {center.x + normal.x * left_wall, center.y + normal.y * left_wall},
        }, red, green, blue, 0.045F);
    }
    addLine(&markers, 2, {{0.0, 0.0}, {tangent.x * 0.6, tangent.y * 0.6}}, 0.2F, 0.7F, 1.0F, 0.025F);

    std::ostringstream label;
    label << std::fixed << std::setprecision(2) << state << "  required=" << required_width << "m";
    if (measured) {
      label << "  L=" << left_clearance << "m R=" << right_clearance << "m";
    }
    addText(&markers, label.str(), red, green, blue);
    marker_pub_->publish(markers);

    std_msgs::msg::String status;
    std::ostringstream json;
    json << std::fixed << std::setprecision(4)
         << "{\"schema\":\"r1_passage_clearance_v1\",\"state\":\"" << state
         << "\",\"footprint_width_m\":" << footprint_width
         << ",\"planning_clearance_m\":" << clearance_m_
         << ",\"required_width_m\":" << required_width
         << ",\"available_width_m\":";
    if (measured) {
      json << available_width;
    } else {
      json << "null";
    }
    json << ",\"left_clearance_m\":";
    if (measured) {
      json << left_clearance;
    } else {
      json << "null";
    }
    json << ",\"right_clearance_m\":";
    if (measured) {
      json << right_clearance;
    } else {
      json << "null";
    }
    json << ",\"probe_distance_m\":" << probe_distance_m_
         << ",\"probe_half_width_m\":" << probe_half_width_m_ << "}";
    status.data = json.str();
    status_pub_->publish(status);
  }

  void publishUnavailable(const std::string & reason)
  {
    std_msgs::msg::String status;
    status.data = "{\"schema\":\"r1_passage_clearance_v1\",\"state\":\"WAIT\",\"reason\":\"" + reason + "\"}";
    status_pub_->publish(status);
  }

  std::string base_frame_;
  std::string scan_topic_;
  std::string path_topic_;
  std::string footprint_topic_;
  double clearance_m_{0.15};
  double probe_distance_m_{0.45};
  double probe_half_width_m_{0.12};
  double publish_rate_hz_{5.0};
  std::mutex mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  nav_msgs::msg::Path::SharedPtr path_;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R1PassageClearanceDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
