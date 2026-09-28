#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

struct ScanSnapshot
{
  sensor_msgs::msg::LaserScan scan;
  std::chrono::steady_clock::time_point received;
  bool available{false};
};

struct Point2D
{
  double x;
  double y;
};

double pointSegmentDistanceSquared(const Point2D & point, const Point2D & start, const Point2D & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    const double px = point.x - start.x;
    const double py = point.y - start.y;
    return px * px + py * py;
  }
  const double factor = std::clamp(
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0);
  const double px = point.x - (start.x + factor * dx);
  const double py = point.y - (start.y + factor * dy);
  return px * px + py * py;
}

}  // namespace

class ScanFusionNode : public rclcpp::Node
{
public:
  ScanFusionNode()
  : Node("r1_scan_fusion"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    livox_topic_ = declare_parameter<std::string>("livox_scan_topic", "/scan_livox");
    rear_topic_ = declare_parameter<std::string>("rear_scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan_neupan");
    status_topic_ = declare_parameter<std::string>("status_topic", "/scan_neupan_fusion/status");
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 360.0);
    range_min_ = declare_parameter<double>("range_min", 0.15);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.10);
    max_scan_age_sec_ = declare_parameter<double>("max_scan_age_sec", 0.20);
    max_pair_delta_sec_ = declare_parameter<double>("max_pair_delta_sec", 0.08);
    pair_wait_sec_ = declare_parameter<double>("pair_wait_sec", 0.08);
    livox_history_size_ = declare_parameter<int>("livox_history_size", 16);
    rear_history_size_ = declare_parameter<int>("rear_history_size", 16);
    self_filter_margin_m_ = declare_parameter<double>("self_filter_margin_m", 0.02);
    const auto polygon = declare_parameter<std::vector<double>>(
      "self_filter_polygon_xy", std::vector<double>{});

    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_ || range_min_ <= 0.0 ||
      range_max_ <= range_min_ || max_scan_age_sec_ <= 0.0 || max_pair_delta_sec_ <= 0.0 ||
      pair_wait_sec_ <= 0.0 || pair_wait_sec_ >= max_scan_age_sec_ ||
      livox_history_size_ <= 0 || rear_history_size_ <= 0 || self_filter_margin_m_ < 0.0)
    {
      throw std::runtime_error("invalid scan fusion parameters");
    }
    if (!polygon.empty() && (polygon.size() < 6U || polygon.size() % 2U != 0U)) {
      throw std::runtime_error("self_filter_polygon_xy must contain at least three x/y pairs");
    }
    for (size_t index = 0; index + 1U < polygon.size(); index += 2U) {
      self_filter_polygon_.push_back({polygon[index], polygon[index + 1U]});
    }
    output_size_ = static_cast<size_t>(std::llround(
      (angle_max_ - angle_min_) / angle_increment_)) + 1U;
    if (output_size_ < 2U) {
      throw std::runtime_error("scan fusion output has fewer than two bins");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    livox_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      livox_topic_, sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) { receiveScan(message, true); });
    rear_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      rear_topic_, sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) { receiveScan(message, false); });
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, sensor_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    pairing_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() { tryPublishFusion(); });

    RCLCPP_INFO(
      get_logger(), "fusing %s and %s into %s in %s", livox_topic_.c_str(), rear_topic_.c_str(),
      output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void receiveScan(const sensor_msgs::msg::LaserScan::SharedPtr & message, const bool livox)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ScanSnapshot snapshot;
      snapshot.scan = *message;
      snapshot.received = std::chrono::steady_clock::now();
      snapshot.available = true;
      if (livox) {
        last_livox_received_ = snapshot.received;
        livox_seen_ = true;
      } else {
        last_rear_received_ = snapshot.received;
        rear_seen_ = true;
      }
      auto & history = livox ? livox_history_ : rear_history_;
      history.push_back(std::move(snapshot));
      const auto history_size = static_cast<size_t>(
        livox ? livox_history_size_ : rear_history_size_);
      while (history.size() > history_size) {
        history.pop_front();
      }
    }
  }

  void tryPublishFusion()
  {
    ScanSnapshot livox;
    ScanSnapshot rear;
    std::string status;
    bool pair_ready = false;
    const auto now_steady = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!rear_history_.empty() &&
        std::chrono::duration<double>(now_steady - rear_history_.front().received).count() >
        max_scan_age_sec_)
      {
        rear_history_.pop_front();
      }
      while (!livox_history_.empty() &&
        std::chrono::duration<double>(now_steady - livox_history_.front().received).count() >
        max_scan_age_sec_)
      {
        livox_history_.pop_front();
        status = "source_scan_stale";
      }

      if (livox_history_.empty()) {
        if (!livox_seen_) {
          status = rear_seen_ ? "waiting_for_livox_scan" : "waiting_for_both_scans";
        } else if (std::chrono::duration<double>(now_steady - last_livox_received_).count() >
          max_scan_age_sec_)
        {
          status = "livox_scan_stale";
        } else if (rear_seen_ &&
          std::chrono::duration<double>(now_steady - last_rear_received_).count() >
          max_scan_age_sec_)
        {
          status = "source_scan_stale";
        }
      } else {
        livox = livox_history_.front();
        const auto livox_age = std::chrono::duration<double>(now_steady - livox.received).count();
        const rclcpp::Time livox_stamp(livox.scan.header.stamp);
        if (livox_stamp.nanoseconds() == 0) {
          livox_history_.pop_front();
          status = "source_scan_has_zero_stamp";
        } else if (rear_history_.empty()) {
          status = livox_age < pair_wait_sec_ ? "waiting_for_rear_pair" : "source_scans_unsynchronised";
          if (livox_age >= pair_wait_sec_) {
            livox_history_.pop_front();
          }
        } else {
          const auto nearest = std::min_element(
            rear_history_.begin(), rear_history_.end(),
            [&livox_stamp](const ScanSnapshot & first, const ScanSnapshot & second) {
              return std::abs((rclcpp::Time(first.scan.header.stamp) - livox_stamp).seconds()) <
                     std::abs((rclcpp::Time(second.scan.header.stamp) - livox_stamp).seconds());
            });
          rear = *nearest;
          const rclcpp::Time rear_stamp(rear.scan.header.stamp);
          const double pair_delta = rear_stamp.nanoseconds() == 0 ?
            std::numeric_limits<double>::infinity() :
            std::abs((livox_stamp - rear_stamp).seconds());
          if (pair_delta <= max_pair_delta_sec_) {
            livox_history_.pop_front();
            rear_history_.erase(nearest);
            pair_ready = true;
          } else if (livox_age < pair_wait_sec_) {
            // A later rear scan can be closer than the currently available one.
            status = "waiting_for_rear_pair";
          } else {
            livox_history_.pop_front();
            status = rear_stamp.nanoseconds() == 0 ?
              "source_scan_has_zero_stamp" : "source_scans_unsynchronised";
          }
        }
      }
    }
    if (!pair_ready) {
      if (!status.empty()) {
        publishStatus(status);
      }
      return;
    }
    const rclcpp::Time livox_stamp(livox.scan.header.stamp);
    const rclcpp::Time rear_stamp(rear.scan.header.stamp);

    std::vector<float> ranges(output_size_, std::numeric_limits<float>::infinity());
    std::string transform_error;
    if (!mergeScan(livox.scan, ranges, transform_error) || !mergeScan(rear.scan, ranges, transform_error)) {
      publishStatus("transform_unavailable:" + transform_error);
      return;
    }

    sensor_msgs::msg::LaserScan output;
    // The older stamp keeps odometry compensation conservative for the latest source.
    output.header.stamp = livox_stamp <= rear_stamp ? livox.scan.header.stamp : rear.scan.header.stamp;
    output.header.frame_id = target_frame_;
    output.angle_min = static_cast<float>(angle_min_);
    output.angle_max = static_cast<float>(angle_min_ + (output_size_ - 1U) * angle_increment_);
    output.angle_increment = static_cast<float>(angle_increment_);
    output.scan_time = static_cast<float>(scan_time_);
    output.time_increment = 0.0F;
    output.range_min = static_cast<float>(range_min_);
    output.range_max = static_cast<float>(range_max_);
    output.ranges = std::move(ranges);
    output.intensities.assign(output_size_, 0.0F);
    scan_pub_->publish(output);
    publishStatus("ok");
  }

  bool mergeScan(
    const sensor_msgs::msg::LaserScan & scan, std::vector<float> & fused_ranges,
    std::string & error)
  {
    if (scan.header.frame_id.empty()) {
      error = "empty_source_frame";
      return false;
    }
    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (scan.header.frame_id != target_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          target_frame_, scan.header.frame_id, tf2::TimePointZero);
        tx = transform.transform.translation.x;
        ty = transform.transform.translation.y;
        const tf2::Quaternion rotation(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
      } catch (const tf2::TransformException & exception) {
        error = exception.what();
        return false;
      }
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (size_t index = 0; index < scan.ranges.size(); ++index) {
      const float source_range = scan.ranges[index];
      if (!std::isfinite(source_range) || source_range < scan.range_min || source_range > scan.range_max) {
        continue;
      }
      const double source_angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const double x = tx + source_range * (cos_yaw * std::cos(source_angle) - sin_yaw * std::sin(source_angle));
      const double y = ty + source_range * (sin_yaw * std::cos(source_angle) + cos_yaw * std::sin(source_angle));
      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_ || insideSelfFilter({x, y})) {
        continue;
      }
      double angle = std::atan2(y, x);
      while (angle < angle_min_) {
        angle += 2.0 * M_PI;
      }
      while (angle > angle_max_) {
        angle -= 2.0 * M_PI;
      }
      const auto bin = static_cast<long>(std::llround((angle - angle_min_) / angle_increment_));
      if (bin < 0 || static_cast<size_t>(bin) >= fused_ranges.size()) {
        continue;
      }
      fused_ranges[static_cast<size_t>(bin)] = std::min(
        fused_ranges[static_cast<size_t>(bin)], static_cast<float>(range));
    }
    return true;
  }

  bool insideSelfFilter(const Point2D & point) const
  {
    if (self_filter_polygon_.empty()) {
      return false;
    }
    bool inside = false;
    double nearest_distance_squared = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < self_filter_polygon_.size(); ++index) {
      const Point2D & start = self_filter_polygon_[index];
      const Point2D & end = self_filter_polygon_[(index + 1U) % self_filter_polygon_.size()];
      const bool crosses = (start.y > point.y) != (end.y > point.y);
      const double x_at_y = (end.x - start.x) * (point.y - start.y) /
        ((end.y != start.y) ? (end.y - start.y) : 1.0) + start.x;
      if (crosses && point.x < x_at_y) {
        inside = !inside;
      }
      nearest_distance_squared = std::min(
        nearest_distance_squared, pointSegmentDistanceSquared(point, start, end));
    }
    return inside || nearest_distance_squared <= self_filter_margin_m_ * self_filter_margin_m_;
  }

  void publishStatus(const std::string & state)
  {
    std_msgs::msg::String status;
    status.data = state;
    status_pub_->publish(status);
  }

  std::string target_frame_;
  std::string livox_topic_;
  std::string rear_topic_;
  std::string output_topic_;
  std::string status_topic_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_min_;
  double range_max_;
  double scan_time_;
  double max_scan_age_sec_;
  double max_pair_delta_sec_;
  double pair_wait_sec_;
  int livox_history_size_;
  int rear_history_size_;
  double self_filter_margin_m_;
  size_t output_size_;
  std::vector<Point2D> self_filter_polygon_;
  std::mutex mutex_;
  std::deque<ScanSnapshot> livox_history_;
  std::deque<ScanSnapshot> rear_history_;
  std::chrono::steady_clock::time_point last_livox_received_{};
  std::chrono::steady_clock::time_point last_rear_received_{};
  bool livox_seen_{false};
  bool rear_seen_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr pairing_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFusionNode>());
  rclcpp::shutdown();
  return 0;
}
