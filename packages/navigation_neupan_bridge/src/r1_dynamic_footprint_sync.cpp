// Synchronizes the live dual-arm projection into both Nav2 costmaps.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "navigation_neupan_bridge/polygon_boundary_distance.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation_neupan_bridge
{
namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

bool finitePolygon(const std::vector<Point2D> & polygon)
{
  return polygon.size() >= 3 && std::all_of(
    polygon.begin(), polygon.end(), [](const Point2D & point) {
      return std::isfinite(point.x) && std::isfinite(point.y);
    });
}

std::string footprintParameter(const std::vector<Point2D> & polygon)
{
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output << std::setprecision(4) << "[";
  for (size_t index = 0; index < polygon.size(); ++index) {
    if (index != 0) {
      output << ", ";
    }
    output << "[" << polygon[index].x << ", " << polygon[index].y << "]";
  }
  output << "]";
  return output.str();
}
}  // namespace

class DynamicFootprintSync : public rclcpp::Node
{
public:
  DynamicFootprintSync()
  : Node("r1_dynamic_footprint_sync")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/r1_arm_footprint");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    global_costmap_node_ = declare_parameter<std::string>(
      "global_costmap_node", "/global_costmap/global_costmap");
    local_costmap_node_ = declare_parameter<std::string>(
      "local_costmap_node", "/local_costmap/local_costmap");
    update_threshold_m_ = declare_parameter<double>("update_threshold_m", 0.02);
    if (input_topic_.empty() || base_frame_.empty() || global_costmap_node_.empty() ||
      local_costmap_node_.empty() || update_threshold_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid dynamic footprint synchronization parameters");
    }

    global_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, global_costmap_node_);
    local_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, local_costmap_node_);
    footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      input_topic_, rclcpp::QoS(20),
      std::bind(&DynamicFootprintSync::footprintCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/r1_dynamic_footprint_sync/status", rclcpp::QoS(20));
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&DynamicFootprintSync::tick, this));
  }

private:
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
  {
    if (message->header.frame_id != base_frame_) {
      publish("FAULT", "footprint_frame_mismatch");
      return;
    }
    std::vector<Point2D> polygon;
    polygon.reserve(message->polygon.points.size());
    for (const auto & point : message->polygon.points) {
      polygon.push_back({point.x, point.y});
    }
    if (!finitePolygon(polygon)) {
      publish("FAULT", "footprint_invalid");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!desired_.empty() && maxBoundaryDifference(desired_, polygon) < update_threshold_m_) {
      received_ = std::chrono::steady_clock::now();
      return;
    }
    desired_ = std::move(polygon);
    desired_parameter_ = footprintParameter(desired_);
    received_ = std::chrono::steady_clock::now();
    ++generation_;
  }

  void requestUpdate(const bool global, const uint64_t generation, const std::string & parameter)
  {
    auto client = global ? global_client_ : local_client_;
    if (!client->service_is_ready()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (global) {
        global_inflight_ = true;
      } else {
        local_inflight_ = true;
      }
    }
    client->set_parameters_atomically({rclcpp::Parameter("footprint", parameter)},
      [this, global, generation](
        std::shared_future<rcl_interfaces::msg::SetParametersResult> result_future) {
        const auto result = result_future.get();
        std::lock_guard<std::mutex> lock(mutex_);
        if (global) {
          global_inflight_ = false;
          global_ack_generation_ = result.successful ? generation : 0;
        } else {
          local_inflight_ = false;
          local_ack_generation_ = result.successful ? generation : 0;
        }
        if (!result.successful) {
          last_failure_ = result.reason.empty() ? "costmap_parameter_rejected" : result.reason;
        }
      });
  }

  void tick()
  {
    std::string parameter;
    uint64_t generation = 0;
    bool request_global = false;
    bool request_local = false;
    bool have_desired = false;
    bool synchronized = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_desired = !desired_.empty();
      if (have_desired) {
        parameter = desired_parameter_;
        generation = generation_;
        request_global = !global_inflight_ && global_ack_generation_ != generation;
        request_local = !local_inflight_ && local_ack_generation_ != generation;
        synchronized = global_ack_generation_ == generation && local_ack_generation_ == generation;
      }
    }
    if (!have_desired) {
      publish("WAIT", "waiting_dynamic_footprint");
      return;
    }
    if (synchronized) {
      publish("READY", "costmaps_synchronized");
      return;
    }
    if (request_global) {
      requestUpdate(true, generation, parameter);
    }
    if (request_local) {
      requestUpdate(false, generation, parameter);
    }
    publish("WAIT", "waiting_costmap_parameter_acknowledgement");
  }

  void publish(const std::string & state, const std::string & reason)
  {
    size_t points = 0;
    uint64_t generation = 0;
    bool global_ready = false;
    bool local_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      points = desired_.size();
      generation = generation_;
      global_ready = global_ack_generation_ == generation && generation != 0;
      local_ready = local_ack_generation_ == generation && generation != 0;
    }
    std_msgs::msg::String message;
    message.data = "{\"schema\":\"r1_dynamic_footprint_sync_status_v1\",\"state\":\"" + state +
      "\",\"reason\":\"" + reason + "\",\"generation\":" + std::to_string(generation) +
      ",\"points\":" + std::to_string(points) + ",\"global_ack\":" +
      (global_ready ? "true" : "false") + ",\"local_ack\":" +
      (local_ready ? "true" : "false") + "}";
    status_pub_->publish(message);
  }

  std::string input_topic_;
  std::string base_frame_;
  std::string global_costmap_node_;
  std::string local_costmap_node_;
  double update_threshold_m_{0.02};
  std::shared_ptr<rclcpp::AsyncParametersClient> global_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> local_client_;
  std::mutex mutex_;
  std::vector<Point2D> desired_;
  std::string desired_parameter_;
  SteadyTime received_{};
  uint64_t generation_{0};
  uint64_t global_ack_generation_{0};
  uint64_t local_ack_generation_{0};
  bool global_inflight_{false};
  bool local_inflight_{false};
  std::string last_failure_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation_neupan_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_neupan_bridge::DynamicFootprintSync>());
  rclcpp::shutdown();
  return 0;
}
