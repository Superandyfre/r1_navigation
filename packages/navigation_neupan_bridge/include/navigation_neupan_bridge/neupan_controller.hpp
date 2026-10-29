#ifndef NAVIGATION_NEUPAN_BRIDGE__NEUPAN_CONTROLLER_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__NEUPAN_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "r1_neupan_runtime/execution_observer.hpp"
#include "r1_neupan_runtime/controller_execution.hpp"
#include "r1_neupan_runtime/stopped_odometry.hpp"
#include "navigation_neupan_bridge/route_update_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace navigation_neupan_bridge
{

class NeuPANController : public nav2_core::Controller, public r1_neupan_runtime::ExecutionObserver
{
public:
  NeuPANController() = default;
  ~NeuPANController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void beginExecution(const std::string & authority, const std::string & attempt) override;
  void checkExecution(const std::string & authority, const std::string & attempt) override;
  void endExecution(const std::string & authority, const std::string & attempt,
    const std::string & reason) noexcept override;

private:
  void runtimeTick();
  nlohmann::json executionContextLocked(const char * state);
  void publishExecutionContextLocked(const char * state);
  void endExecutionLocked(const std::string & reason);
  std::unique_ptr<r1_neupan_runtime::ControllerExecution> execution_;
  std::map<std::string, rclcpp::ParameterValue> configured_parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  std::string worker_health_topic_{"/neupan_worker_health"};
  std::string execution_context_topic_{"/neupan_execution_context"};
  std::string motion_grant_topic_{"/neupan_motion_grant"};
  std::string motion_arm_service_{"/neupan_controller/arm"};
  std::string motion_retry_service_{"/neupan_controller/can_retry"};
  std::string odom_topic_{"/odom"};
  std::int64_t context_sequence_{0};
  r1_neupan_runtime::StoppedOdometry stopped_odometry_;
  std::string arm_odom_frame_{"odom"}, arm_base_frame_{"base_link"};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_context_pub_, motion_grant_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr worker_health_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_, retry_service_;
  rclcpp::TimerBase::SharedPtr runtime_timer_;
  struct Envelope
  {
    double linear_x{0.0};
    double angular_z{0.0};
    double planner_result_monotonic{0.0};
    std::int64_t worker_generation{0};
    int path_version{0};
    int goal_version{0};
    bool valid{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct GateStatus
  {
    double input_linear_x{0.0};
    double input_angular_z{0.0};
    double output_linear_x{0.0};
    double output_angular_z{0.0};
    double planner_result_monotonic{0.0};
    int path_version{-1};
    int goal_version{-1};
    bool emergency_stop{true};
    bool failure_latched{false};
    bool valid{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct GuardStatus
  {
    std::string state;
    std::string reason;
    bool valid{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct LocalizationHealth
  {
    std::string state;
    std::string reason;
    bool valid{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  void envelopeCallback(const std_msgs::msg::String::SharedPtr message);
  void gateCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message);
  void gateStatusCallback(const std_msgs::msg::String::SharedPtr message);
  void guardStatusCallback(const std_msgs::msg::String::SharedPtr message);
  void localizationHealthCallback(const std_msgs::msg::String::SharedPtr message);
  void publishDiagnostic(const std::string & level, const std::string & reason) const;
  void rejectPlan(const std::string & reason);
  double monotonicNow() const;
  static bool finite(double value);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  std::string navigation_frame_{"map"};
  std::string path_topic_{"/neupan_path"};
  std::string goal_topic_{"/neupan_goal"};
  std::string envelope_topic_{"/neupan_cmd_envelope"};
  std::string gate_command_topic_{"/cmd_vel"};
  std::string gate_status_topic_{"/r1_safety_gate_status"};
  std::string guard_status_topic_{"/r1_collision_guard/status"};
  std::string localization_health_topic_{"/r1_localization_health"};
  std::string diagnostics_topic_{"/neupan_controller_status"};
  double envelope_timeout_sec_{0.3};
  double envelope_startup_timeout_sec_{1.0};
  double envelope_recovery_timeout_sec_{5.0};
  double max_linear_{0.15};
  double max_reverse_{0.15};
  double max_angular_{0.4};
  bool allow_reverse_{false};
  bool shadow_mode_{true};
  bool require_gate_match_{false};
  double command_match_epsilon_{1e-4};
  double guard_status_timeout_sec_{1.5};
  double guard_startup_timeout_sec_{2.0};
  double guard_stop_timeout_sec_{10.0};
  double localization_health_timeout_sec_{1.5};
  double localization_startup_timeout_sec_{10.0};
  bool require_localization_ready_{true};
  bool active_{false};
  bool plan_valid_{false};
  bool localization_ready_observed_{false};
  int path_version_{0};
  // The worker starts without a goal at version -1.  Retaining that origin
  // keeps independently delivered path and goal updates on the same contract.
  int goal_version_{-1};
  geometry_msgs::msg::PoseStamped published_goal_;
  bool goal_published_{false};
  nav_msgs::msg::Path published_path_;
  bool path_published_{false};
  RouteUpdateGate route_update_gate_;
  std::chrono::steady_clock::time_point plan_published_at_{};
  std::int64_t worker_generation_{-1};
  Envelope envelope_;
  bool envelope_recovery_active_{false};
  std::chrono::steady_clock::time_point envelope_recovery_started_at_{};
  GateStatus gate_status_;
  GuardStatus guard_status_;
  LocalizationHealth localization_health_;
  bool guard_stop_active_{false};
  std::chrono::steady_clock::time_point guard_stop_started_at_{};
  geometry_msgs::msg::Twist gate_command_;
  bool gate_command_received_{false};
  std::mutex mutex_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr envelope_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gate_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gate_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
};

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__NEUPAN_CONTROLLER_HPP_
