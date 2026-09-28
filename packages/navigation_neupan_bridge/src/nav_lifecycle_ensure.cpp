#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class NavLifecycleEnsure : public rclcpp::Node
{
public:
  NavLifecycleEnsure()
  : Node("nav_lifecycle_ensure")
  {
    node_names_ = declare_parameter<std::vector<std::string>>(
      "node_names", {"controller_server", "planner_server", "bt_navigator"});
    for (const auto & node_name : node_names_) {
      const auto prefix = "/" + node_name;
      get_state_clients_.emplace(
        node_name, create_client<lifecycle_msgs::srv::GetState>(prefix + "/get_state"));
      change_state_clients_.emplace(
        node_name, create_client<lifecycle_msgs::srv::ChangeState>(prefix + "/change_state"));
    }
  }

  int ensureActive()
  {
    constexpr int kMaxAttempts = 30;
    for (int attempt = 0; rclcpp::ok() && attempt < kMaxAttempts; ++attempt) {
      bool all_active = true;
      for (const auto & node_name : node_names_) {
        const auto state = getState(node_name);
        if (!state) {
          all_active = false;
          RCLCPP_WARN(get_logger(), "Waiting for %s lifecycle services (%d/%d)",
            node_name.c_str(), attempt + 1, kMaxAttempts);
          continue;
        }
        if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          continue;
        }

        all_active = false;
        if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          RCLCPP_WARN(get_logger(), "%s is unconfigured; configuring it", node_name.c_str());
          changeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_WARN(get_logger(), "%s is inactive; activating it", node_name.c_str());
          changeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        } else {
          RCLCPP_WARN(get_logger(), "%s is in lifecycle state %u; waiting",
            node_name.c_str(), *state);
        }
      }
      if (all_active) {
        RCLCPP_INFO(get_logger(), "NeuPAN Nav2 lifecycle nodes are active");
        return 0;
      }
      rclcpp::sleep_for(1s);
    }

    RCLCPP_ERROR(get_logger(), "NeuPAN Nav2 lifecycle nodes did not reach active state");
    return 1;
  }

private:
  std::optional<uint8_t> getState(const std::string & node_name)
  {
    const auto & client = get_state_clients_.at(node_name);
    if (!client->wait_for_service(1s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return std::nullopt;
    }
    return future.get()->current_state.id;
  }

  void changeState(const std::string & node_name, const uint8_t transition)
  {
    const auto & client = change_state_clients_.at(node_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "%s change_state service is unavailable", node_name.c_str());
      return;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "%s lifecycle transition timed out", node_name.c_str());
      return;
    }
    if (!future.get()->success) {
      RCLCPP_WARN(get_logger(), "%s rejected lifecycle transition %u",
        node_name.c_str(), transition);
    }
  }

  std::vector<std::string> node_names_;
  std::unordered_map<std::string,
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
  std::unordered_map<std::string,
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavLifecycleEnsure>();
  const int result = node->ensureActive();
  rclcpp::shutdown();
  return result;
}
