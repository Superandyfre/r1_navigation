#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MapServerLifecycleEnsure : public rclcpp::Node
{
public:
  MapServerLifecycleEnsure()
  : Node("map_server_lifecycle_ensure")
  {
    get_state_client_ = create_client<lifecycle_msgs::srv::GetState>("/map_server/get_state");
    change_state_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
      "/map_server/change_state");
  }

  int ensure_active()
  {
    constexpr int kMaxAttempts = 30;
    for (int attempt = 0; rclcpp::ok() && attempt < kMaxAttempts; ++attempt) {
      const auto state = get_state();
      if (!state) {
        RCLCPP_WARN(get_logger(), "Waiting for map_server lifecycle services (%d/%d)",
          attempt + 1, kMaxAttempts);
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(get_logger(), "map_server is active");
        return 0;
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_WARN(get_logger(), "map_server is unconfigured; configuring it");
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_WARN(get_logger(), "map_server is inactive; activating it");
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      } else {
        RCLCPP_WARN(get_logger(), "map_server is in lifecycle state %u; waiting", *state);
      }
      rclcpp::sleep_for(1s);
    }

    RCLCPP_ERROR(get_logger(), "map_server did not reach active state before timeout");
    return 1;
  }

private:
  std::optional<uint8_t> get_state()
  {
    if (!get_state_client_->wait_for_service(1s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = get_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return std::nullopt;
    }
    return future.get()->current_state.id;
  }

  void change_state(uint8_t transition)
  {
    if (!change_state_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "map_server change_state service is unavailable");
      return;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto future = change_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "map_server lifecycle transition request timed out");
      return;
    }
    if (!future.get()->success) {
      RCLCPP_WARN(get_logger(), "map_server rejected lifecycle transition %u", transition);
    }
  }

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServerLifecycleEnsure>();
  const int result = node->ensure_active();
  rclcpp::shutdown();
  return result;
}
