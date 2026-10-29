// Test-only decorator. Deliberately not built or installed by package CMake.
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include "behaviortree_cpp_v3/bt_factory.h"

class TestPauseTick : public BT::DecoratorNode
{
public:
  TestPauseTick(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    for (const auto & expected : {std::pair{"ROS_DOMAIN_ID", "192"},
        std::pair{"ROS_LOCALHOST_ONLY", "1"}, std::pair{"NEUPAN_ISOLATED_ROS_TEST", "1"}})
    {
      const auto value = std::getenv(expected.first);
      if (!value || std::string(value) != expected.second) {
        throw std::runtime_error("test pause decorator requires isolated domain192");
      }
    }
  }
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("pause_file")};}
  BT::NodeStatus tick() override
  {
    std::string path;
    if (!getInput("pause_file", path) || !std::filesystem::path(path).is_absolute()) {
      throw std::runtime_error("absolute test pause marker required");
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (std::filesystem::exists(path)) {return BT::NodeStatus::RUNNING;}
    return child_node_->executeTick();
  }
};

BT_REGISTER_NODES(factory) {factory.registerNodeType<TestPauseTick>("TestPauseTick");}
