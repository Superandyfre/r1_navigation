// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#include <cstdlib>
#include <optional>
#include <string>
#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

TEST(SameUUIDAdapter, RefusesProductionConfigurationBeforeAnyROSIO)
{
  const char * previous = std::getenv("NEUPAN_ISOLATED_ROS_TEST");
  const std::optional<std::string> saved = previous ? std::optional<std::string>(previous) : std::nullopt;
  setenv("NEUPAN_ISOLATED_ROS_TEST", "0", 1);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance("navigation_neupan_bridge/SameUUIDRouteController");
  EXPECT_THROW(controller->configure({}, "FollowPath", nullptr, nullptr), std::runtime_error);
  if (saved) {setenv("NEUPAN_ISOLATED_ROS_TEST", saved->c_str(), 1);}
  else {unsetenv("NEUPAN_ISOLATED_ROS_TEST");}
}

TEST(ActionStallAdapter, RefusesProductionConfigurationBeforeAnyROSIO)
{
  const char * previous = std::getenv("NEUPAN_ISOLATED_ROS_TEST");
  const std::optional<std::string> saved = previous ? std::optional<std::string>(previous) : std::nullopt;
  setenv("NEUPAN_ISOLATED_ROS_TEST", "0", 1);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance("navigation_neupan_bridge/ActionStallController");
  EXPECT_THROW(controller->configure({}, "FollowPath", nullptr, nullptr), std::runtime_error);
  if (saved) {setenv("NEUPAN_ISOLATED_ROS_TEST", saved->c_str(), 1);}
  else {unsetenv("NEUPAN_ISOLATED_ROS_TEST");}
}
