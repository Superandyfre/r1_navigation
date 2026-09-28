// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"

TEST(GuardAwareProgressChecker, IsDiscoverableThroughNav2Pluginlib)
{
  pluginlib::ClassLoader<nav2_core::ProgressChecker> loader(
    "nav2_core", "nav2_core::ProgressChecker");
  EXPECT_TRUE(loader.isClassAvailable(
      "navigation_neupan_bridge/GuardAwareProgressChecker"));
  const auto checker = loader.createSharedInstance(
    "navigation_neupan_bridge/GuardAwareProgressChecker");
  EXPECT_NE(checker, nullptr);
}
