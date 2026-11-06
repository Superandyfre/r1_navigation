// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>
namespace r1_collision_guard
{
struct Decision
{
  enum class State {PASS, STOP, FAULT};
  State state{State::FAULT};
  std::string reason{"uninitialized"};
  double linear_x{0.0};
  double angular_z{0.0};
};
inline const char * stateName(Decision::State state)
{
  switch (state) {
    case Decision::State::PASS: return "PASS";
    case Decision::State::STOP: return "STOP";
    case Decision::State::FAULT: return "FAULT";
  }
  return "FAULT";
}
}
