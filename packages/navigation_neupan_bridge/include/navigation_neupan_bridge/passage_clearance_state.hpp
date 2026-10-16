#ifndef NAVIGATION_NEUPAN_BRIDGE__PASSAGE_CLEARANCE_STATE_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__PASSAGE_CLEARANCE_STATE_HPP_

#include <string>

namespace navigation_neupan_bridge
{

inline std::string passageClearanceState(
  bool measured, double available_width_m, double required_width_m,
  double left_clearance_m, double right_clearance_m, double target_clearance_m)
{
  if (!measured) {
    return "INSUFFICIENT_SCAN";
  }
  if (available_width_m < required_width_m) {
    return "TOO_NARROW";
  }
  if (left_clearance_m < target_clearance_m || right_clearance_m < target_clearance_m) {
    return "ROUTE_TOO_CLOSE";
  }
  return "CLEAR";
}

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__PASSAGE_CLEARANCE_STATE_HPP_
