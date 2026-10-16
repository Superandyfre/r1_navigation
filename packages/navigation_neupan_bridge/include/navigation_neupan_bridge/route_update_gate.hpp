#ifndef NAVIGATION_NEUPAN_BRIDGE__ROUTE_UPDATE_GATE_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__ROUTE_UPDATE_GATE_HPP_

#include <chrono>

#include "navigation_neupan_bridge/path_replan_distance.hpp"

namespace navigation_neupan_bridge
{

class RouteUpdateGate
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  bool consider(
    const nav_msgs::msg::Path & candidate,
    const nav_msgs::msg::Path & published,
    TimePoint now)
  {
    const double deviation = forwardRouteDeviation(candidate, published);
    if (deviation < 0.08) {
      pending_count_ = 0;
      return false;
    }
    const bool consistent = pending_count_ > 0 &&
      forwardRouteDeviation(candidate, pending_path_) < 0.05;
    pending_count_ = consistent ? pending_count_ + 1 : 1;
    pending_path_ = candidate;
    const double elapsed = std::chrono::duration<double>(now - last_publication_).count();
    return elapsed >= 2.0 && pending_count_ >= 2;
  }

  void published(TimePoint now)
  {
    pending_count_ = 0;
    last_publication_ = now;
  }

  void reset()
  {
    pending_count_ = 0;
    last_publication_ = TimePoint{};
  }

private:
  nav_msgs::msg::Path pending_path_;
  int pending_count_{0};
  TimePoint last_publication_{};
};

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__ROUTE_UPDATE_GATE_HPP_
