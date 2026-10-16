#pragma once

#include <chrono>
#include <cstdint>

namespace navigation_neupan_bridge
{
enum class StaticMapOverlapTransition {None, Latched, Recovered};

class StaticMapOverlapState
{
public:
  explicit StaticMapOverlapState(double recovery_window_sec)
  : recovery_window_sec_(recovery_window_sec) {}

  StaticMapOverlapTransition observe(
    bool valid, bool fresh_sample, bool lethal, std::chrono::steady_clock::time_point now)
  {
    if (!valid) {
      consecutive_lethal_samples_ = 0;
      clear_samples_ = 0;
      return StaticMapOverlapTransition::None;
    }
    if (!fresh_sample) {
      return StaticMapOverlapTransition::None;
    }
    if (lethal) {
      clear_samples_ = 0;
      if (++consecutive_lethal_samples_ >= 2 && !latched_) {
        latched_ = true;
        return StaticMapOverlapTransition::Latched;
      }
      return StaticMapOverlapTransition::None;
    }

    consecutive_lethal_samples_ = 0;
    if (!latched_) {
      return StaticMapOverlapTransition::None;
    }
    if (clear_samples_++ == 0) {
      clear_since_ = now;
    }
    if (clear_samples_ >= 2 &&
      std::chrono::duration<double>(now - clear_since_).count() >= recovery_window_sec_)
    {
      latched_ = false;
      clear_samples_ = 0;
      return StaticMapOverlapTransition::Recovered;
    }
    return StaticMapOverlapTransition::None;
  }

  void reset()
  {
    latched_ = false;
    consecutive_lethal_samples_ = 0;
    clear_samples_ = 0;
  }

private:
  double recovery_window_sec_;
  bool latched_{false};
  uint32_t consecutive_lethal_samples_{0};
  uint32_t clear_samples_{0};
  std::chrono::steady_clock::time_point clear_since_{};
};
}  // namespace navigation_neupan_bridge
