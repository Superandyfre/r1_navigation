// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>
#include <optional>
#include <vector>
#include "r1_neupan_runtime/motion_authorization.hpp"

namespace r1_neupan_runtime
{
inline constexpr char kTrajectoryProofSchema[] = "r1_neupan_trajectory_proof_v1";

// This verifies provenance only. Motion still requires the live task grant,
// worker health, fresh pose/scan and checks at the final actuator boundary.
class TrajectoryProofVerifier
{
public:
  void install(const Source & source, double step, const std::vector<std::array<double,2>> & controls)
  {
    if (!std::isfinite(step) || step<=0. || controls.empty() || controls.size()>1000) {
      throw std::invalid_argument("invalid_trajectory_horizon");
    }
    for (const auto & u:controls) {
      if (!std::isfinite(u[0]) || !std::isfinite(u[1])) {throw std::invalid_argument("invalid_trajectory_action");}
    }
    // The first planned control and published source must be the same action.
    if (std::abs(controls[0][0]-source.linear)>1e-7 || std::abs(controls[0][1]-source.angular)>1e-7) {
      throw std::invalid_argument("trajectory_source_mismatch");
    }
    source_=source;step_=step;controls_=controls;sequence_=0;scan_=source.scan;last_.reset();
  }
  void clear() {source_.reset();controls_.clear();last_.reset();sequence_=0;scan_=0;}

  bool accept(const Json & j, double now, double input_timeout)
  {
    if (!source_ || !std::isfinite(now) || !std::isfinite(input_timeout) || input_timeout<=0.) {return false;}
    try {
      requireSchema(j,kTrajectoryProofSchema);
      const auto context=parseContext(j);
      if (!(context==source_->context) || requiredInt(j,"parent_request_id",1)!=source_->request ||
          requiredInt(j,"parent_cycle_id",1)!=source_->cycle ||
          requiredInt(j,"parent_scan_counter",1)!=source_->scan) {return false;}
      const auto sequence=requiredInt(j,"sequence",1), scan=requiredInt(j,"scan_counter",1);
      if (sequence<=sequence_ || scan<=scan_) {return false;}
      const double plan_start=requiredNumber(j,"plan_started_monotonic"),
        scan_time=requiredNumber(j,"scan_receive_monotonic"), pose_time=requiredNumber(j,"pose_receive_monotonic"),
        started=requiredNumber(j,"validation_started_monotonic"), done=requiredNumber(j,"validated_monotonic"),
        issued=requiredNumber(j,"published_monotonic");
      if (plan_start!=source_->published || scan_time<=plan_start || pose_time>started || scan_time>started ||
          started>done || done>issued || issued>now || now-scan_time>input_timeout ||
          now-pose_time>input_timeout || now-issued>input_timeout ||
          !j.at("valid").is_boolean()) {return false;}
      if (!j.at("valid").get<bool>()) {
        // A newer failed revalidation revokes an earlier proof immediately.
        sequence_=sequence;scan_=scan;last_.reset();return true;
      }
      const double at_start=(started-plan_start)/step_, at_end=(done-plan_start)/step_,
        at_now=(now-plan_start)/step_;
      if (at_start<0. || at_end<0. || at_now<0. || at_start>=controls_.size() ||
          at_end>=controls_.size() || at_now>=controls_.size() ||
          static_cast<std::size_t>(at_start)!=static_cast<std::size_t>(at_end)) {return false;}
      const auto & expected=controls_[static_cast<std::size_t>(at_start)];
      const auto & command=j.at("command");
      const double linear=requiredNumber(command,"linear_x",false), angular=requiredNumber(command,"angular_z",false);
      if (std::abs(linear-expected[0])>1e-7 || std::abs(angular-expected[1])>1e-7) {return false;}
      sequence_=sequence;scan_=scan;last_=j;return true;
    } catch (const std::exception &) {return false;}
  }
  const std::optional<Json> & latest() const {return last_;}
  std::optional<std::array<double,2>> action(double now, double input_timeout) const
  {
    if (!source_ || !last_ || !std::isfinite(now) || !std::isfinite(input_timeout) || input_timeout<=0.) {
      return std::nullopt;
    }
    const double scan=last_->at("scan_receive_monotonic").get<double>();
    const double pose=last_->at("pose_receive_monotonic").get<double>();
    const double issued=last_->at("published_monotonic").get<double>();
    if (now<issued || now-scan>input_timeout || now-pose>input_timeout || now-issued>input_timeout) {
      return std::nullopt;
    }
    const double knot=(now-source_->published)/step_;
    if (knot<0. || knot>=controls_.size()) {return std::nullopt;}
    return controls_[static_cast<std::size_t>(knot)];
  }
private:
  std::optional<Source> source_;
  double step_{0.};
  std::vector<std::array<double,2>> controls_;
  std::int64_t sequence_{0},scan_{0};
  std::optional<Json> last_;
};
}
