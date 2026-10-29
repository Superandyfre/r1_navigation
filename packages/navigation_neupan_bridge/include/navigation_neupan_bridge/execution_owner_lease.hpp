#pragma once
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <nlohmann/json.hpp>
#include "r1_neupan_runtime/motion_authorization.hpp"

namespace navigation_neupan_bridge
{
inline std::string executionAttemptId(const std::array<unsigned char, 16> & uuid)
{
  std::ostringstream id;
  id << std::hex << std::setfill('0');
  for (const auto byte : uuid) {id << std::setw(2) << static_cast<unsigned int>(byte);}
  return id.str();
}
// Same-host, deny-by-default client ownership. An accepted goal alone is not
// proof that the client received its ACK or still holds the parent execution.
class ExecutionOwnerLease
{
public:
  void begin(std::string authority, std::string controller, std::string attempt, double now)
  {
    authority_ = std::move(authority); controller_ = std::move(controller);
    attempt_ = std::move(attempt); started_ = now; stamp_ = -1.; sequence_ = 0;
    owner_.clear(); fault_.clear(); payload_ = nullptr;
  }
  bool ready(double now)
  {
    if (!fault_.empty()) {return false;}
    if (stamp_ < 0.) {
      if (now < started_ || now - started_ > 1.) {fault_ = "owner_ack_timeout";}
      return false;
    }
    if (now < stamp_ || now - stamp_ > .5) {fault_ = "owner_lease_expired"; return false;}
    return true;
  }
  void observe(const nlohmann::json & j, double now)
  {
    using namespace r1_neupan_runtime;
    ready(now);
    if (!fault_.empty()) {return;}
    try {
      requireSchema(j, "r1_neupan_execution_owner_v1");
      if (requiredString(j, "authority_boot_id") != authority_ ||
        requiredString(j, "controller_boot_id") != controller_ ||
        requiredString(j, "navigation_attempt_id") != attempt_) {return;}
      const auto owner = requiredString(j, "owner_id");
      const auto sequence = requiredInt(j, "sequence", 1);
      const auto stamp = requiredNumber(j, "monotonic");
      if (stamp > now || now - stamp > .5) {return;}
      if (!owner_.empty() && owner != owner_) {fault_ = "owner_identity_conflict"; return;}
      if (sequence <= sequence_) {
        if (sequence == sequence_ && j != payload_) {fault_ = "owner_sequence_conflict";}
        return;
      }
      if (stamp_ >= 0. && stamp <= stamp_) {return;}
      owner_ = owner; sequence_ = sequence; stamp_ = stamp; payload_ = j;
    } catch (const std::exception &) {fault_ = "invalid_execution_owner";}
  }
  const std::string & fault() const {return fault_;}
private:
  std::string authority_, controller_, attempt_, owner_, fault_;
  double started_{0.}, stamp_{-1.};
  std::int64_t sequence_{0};
  nlohmann::json payload_;
};
}
