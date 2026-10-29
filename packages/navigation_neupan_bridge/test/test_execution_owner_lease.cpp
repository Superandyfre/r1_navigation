#include <gtest/gtest.h>
#include "navigation_neupan_bridge/execution_owner_lease.hpp"
using navigation_neupan_bridge::ExecutionOwnerLease;
nlohmann::json owner(double stamp=10.1, int sequence=1) {
  return {{"schema","r1_neupan_execution_owner_v1"},{"authority_boot_id","a"},
    {"controller_boot_id","c"},{"navigation_attempt_id","g"},{"owner_id","o"},
    {"sequence",sequence},{"monotonic",stamp}};
}
TEST(ExecutionOwnerLease, MissingAcknowledgementNeverPermits) {
  ExecutionOwnerLease l;l.begin("a","c","g",10.);
  EXPECT_FALSE(l.ready(10.5));EXPECT_TRUE(l.fault().empty());
  EXPECT_FALSE(l.ready(11.01));EXPECT_EQ(l.fault(),"owner_ack_timeout");
  l.observe(owner(11.02),11.02);EXPECT_FALSE(l.ready(11.02));
}
TEST(ExecutionOwnerLease, ExactIdentityAndFreshnessRequired) {
  ExecutionOwnerLease l;l.begin("a","c","g",10.);
  auto j=owner();j["navigation_attempt_id"]="old";l.observe(j,10.1);EXPECT_FALSE(l.ready(10.1));
  j=owner();j["controller_boot_id"]="old";l.observe(j,10.1);EXPECT_FALSE(l.ready(10.1));
  l.observe(owner(10.2),10.1);EXPECT_FALSE(l.ready(10.1));
  l.observe(owner(),10.1);EXPECT_TRUE(l.ready(10.1));
}
TEST(ExecutionOwnerLease, DuplicateCannotRenewAndExpiryIrreversible) {
  ExecutionOwnerLease l;l.begin("a","c","g",10.);l.observe(owner(),10.1);
  l.observe(owner(),10.55);EXPECT_TRUE(l.ready(10.55));
  EXPECT_FALSE(l.ready(10.61));EXPECT_EQ(l.fault(),"owner_lease_expired");
  l.observe(owner(10.62,2),10.62);EXPECT_FALSE(l.ready(10.62));
}
TEST(ExecutionOwnerLease, FreshSequencesRenewWithoutChangingIdentity) {
  ExecutionOwnerLease l;l.begin("a","c","g",10.);l.observe(owner(),10.1);
  l.observe(owner(10.5,2),10.5);EXPECT_TRUE(l.ready(10.9));
  auto j=owner(10.91,3);j["owner_id"]="other";l.observe(j,10.91);
  EXPECT_EQ(l.fault(),"owner_identity_conflict");
}
TEST(ExecutionOwnerLease, ConflictingSequenceFailsClosed) {
  ExecutionOwnerLease l;l.begin("a","c","g",10.);l.observe(owner(),10.1);
  l.observe(owner(10.2,1),10.2);EXPECT_EQ(l.fault(),"owner_sequence_conflict");
}
