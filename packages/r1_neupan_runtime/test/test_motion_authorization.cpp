#include <gtest/gtest.h>
#include "r1_neupan_runtime/motion_authorization.hpp"
using namespace r1_neupan_runtime;
namespace {
Json context()
{
  return {{"authority_boot_id", "authority-a"}, {"controller_boot_id", "controller-a"},
    {"navigation_attempt_id", "attempt-a"}, {"worker_boot_id", "worker-a"},
    {"worker_generation", 7}, {"path_version", 1}, {"goal_version", 0}};
}
Json health(double now = 100., int sequence = 1)
{
  return {{"schema", kHealthSchema}, {"boot_id", "worker-a"}, {"sequence", sequence},
    {"worker_generation", 7}, {"navigation_attempt_id", "attempt-a"},
    {"state", "RUNNING"}, {"monotonic", now}, {"fault_id", nullptr}, {"fault_code", nullptr}};
}
Json source(double now = 100., int sequence = 1)
{
  auto j = context();
  j.update(Json{{"schema", kSourceSchema}, {"request_id", sequence}, {"cycle_id", sequence},
    {"scan_counter", sequence}, {"finite", true}, {"scan_receive_monotonic", now - .1},
    {"request_dispatched_monotonic", now - .08}, {"worker_solve_end_monotonic", now - .02},
    {"planner_result_monotonic", now}, {"command", {{"linear_x", .1}, {"angular_z", .2}}}});
  return j;
}
Json grant(const char * state = "PREPARE", double now = 100., int sequence = 1)
{
  auto j = context();
  j.update(Json{{"schema", kGrantSchema}, {"guard_boot_id", "guard-a"}, {"grant_id", "grant-a"},
    {"sequence", sequence}, {"state", state}, {"monotonic", now}});
  return j;
}
class Gate : public ::testing::Test {
public:
  MotionAuthorization gate{"guard-a"};
  void prepared() {gate.health(health(),100.); gate.source(source(),100.); gate.authorization(grant(),100.);}
  void armed() {prepared(); gate.authorization(grant("COMMIT",100.,2),100.);}
};
TEST_F(Gate, StartupIsDisarmedAndHealthySourceAloneNeverArms) {
  EXPECT_FALSE(gate.evaluate(100.).fault);
  gate.health(health(),100.); gate.source(source(),100.);
  EXPECT_FALSE(gate.evaluate(100.).allowed);
}
TEST_F(Gate, PrepareRequiresCommitAndCommitCannotPrecedePrepare) {
  gate.health(health(),100.); gate.source(source(),100.);
  gate.authorization(grant("COMMIT"),100.);
  EXPECT_EQ(gate.evaluate(100.).state,"DISARMED");
  gate.authorization(grant(),100.);
  EXPECT_EQ(gate.evaluate(100.).state,"PREPARED");
  gate.authorization(grant("COMMIT",100.,2),100.);
  EXPECT_TRUE(gate.evaluate(100.).allowed);
}
TEST_F(Gate, FatalWinsOverFreshCachedSourceAndNeverClears) {
  armed(); auto j=health(100.01,2); j["state"]="FAULT_LATCHED";
  j["fault_id"]="fault-a"; j["fault_code"]="worker_response_failure";
  gate.health(j,100.01);
  gate.health(health(100.02,3),100.02); gate.source(source(100.02,2),100.02);
  gate.authorization(grant("COMMIT",100.02,3),100.02);
  EXPECT_EQ(gate.evaluate(100.02).reason,"worker_fault_latched");
}
TEST_F(Gate, HealthContinuesButSourceStalesDespiteCommandHeartbeats) {
  armed(); gate.health(health(100.3,2),100.3);
  gate.authorization(grant("COMMIT",100.3,3),100.3);
  gate.health(health(100.55,3),100.55);
  gate.authorization(grant("COMMIT",100.55,4),100.55);
  EXPECT_EQ(gate.evaluate(100.55).state,"SOURCE_STALE_HOLD");
  gate.source(source(100.56,2),100.56);
  EXPECT_TRUE(gate.evaluate(100.56).allowed);
}
TEST_F(Gate, ExactDuplicatesAndRelayDoNotRefreshSource) {
  armed(); auto j=source(); j["relay_sequence"]=500; j["relay_publish_monotonic"]=100.55;
  gate.health(health(100.3,2),100.3);
  gate.authorization(grant("COMMIT",100.3,3),100.3);
  gate.source(j,100.55); gate.health(health(100.55,3),100.55);
  gate.authorization(grant("COMMIT",100.55,4),100.55);
  EXPECT_EQ(gate.evaluate(100.55).reason,"planner_result_stale");
}
TEST_F(Gate, DuplicateIdentityWithChangedCommandIsFatal) {
  armed(); auto j=source(); j["command"]["linear_x"]=.15; gate.source(j,100.01);
  EXPECT_EQ(gate.evaluate(100.01).reason,"duplicate_command_mismatch");
}
TEST_F(Gate, DuplicateIdentityWithChangedTimestampIsFatal) {
  armed(); auto j=source(); j["planner_result_monotonic"]=100.01; gate.source(j,100.01);
  EXPECT_TRUE(gate.evaluate(100.01).fault);
}
TEST_F(Gate, GenerationChangeAndLateOldGenerationCannotRestoreMotion) {
  armed(); auto j=source(100.01,2); j["worker_generation"]=8; gate.source(j,100.01);
  gate.source(source(100.02,3),100.02);
  EXPECT_EQ(gate.evaluate(100.02).reason,"unauthorized_worker_generation");
}
TEST_F(Gate, HealthBootChangeIsFatalAfterPreparation) {
  prepared(); auto j=health(100.01,2); j["boot_id"]="worker-b"; gate.health(j,100.01);
  EXPECT_TRUE(gate.evaluate(100.01).fault);
}
TEST_F(Gate, HealthTimeoutIsIndependentOfFreshSource) {
  armed(); gate.source(source(100.51,2),100.51);
  gate.authorization(grant("COMMIT",100.51,3),100.51);
  EXPECT_EQ(gate.evaluate(100.51).reason,"worker_health_stale");
}
TEST_F(Gate, AuthorityTimeoutIsIndependentOfHealthyWorkerAndSource) {
  armed(); gate.health(health(100.4,2),100.4); gate.source(source(100.4,2),100.4);
  EXPECT_EQ(gate.evaluate(100.51).reason,"navigation_authority_stale");
}
TEST_F(Gate, RevocationIsPermanentEvenBeforePrepare) {
  gate.authorization(grant("REVOKE"),100.);
  gate.health(health(),100.); gate.source(source(),100.);
  gate.authorization(grant("PREPARE",100.,2),100.);
  gate.authorization(grant("COMMIT",100.,3),100.);
  EXPECT_FALSE(gate.evaluate(100.).allowed);
  EXPECT_FALSE(gate.evaluate(100.).fault);
}
TEST_F(Gate, LateMessagesOfRetiredAttemptDoNotPoisonNewAttempt) {
  armed(); gate.authorization(grant("REVOKE",100.01,3),100.01);
  auto h=health(100.02,2); h["navigation_attempt_id"]="attempt-b"; gate.health(h,100.02);
  auto g=grant("PREPARE",100.02,4); g["navigation_attempt_id"]="attempt-b"; g["grant_id"]="grant-b";
  gate.authorization(g,100.02);
  gate.source(source(100.03,2),100.03);
  auto s=source(100.03,2); s["navigation_attempt_id"]="attempt-b"; gate.source(s,100.03);
  g["state"]="COMMIT"; g["sequence"]=5; g["monotonic"]=100.03; gate.authorization(g,100.03);
  EXPECT_TRUE(gate.evaluate(100.03).allowed);
}
TEST_F(Gate, FreshGuardBootRejectsResidualGrant) {
  MotionAuthorization restarted("guard-b");
  restarted.health(health(),100.); restarted.source(source(),100.);
  restarted.authorization(grant(),100.); restarted.authorization(grant("COMMIT",100.,2),100.);
  EXPECT_EQ(restarted.evaluate(100.).state,"DISARMED");
}
TEST_F(Gate, OldInputCannotBeLaunderedByNewPublication) {
  prepared(); auto s=source(100.1,2); s["scan_receive_monotonic"]=98.; gate.source(s,100.1);
  gate.authorization(grant("COMMIT",100.1,2),100.1);
  EXPECT_FALSE(gate.evaluate(100.1).allowed);
}
TEST_F(Gate, UnknownProtocolAndMalformedInputDuringExecutionLatch) {
  armed(); auto s=source(100.01,2); s["schema"]="v_unknown"; gate.source(s,100.01);
  EXPECT_EQ(gate.evaluate(100.01).reason,"invalid_planner_source");
}
TEST_F(Gate, MalformedIdleInputDoesNotCreatePermanentFaultOrAuthorization) {
  gate.health(Json::object(),100.); gate.source(Json::object(),100.);
  EXPECT_FALSE(gate.evaluate(100.).fault); EXPECT_FALSE(gate.evaluate(100.).allowed);
  prepared(); EXPECT_EQ(gate.evaluate(100.).state,"PREPARED");
}
TEST_F(Gate, ReplayedHealthAndAuthorityDoNotRefreshLease) {
  armed(); gate.health(health(),100.4); gate.authorization(grant("COMMIT",100.,2),100.4);
  EXPECT_TRUE(gate.evaluate(100.51).fault);
}
TEST_F(Gate, SameSequenceDifferentHealthCannotRefreshOrClearFault) {
  armed(); auto h=health(); h["monotonic"]=100.01; gate.health(h,100.01);
  EXPECT_TRUE(gate.evaluate(100.01).fault);
}
TEST_F(Gate, RouteRevisionPreservesAttemptButRequiresNewPrepareSourceCommit) {
  armed(); auto h=health(100.1,2); gate.health(h,100.1);
  auto g=grant("PREPARE",100.1,3); g["path_version"]=2; g["goal_version"]=1; g["grant_id"]="route-b";
  gate.authorization(g,100.1); EXPECT_FALSE(gate.evaluate(100.1).allowed);
  gate.source(source(100.1,2),100.1); EXPECT_FALSE(gate.evaluate(100.1).allowed);
  auto s=source(100.12,3); s["path_version"]=2; s["goal_version"]=1; gate.source(s,100.12);
  EXPECT_EQ(gate.evaluate(100.12).state,"PREPARED");
  g["state"]="COMMIT";g["sequence"]=4;g["monotonic"]=100.12;gate.authorization(g,100.12);
  EXPECT_TRUE(gate.evaluate(100.12).allowed);
}
TEST_F(Gate, NullAttemptHealthNeverAuthorizes) {
  auto h=health(); h["navigation_attempt_id"]=nullptr; gate.health(h,100.);
  gate.source(source(),100.);gate.authorization(grant(),100.);
  EXPECT_EQ(gate.evaluate(100.).state,"DISARMED");
}
TEST_F(Gate, StrictNumbersRejectBoolFloatOverflowAndNonfinite) {
  for(const auto & v: {Json(true),Json(1.0),Json("1"),Json(std::uint64_t(-1))}) {
    auto s=source();s["request_id"]=v;EXPECT_THROW(parseSource(s,100.),std::exception);
  }
  auto s=source();s["command"]["linear_x"]=std::numeric_limits<double>::infinity();
  EXPECT_THROW(parseSource(s,100.),std::exception);
  s=source();s["worker_solve_end_monotonic"]=101.;EXPECT_THROW(parseSource(s,100.),std::exception);
}
TEST_F(Gate, InvalidConfigurationRejected) {
  AuthorizationConfig c;c.source_timeout=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MotionAuthorization("boot",c),std::invalid_argument);
}
TEST_F(Gate, RecoveryDeadlineCannotBeExtendedByDuplicateOrLateFreshSource) {
  armed(); gate.health(health(100.3,2),100.3);
  gate.authorization(grant("COMMIT",100.3,3),100.3);
  gate.health(health(100.55,3),100.55);
  gate.authorization(grant("COMMIT",100.55,4),100.55);
  EXPECT_EQ(gate.evaluate(100.55).state,"SOURCE_STALE_HOLD");
  gate.source(source(),100.6);
  for (int i=1; i<=20; ++i) {
    const double now=100.55+i*.25;
    gate.health(health(now,3+i),now);
    gate.authorization(grant("COMMIT",now,4+i),now);
    gate.evaluate(now);
  }
  gate.health(health(105.6,24),105.6);
  gate.authorization(grant("COMMIT",105.6,25),105.6);
  gate.source(source(105.6,2),105.6);
  EXPECT_EQ(gate.evaluate(105.6).reason,"planner_result_recovery_timeout");
}
TEST_F(Gate, StartupPreparationDoesNotConsumeActiveRecoveryDeadline) {
  gate.health(health(),100.); gate.authorization(grant(),100.);
  EXPECT_EQ(gate.evaluate(100.).state,"SOURCE_STALE_HOLD");
  for (int i=1; i<=40; ++i) {
    const double now=100.+i*.25;
    gate.health(health(now,i+1),now);
    gate.authorization(grant("PREPARE",now,i+1),now);
    EXPECT_FALSE(gate.evaluate(now).fault);
  }
  gate.source(source(110.,1),110.);
  gate.authorization(grant("COMMIT",110.,42),110.);
  EXPECT_TRUE(gate.evaluate(110.).allowed);
}
TEST_F(Gate, FatalBeforePrepareRequiresNewWorkerBootAndCannotBeClearedByRunning) {
  auto h=health(); h["state"]="FAULT_LATCHED";h["fault_id"]="fatal-a";h["fault_code"]="worker_crash";
  gate.health(h,100.);gate.health(health(100.01,2),100.01);
  gate.source(source(100.01,1),100.01);gate.authorization(grant("PREPARE",100.01,1),100.01);
  EXPECT_EQ(gate.evaluate(100.01).state,"DISARMED");
}
TEST_F(Gate, TimestampRegressionFutureAndNonfiniteNeverReviveCachedSource) {
  armed();auto s=source(100.01,2);s["planner_result_monotonic"]=99.;
  gate.source(s,100.01);EXPECT_TRUE(gate.evaluate(100.01).fault);
}
TEST_F(Gate, RequestCycleAndScanEachRequireStrictProgression) {
  for(const auto * field : {"request_id","cycle_id","scan_counter"}) {
    MotionAuthorization g("guard-a");g.health(health(),100.);g.source(source(),100.);
    g.authorization(grant(),100.);g.authorization(grant("COMMIT",100.,2),100.);
    auto s=source(100.01,2);s[field]=1;g.source(s,100.01);
    EXPECT_EQ(g.evaluate(100.01).reason,"source_identity_regressed");
  }
}
TEST_F(Gate, DelayedRevokeStillRetiresAttempt) {
  armed();gate.authorization(grant("REVOKE",99.,3),100.1);
  gate.authorization(grant("PREPARE",100.1,4),100.1);
  EXPECT_EQ(gate.evaluate(100.1).state,"DISARMED");
}
TEST_F(Gate, WrongGuardBootAndUnrecognizedOldProtocolCannotAuthorize) {
  gate.health(health(),100.);gate.source(source(),100.);
  auto g=grant();g["guard_boot_id"]="old-guard";gate.authorization(g,100.);
  EXPECT_FALSE(gate.evaluate(100.).allowed);
  g=grant();g["schema"]="old-protocol";gate.authorization(g,100.);
  EXPECT_FALSE(gate.evaluate(100.).allowed);
}
TEST_F(Gate, AuthorityRestartCannotIssueNewGrantAfterAdmittedAttempt) {
  armed();auto g=grant("COMMIT",100.01,3);g["authority_boot_id"]="authority-b";
  gate.authorization(g,100.01);EXPECT_EQ(gate.evaluate(100.01).reason,"authority_instance_changed");
}
TEST_F(Gate, LateAuthorityHeartbeatCannotReviveExpiredGrantBeforeTimerTick) {
  armed();gate.health(health(100.4,2),100.4);gate.source(source(100.4,2),100.4);
  gate.authorization(grant("COMMIT",100.51,3),100.51);
  EXPECT_EQ(gate.evaluate(100.51).reason,"navigation_authority_stale");
}
TEST_F(Gate, LateHealthCannotReviveExpiredWorkerBeforeTimerTick) {
  armed();gate.authorization(grant("COMMIT",100.4,3),100.4);
  gate.health(health(100.51,2),100.51);
  gate.source(source(100.51,2),100.51);
  EXPECT_EQ(gate.evaluate(100.51).reason,"worker_health_stale");
}
TEST_F(Gate, BootIdsDifferAcrossNewInstances) {
  EXPECT_NE(newBootId(),newBootId());
}
}  // namespace
