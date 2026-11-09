#include <gtest/gtest.h>
#include "r1_neupan_runtime/controller_execution.hpp"
using namespace r1_neupan_runtime;
namespace
{
Json guard(double now = 100., int seq = 1, std::string state = "DISARMED", std::string grant = "")
{
  return {{"schema", "r1_collision_guard_status_v1"}, {"authorization_protocol", kGrantSchema},
    {"health_protocol", kHealthSchema}, {"source_protocol", kSourceSchema}, {"guard_boot_id", "guard-a"}, {"sequence", seq},
    {"monotonic", now},
    {"authorization_state", state}, {"physical_safety_state", "PASS"},
    {"command_barrier_ready", true}, {"grant_id", grant},
    {"navigation_attempt_id", "attempt-a"}, {"authority_boot_id", "authority-a"},
    {"controller_boot_id", "controller-a"}, {"worker_boot_id", "worker-a"},
    {"worker_generation", 7},
    {"path_version", 1}, {"goal_version", 0}, {"output_linear_x", 0.}, {"output_angular_z", 0.}};
}
Json health(
  double now = 100., int seq = 1, std::string attempt = "attempt-a", int path = 1,
  int goal = 0)
{
  return {{"schema", kHealthSchema}, {"planner_state", "ready"}, {"planner_failed_request_id", -1}, {"boot_id", "worker-a"}, {"sequence", seq}, {"monotonic", now},
    {"state", "RUNNING"}, {"worker_generation", 7}, {"navigation_attempt_id", attempt},
    {"path_version", path}, {"goal_version", goal}};
}
Json source(
  double now = 100., int seq = 1, std::string attempt = "attempt-a", int path = 1,
  int goal = 0)
{
  return {{"schema", kSourceSchema}, {"authority_boot_id", "authority-a"},
    {"controller_boot_id", "controller-a"},
    {"navigation_attempt_id", attempt}, {"worker_boot_id", "worker-a"}, {"worker_generation", 7},
    {"path_version", path}, {"goal_version", goal}, {"request_id", seq}, {"cycle_id", seq},
    {"scan_counter", seq},
    {"scan_receive_monotonic", now - .1}, {"request_dispatched_monotonic", now - .08},
    {"worker_solve_end_monotonic", now - .02}, {"planner_result_monotonic", now},
    {"finite", true}, {"command", {{"linear_x", .1}, {"angular_z", .0}}}};
}
class Execution : public ::testing::Test
{
public:
  ControllerExecution runtime{"controller-a"};
  void begin()
  {
    runtime.guard(guard(), 100.);ASSERT_TRUE(runtime.arm(100., true));runtime.begin(
      "authority-a",
      "attempt-a",
      100.);
    runtime.plan(1, 0, 100.);runtime.health(health(), 100.);
  }
  void armed()
  {
    begin();auto g = runtime.tick(100.);ASSERT_TRUE(g);runtime.source(source(), 100.);
    runtime.guard(guard(100.01, 2, "PREPARED", runtime.grantId()), 100.01);
    g = runtime.tick(100.01);ASSERT_TRUE(g);ASSERT_EQ((*g)["state"], "COMMIT");
    runtime.guard(guard(100.02, 3, "ARMED", runtime.grantId()), 100.02);
    ASSERT_TRUE(runtime.permitted(100.02));
  }
};
TEST(ControllerContinuation, RequiresGuardAcknowledgementOfSamePlanAndScan) {
 AuthorizationConfig c;c.continuation_enabled=true;ControllerExecution runtime("controller-a",c);
 runtime.guard(guard(),100.);ASSERT_TRUE(runtime.arm(100.,true));runtime.begin("authority-a","attempt-a",100.);
 runtime.plan(1,0,100.);runtime.health(health(),100.);ASSERT_TRUE(runtime.tick(100.));
 auto src=source();src["trajectory"]={{"step_sec",.25},{"controls",{{.1,0.},{.15,-.1},{.2,0.}}}};
 ASSERT_TRUE(runtime.source(src,100.));runtime.guard(guard(100.01,2,"PREPARED",runtime.grantId()),100.01);
 ASSERT_TRUE(runtime.tick(100.01));runtime.guard(guard(100.02,3,"ARMED",runtime.grantId()),100.02);
 ASSERT_TRUE(runtime.permitted(100.02));
 runtime.health(health(100.3,2),100.3);runtime.guard(guard(100.3,4,"ARMED",runtime.grantId()),100.3);
 runtime.check("authority-a","attempt-a",100.3);ASSERT_TRUE(runtime.tick(100.3));
 runtime.health(health(100.55,3),100.55);runtime.guard(guard(100.55,5,"ARMED",runtime.grantId()),100.55);
 runtime.check("authority-a","attempt-a",100.55);ASSERT_TRUE(runtime.tick(100.55));
 auto proof=src;proof.erase("trajectory");proof["schema"]=kTrajectoryProofSchema;
 proof["parent_request_id"]=1;proof["parent_cycle_id"]=1;proof["parent_scan_counter"]=1;
 proof["sequence"]=1;proof["scan_counter"]=2;proof["plan_started_monotonic"]=100.;
 proof["scan_receive_monotonic"]=100.52;proof["pose_receive_monotonic"]=100.52;
 proof["validation_started_monotonic"]=100.53;proof["validated_monotonic"]=100.54;
 proof["published_monotonic"]=100.55;proof["valid"]=true;
 proof["command"]={{"linear_x",.2},{"angular_z",0.}};
 runtime.continuation(proof,100.56);EXPECT_FALSE(runtime.continuationAction(100.56));
 auto ack=guard(100.57,6,"ARMED",runtime.grantId());
 ack["continuation_parent_request_id"]=1;ack["continuation_scan_counter"]=2;
 runtime.guard(ack,100.57);ASSERT_TRUE(runtime.continuationAction(100.57));
 EXPECT_DOUBLE_EQ(runtime.continuationAction(100.57)->at(0),.2);
 // A later proof reaching Controller before Guard must not discard the
 // still-fresh scan 2 proof that Guard already acknowledged.
 auto later=proof;later["sequence"]=2;later["scan_counter"]=3;
 later["scan_receive_monotonic"]=100.59;later["pose_receive_monotonic"]=100.59;
 later["validation_started_monotonic"]=100.60;later["validated_monotonic"]=100.61;
 later["published_monotonic"]=100.62;runtime.continuation(later,100.63);
 ASSERT_TRUE(runtime.continuationAction(100.63));
 EXPECT_DOUBLE_EQ(runtime.continuationAction(100.63)->at(0),.2);
 ack["sequence"]=7;ack["monotonic"]=100.64;ack["continuation_parent_request_id"]=2;
 runtime.guard(ack,100.64);EXPECT_FALSE(runtime.continuationAction(100.64));
 auto denied=later;denied["valid"]=false;denied["sequence"]=3;denied["scan_counter"]=4;
 denied["scan_receive_monotonic"]=100.65;denied["pose_receive_monotonic"]=100.65;
 denied["validation_started_monotonic"]=100.66;denied["validated_monotonic"]=100.67;
 denied["published_monotonic"]=100.68;runtime.continuation(denied,100.69);
 ack["sequence"]=8;ack["monotonic"]=100.70;ack["continuation_parent_request_id"]=1;
 runtime.guard(ack,100.70);EXPECT_FALSE(runtime.continuationAction(100.70));
}
TEST(ControllerContinuation, RouteHandoverNeedsGuardAcknowledgementOfOldProof) {
 AuthorizationConfig c;c.continuation_enabled=true;ControllerExecution runtime("controller-a",c);
 runtime.guard(guard(),100.);ASSERT_TRUE(runtime.arm(100.,true));runtime.begin("authority-a","attempt-a",100.);
 runtime.plan(1,0,100.);runtime.health(health(),100.);ASSERT_TRUE(runtime.tick(100.));
 auto src=source();src["trajectory"]={{"step_sec",.25},{"controls",{{.1,0.},{.15,-.1},{.2,0.},{.25,0.}}}};
 ASSERT_TRUE(runtime.source(src,100.));runtime.guard(guard(100.01,2,"PREPARED",runtime.grantId()),100.01);
 ASSERT_TRUE(runtime.tick(100.01));runtime.guard(guard(100.02,3,"ARMED",runtime.grantId()),100.02);
 runtime.health(health(100.3,2),100.3);runtime.guard(guard(100.3,4,"ARMED",runtime.grantId()),100.3);
 runtime.check("authority-a","attempt-a",100.3);ASSERT_TRUE(runtime.tick(100.3));
 runtime.health(health(100.55,3),100.55);runtime.guard(guard(100.55,5,"ARMED",runtime.grantId()),100.55);
 runtime.check("authority-a","attempt-a",100.55);ASSERT_TRUE(runtime.tick(100.55));
 auto proof=src;proof.erase("trajectory");proof["schema"]=kTrajectoryProofSchema;
 proof["parent_request_id"]=1;proof["parent_cycle_id"]=1;proof["parent_scan_counter"]=1;
 proof["sequence"]=1;proof["scan_counter"]=2;proof["plan_started_monotonic"]=100.;
 proof["scan_receive_monotonic"]=100.52;proof["pose_receive_monotonic"]=100.52;
 proof["validation_started_monotonic"]=100.53;proof["validated_monotonic"]=100.54;
 proof["published_monotonic"]=100.55;proof["valid"]=true;
 proof["command"]={{"linear_x",.2},{"angular_z",0.}};
 runtime.continuation(proof,100.56);runtime.plan(2,1,100.56);ASSERT_TRUE(runtime.tick(100.56));
 EXPECT_FALSE(runtime.handoverContinuationAction(100.56));
 auto ack=guard(100.57,6,"ROUTE_HANDOVER",runtime.grantId());ack["path_version"]=2;ack["goal_version"]=1;
 ack["continuation_parent_request_id"]=1;ack["continuation_scan_counter"]=2;
 runtime.guard(ack,100.57);ASSERT_TRUE(runtime.handoverContinuationAction(100.57));
 EXPECT_DOUBLE_EQ(runtime.handoverContinuationAction(100.57)->at(0),.2);
 auto newer=proof;newer["sequence"]=2;newer["scan_counter"]=3;
 newer["scan_receive_monotonic"]=100.59;newer["pose_receive_monotonic"]=100.59;
 newer["validation_started_monotonic"]=100.60;newer["validated_monotonic"]=100.61;
 newer["published_monotonic"]=100.62;runtime.continuation(newer,100.63);
 ASSERT_TRUE(runtime.handoverContinuationAction(100.63));
 EXPECT_DOUBLE_EQ(runtime.handoverContinuationAction(100.63)->at(0),.2);
 ack["sequence"]=7;ack["monotonic"]=100.64;ack["continuation_parent_request_id"]=2;
 runtime.guard(ack,100.64);EXPECT_FALSE(runtime.handoverContinuationAction(100.64));
}
TEST(ControllerContinuation, CertifiedNewRouteNeedsGuardAckBeforeCommit) {
 AuthorizationConfig c;c.continuation_enabled=true;c.source_timeout=.1;ControllerExecution runtime("controller-a",c);
 runtime.guard(guard(),100.);ASSERT_TRUE(runtime.arm(100.,true));runtime.begin("authority-a","attempt-a",100.);
 runtime.plan(1,0,100.);runtime.health(health(),100.);ASSERT_TRUE(runtime.tick(100.));
 auto src=source();src["trajectory"]={{"step_sec",.25},{"controls",{{.1,0.},{.15,-.1},{.2,0.},{.25,0.}}}};
 ASSERT_TRUE(runtime.source(src,100.));runtime.guard(guard(100.01,2,"PREPARED",runtime.grantId()),100.01);
 ASSERT_TRUE(runtime.tick(100.01));runtime.guard(guard(100.02,3,"ARMED",runtime.grantId()),100.02);
 runtime.health(health(100.3,2),100.3);runtime.guard(guard(100.3,4,"ARMED",runtime.grantId()),100.3);
 runtime.check("authority-a","attempt-a",100.3);ASSERT_TRUE(runtime.tick(100.3));
 runtime.health(health(100.55,3),100.55);runtime.guard(guard(100.55,5,"ARMED",runtime.grantId()),100.55);
 runtime.check("authority-a","attempt-a",100.55);ASSERT_TRUE(runtime.tick(100.55));
 auto proof=src;proof.erase("trajectory");proof["schema"]=kTrajectoryProofSchema;
 proof["parent_request_id"]=1;proof["parent_cycle_id"]=1;proof["parent_scan_counter"]=1;
 proof["sequence"]=1;proof["scan_counter"]=2;proof["plan_started_monotonic"]=100.;
 proof["scan_receive_monotonic"]=100.52;proof["pose_receive_monotonic"]=100.52;
 proof["validation_started_monotonic"]=100.53;proof["validated_monotonic"]=100.54;
 proof["published_monotonic"]=100.55;proof["valid"]=true;proof["command"]={{"linear_x",.2},{"angular_z",0.}};
 runtime.continuation(proof,100.56);runtime.plan(2,1,100.56);ASSERT_TRUE(runtime.tick(100.56));
 auto next=source(100.56,2,"attempt-a",2,1);
 next["trajectory"]={{"step_sec",.25},{"controls",{{.1,0.},{.15,-.1},{.2,0.}}}};
 EXPECT_FALSE(runtime.source(next,100.56));
 auto checked=proof;checked["path_version"]=2;checked["goal_version"]=1;
 checked["parent_request_id"]=2;checked["parent_cycle_id"]=2;checked["parent_scan_counter"]=2;
 checked["scan_counter"]=3;checked["plan_started_monotonic"]=100.56;
 checked["scan_receive_monotonic"]=100.59;checked["pose_receive_monotonic"]=100.59;
 checked["validation_started_monotonic"]=100.60;checked["validated_monotonic"]=100.61;
 checked["published_monotonic"]=100.62;checked["command"]={{"linear_x",.1},{"angular_z",0.}};
 runtime.continuation(checked,100.63);EXPECT_FALSE(runtime.handoverNewAction(100.63));
 auto ack=guard(100.70,6,"ROUTE_HANDOVER",runtime.grantId());ack["path_version"]=2;ack["goal_version"]=1;
 ack["handover_new_parent_request_id"]=2;ack["handover_new_scan_counter"]=3;
 runtime.guard(ack,100.70);ASSERT_TRUE(runtime.handoverNewAction(100.70));
 ASSERT_FALSE(runtime.handoverNewSourceFresh(100.70));
 const auto grant=runtime.tick(100.70);ASSERT_TRUE(grant);EXPECT_EQ((*grant)["state"],"COMMIT");
}
TEST_F(Execution, NoImplicitArmOrFirstResultAuthorization) {
  runtime.guard(guard(), 100.);
  EXPECT_THROW(runtime.begin("authority-a", "attempt-a", 100.), std::exception);
  EXPECT_FALSE(runtime.arm(100., false));begin();
  EXPECT_FALSE(runtime.permitted(100.));runtime.source(source(), 100.);
  EXPECT_FALSE(runtime.permitted(100.));
}
TEST_F(Execution, MatchingPreparedCommitAndArmedAckRequired) {
  armed();
}
TEST_F(Execution, PreparedAckCannotCommitBeforeFreshSource) {
  begin(); ASSERT_TRUE(runtime.tick(100.));
  runtime.guard(guard(100.01, 2, "PREPARED", runtime.grantId()), 100.01);
  auto g = runtime.tick(100.01); ASSERT_TRUE(g);
  EXPECT_EQ((*g)["state"], "PREPARE");
  ASSERT_TRUE(runtime.source(source(100.02), 100.02));
  g = runtime.tick(100.02); ASSERT_TRUE(g); EXPECT_EQ((*g)["state"], "COMMIT");
}
TEST_F(Execution, AdvancingResultsBridgeBriefMissingHealthWithoutStopping) {
  armed();
  runtime.guard(guard(100.3,4,"ARMED",runtime.grantId()),100.3);
  runtime.check("authority-a","attempt-a",100.3);
  runtime.source(source(100.3,2),100.3);runtime.tick(100.3);
  runtime.guard(guard(100.55,5,"ARMED",runtime.grantId()),100.55);
  runtime.check("authority-a","attempt-a",100.55);
  runtime.source(source(100.55,3),100.55);runtime.tick(100.55);
  EXPECT_TRUE(runtime.permitted(100.55));
  runtime.guard(guard(100.81,6,"ARMED",runtime.grantId()),100.81);
  runtime.check("authority-a","attempt-a",100.81);
  runtime.source(source(100.81,4),100.81);runtime.tick(100.81);
  EXPECT_TRUE(runtime.permitted(100.81));
  runtime.guard(guard(101.01,7,"ARMED",runtime.grantId()),101.01);
  EXPECT_NO_THROW(runtime.check("authority-a","attempt-a",101.01));
  EXPECT_FALSE(runtime.permitted(101.01));
  EXPECT_FALSE(runtime.faulted());
  runtime.health(health(101.02,2),101.02);
  runtime.source(source(101.02,5),101.02);
  runtime.guard(guard(101.02,8,"ARMED",runtime.grantId()),101.02);
  EXPECT_TRUE(runtime.permitted(101.02));
}
TEST_F(Execution, FatalReasonSurvivesTimerEndingExecution) {
  begin(); runtime.fail("worker_fault_latched"); runtime.end(100.01);
  try {
    runtime.check("authority-a", "attempt-a", 100.02); FAIL() << "expected failure";
  } catch (const std::exception & e) {
    EXPECT_STREQ(e.what(), "worker_fault_latched");
  }
}
TEST_F(Execution, AckMustMatchCompleteExecutionContext) {
  armed();
  auto ack = guard(100.03, 4, "ARMED", runtime.grantId());
  ack["path_version"] = 2; runtime.guard(ack, 100.03);
  EXPECT_FALSE(runtime.permitted(100.03));
  ack["sequence"] = 5; ack["monotonic"] = 100.04; ack["path_version"] = 1;
  ack["worker_boot_id"] = "other-worker"; runtime.guard(ack, 100.04);
  EXPECT_FALSE(runtime.permitted(100.04));
}
TEST_F(Execution, WrongGrantAckAndReplayCannotAuthorize) {
  begin();runtime.tick(100.);runtime.source(source(), 100.);
  runtime.guard(guard(100.01, 2, "ARMED", "old-grant"), 100.01);
  EXPECT_FALSE(runtime.permitted(100.01));
  auto j = guard(100.02, 2, "ARMED", runtime.grantId());runtime.guard(j, 100.02);
  EXPECT_FALSE(runtime.permitted(100.02));
}
TEST_F(Execution, LazyStartingHealthDoesNotDeadlockFirstPlan) {
  begin();auto h = health(100.01, 2);h["state"] = "STARTING";
  runtime.health(h, 100.01);EXPECT_FALSE(runtime.tick(100.01));
  runtime.health(health(100.02, 3), 100.02);EXPECT_TRUE(runtime.tick(100.02));
}
TEST_F(Execution, FatalBeforeFirstSourceEndsAttemptAndCannotBeRearmed) {
  begin();auto h = health(100.01, 2);h["state"] = "FAULT_LATCHED";
  h["fault_id"] = "fault-a";h["fault_code"] = "worker_crash";runtime.health(h, 100.01);
  EXPECT_THROW(runtime.check("authority-a", "attempt-a", 100.02), std::exception);
  runtime.end(100.02);EXPECT_FALSE(runtime.arm(100.02, true));
  EXPECT_THROW(runtime.begin("authority-a", "attempt-b", 100.02), std::exception);
}
TEST_F(Execution, BackgroundTimerCannotRenewStalledAction) {
  armed();runtime.health(health(100.4, 2), 100.4);
  runtime.guard(guard(100.4, 4, "ARMED", runtime.grantId()), 100.4);
  runtime.tick(100.4);
  runtime.health(health(100.51, 3), 100.51);
  runtime.guard(guard(100.51, 5, "ARMED", runtime.grantId()), 100.51);
  EXPECT_FALSE(runtime.tick(100.51));EXPECT_EQ(runtime.fault(), "action_loop_stale");
}
TEST_F(Execution, NewWorkerOrGuardBootCannotClearRunLatch) {
  armed();auto g = guard(100.03, 4);g["guard_boot_id"] = "guard-b";runtime.guard(g, 100.03);
  EXPECT_EQ(runtime.fault(), "guard_instance_changed");
  runtime.end(100.03);EXPECT_FALSE(runtime.arm(100.03, true));
}
TEST_F(Execution, RevokeRetiresExactAttemptEvenSamePath) {
  armed();auto revoke = runtime.end(100.03);ASSERT_TRUE(revoke);EXPECT_EQ(
    (*revoke)["state"],
    "REVOKE");
  runtime.guard(guard(100.04, 4), 100.04);
  EXPECT_THROW(runtime.begin("authority-a", "attempt-a", 100.04), std::exception);
  EXPECT_NO_THROW(runtime.begin("authority-a", "attempt-b", 100.04));
  runtime.plan(1, 0, 100.04);runtime.health(health(100.04, 2, "attempt-b"), 100.04);
  EXPECT_TRUE(runtime.tick(100.04));
  EXPECT_FALSE(runtime.source(source(100.04, 2), 100.04));
  EXPECT_TRUE(runtime.source(source(100.04, 2, "attempt-b"), 100.04));
}
TEST_F(Execution, RoutePrepareDoesNotWaitForNewWorkerRouteHealth) {
  armed(); runtime.plan(2, 1, 100.03);
  auto g = runtime.tick(100.03); ASSERT_TRUE(g);
  EXPECT_EQ((*g)["state"], "PREPARE"); EXPECT_EQ((*g)["path_version"], 2);
}
TEST_F(Execution, PreparedAckRequiresCommandZeroBarrierBeforeCommit) {
  begin(); runtime.tick(100.); runtime.source(source(), 100.);
  auto ack = guard(100.01, 2, "PREPARED", runtime.grantId()); ack["command_barrier_ready"] = false;
  runtime.guard(ack, 100.01); auto g = runtime.tick(100.01); ASSERT_TRUE(g);
  EXPECT_EQ((*g)["state"], "PREPARE");
}
TEST_F(Execution, RouteRevisionInvalidatesOldGrantAndAllowsNewSource) {
  armed();runtime.plan(2, 1, 100.03);EXPECT_FALSE(runtime.permitted(100.03));
  runtime.health(health(100.04, 2, "attempt-a", 2, 1), 100.04);
  auto next = runtime.tick(100.04);ASSERT_TRUE(next);EXPECT_EQ((*next)["state"], "PREPARE");
  EXPECT_FALSE(runtime.source(source(100.04, 2), 100.04));
  EXPECT_TRUE(runtime.source(source(100.04, 2, "attempt-a", 2, 1), 100.04));
  auto ack = guard(100.05, 4, "PREPARED", runtime.grantId()); ack["path_version"] = 2;
  ack["goal_version"] = 1;
  runtime.guard(ack, 100.05);runtime.tick(100.05);
  ack["sequence"] = 5; ack["monotonic"] = 100.06; ack["authorization_state"] = "ARMED";
  runtime.guard(ack, 100.06);
  EXPECT_TRUE(runtime.permitted(100.06));
}
TEST_F(Execution, RepeatedRouteChangesCannotRestartFirstSourceGrace) {
  armed();
  for (int i = 1; i <= 22; ++i) {
    const double now = 100. + i * .25;
    runtime.health(health(now, i + 1), now);
    runtime.guard(guard(now, i + 3, "PREPARED", runtime.grantId()), now);
    try {
      runtime.check("authority-a", "attempt-a", now);
      runtime.plan(i + 1, i, now); runtime.tick(now);
    } catch (const std::exception &) {
    }
  }
  EXPECT_FALSE(runtime.faulted());
  EXPECT_EQ(runtime.taskFailure(), "route_source_timeout");
  EXPECT_FALSE(runtime.permitted(105.5));
  EXPECT_TRUE(runtime.end(105.5));
}
TEST_F(Execution, DuplicateSourceNeverExtendsStaleRecoveryDeadline) {
  armed();
  for (int i = 1; i <= 23; ++i) {
    double now = 100. + i * .25;
    runtime.health(health(now, i + 1), now);
    runtime.guard(guard(now, i + 3, "ARMED", runtime.grantId()), now);
    try {
      runtime.check("authority-a", "attempt-a", now);
    } catch (const std::exception &) {
    }
    runtime.source(source(), now);runtime.tick(now);runtime.permitted(now);
  }
  EXPECT_FALSE(runtime.faulted());
  EXPECT_FALSE(runtime.permitted(105.75));
  const auto revoke = runtime.end(105.75);
  ASSERT_TRUE(revoke);
  EXPECT_EQ((*revoke)["state"], "REVOKE");
  EXPECT_FALSE(runtime.active());
}
}  // namespace

TEST_F(Execution, RetryQueryRequiresEndedHealthyArmedRuntime) {
  EXPECT_FALSE(runtime.retryAllowed(100.));
  armed();
  EXPECT_FALSE(runtime.retryAllowed(100.02));
  runtime.end(100.03);
  EXPECT_TRUE(runtime.retryAllowed(100.03));
  EXPECT_FALSE(runtime.retryAllowed(110.));
  runtime.disarm();
  EXPECT_FALSE(runtime.retryAllowed(100.04));
}
TEST_F(Execution, RetryQueryNeverClearsFatalLatch) {
  armed();
  auto fatal = health(100.03, 2);
  fatal["state"] = "FAULT_LATCHED";
  fatal["fault_id"] = "fatal-a"; fatal["fault_code"] = "worker_timeout";
  runtime.health(fatal, 100.03); runtime.end(100.04);
  EXPECT_FALSE(runtime.retryAllowed(100.04));
  runtime.health(health(100.05, 3), 100.05);
  runtime.guard(guard(100.05, 4), 100.05);
  EXPECT_FALSE(runtime.retryAllowed(100.05));
  EXPECT_TRUE(runtime.faulted());
  EXPECT_FALSE(runtime.arm(100.05, true));
}
