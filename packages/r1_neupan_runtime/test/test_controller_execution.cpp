#include <gtest/gtest.h>
#include "r1_neupan_runtime/controller_execution.hpp"
using namespace r1_neupan_runtime;
namespace
{
Json guard(double now = 100., int seq = 1, std::string state = "DISARMED", std::string grant = "")
{
  return {{"schema", "r1_collision_guard_status_v1"}, {"authorization_protocol", kGrantSchema},
    {"source_protocol", kSourceSchema}, {"guard_boot_id", "guard-a"}, {"sequence", seq},
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
  return {{"schema", kHealthSchema}, {"boot_id", "worker-a"}, {"sequence", seq}, {"monotonic", now},
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
  EXPECT_EQ(runtime.fault(), "route_source_timeout");
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
  EXPECT_EQ(runtime.fault(), "planner_result_recovery_timeout");
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
