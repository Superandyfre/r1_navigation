#include <gtest/gtest.h>
#include "r1_neupan_runtime/trajectory_proof.hpp"
using namespace r1_neupan_runtime;
namespace {
Json base() {return {{"authority_boot_id","authority"},{"controller_boot_id","controller"},
 {"navigation_attempt_id","attempt"},{"worker_boot_id","worker"},{"worker_generation",7},
 {"path_version",1},{"goal_version",0}};}
Source source() {
 auto j=base();j.update(Json{{"schema",kSourceSchema},{"request_id",1},{"cycle_id",1},{"scan_counter",1},
 {"finite",true},{"scan_receive_monotonic",99.9},{"request_dispatched_monotonic",99.92},
 {"worker_solve_end_monotonic",99.98},{"planner_result_monotonic",100.},
 {"command",{{"linear_x",.1},{"angular_z",.2}}}});return parseSource(j,100.);}
Json proof() {
 auto j=base();j.update(Json{{"schema",kTrajectoryProofSchema},{"parent_request_id",1},{"parent_cycle_id",1},
 {"parent_scan_counter",1},{"sequence",1},{"scan_counter",2},{"plan_started_monotonic",100.},
 {"scan_receive_monotonic",100.28},{"pose_receive_monotonic",100.28},
 {"validation_started_monotonic",100.30},{"validated_monotonic",100.31},
 {"published_monotonic",100.32},{"valid",true},{"command",{{"linear_x",.15},{"angular_z",-.1}}}});return j;}
TrajectoryProofVerifier verifier() {
 TrajectoryProofVerifier v;v.install(source(),.25,{{.1,.2},{.15,-.1},{.2,0.}});return v;}
TEST(TrajectoryProof, ValidProofRequiresNewObservationAndCannotReplay) {
 auto v=verifier();auto p=proof();EXPECT_TRUE(v.accept(p,100.33,.5));EXPECT_FALSE(v.accept(p,100.34,.5));
 EXPECT_TRUE(v.latest().has_value());v.clear();EXPECT_FALSE(v.accept(p,100.34,.5));
}
TEST(TrajectoryProof, FreshValidationCanAdvanceKnotAndNewWallRevokes) {
 auto v=verifier();auto p=proof();ASSERT_TRUE(v.accept(p,100.33,.5));
 ASSERT_TRUE(v.action(100.34,.5).has_value());EXPECT_DOUBLE_EQ(v.action(100.34,.5)->at(0),.15);
 ASSERT_TRUE(v.action(100.51,.5).has_value());EXPECT_DOUBLE_EQ(v.action(100.51,.5)->at(0),.2);
 EXPECT_FALSE(v.action(100.8,.5).has_value());
 auto deny=proof();deny["sequence"]=2;deny["scan_counter"]=3;deny["valid"]=false;
 deny["scan_receive_monotonic"]=100.34;deny["pose_receive_monotonic"]=100.34;
 deny["validation_started_monotonic"]=100.35;deny["validated_monotonic"]=100.36;
 deny["published_monotonic"]=100.37;
 EXPECT_TRUE(v.accept(deny,100.38,.5));EXPECT_FALSE(v.action(100.38,.5).has_value());
}
TEST(TrajectoryProof, WrongTaskOrPlanCannotAuthorize) {
 auto v=verifier();auto p=proof();p["navigation_attempt_id"]="replacement";EXPECT_FALSE(v.accept(p,100.33,.5));
 p=proof();p["parent_request_id"]=2;EXPECT_FALSE(v.accept(p,100.33,.5));
 p=proof();p["plan_started_monotonic"]=100.01;EXPECT_FALSE(v.accept(p,100.33,.5));
}
TEST(TrajectoryProof, NoStaleInputsOrKnotCrossing) {
 auto v=verifier();auto p=proof();EXPECT_FALSE(v.accept(p,100.8,.5));
 p["pose_receive_monotonic"]=99.7;EXPECT_FALSE(v.accept(p,100.33,.5));
 p=proof();p["scan_receive_monotonic"]=99.9;EXPECT_FALSE(v.accept(p,100.33,.5));
 p=proof();p["validated_monotonic"]=100.51;p["published_monotonic"]=100.52;EXPECT_FALSE(v.accept(p,100.53,.5));
}
TEST(TrajectoryProof, CannotChangeOrExtendOriginalControlHorizon) {
 auto v=verifier();auto p=proof();p["command"]["linear_x"]=-.5;EXPECT_FALSE(v.accept(p,100.33,.5));
 p=proof();p["valid"]=false;EXPECT_TRUE(v.accept(p,100.33,.5));EXPECT_FALSE(v.latest().has_value());
 p=proof();p["validation_started_monotonic"]=101.;p["validated_monotonic"]=101.01;
 p["published_monotonic"]=101.02;p["scan_receive_monotonic"]=100.99;p["pose_receive_monotonic"]=100.99;
 EXPECT_FALSE(v.accept(p,101.03,.5));
}
TEST(TrajectoryProof, SourceBindsImmutableHorizonAndRejectsInvalidData) {
 auto j=base();j.update(Json{{"schema",kSourceSchema},{"request_id",1},{"cycle_id",1},{"scan_counter",1},
 {"finite",true},{"scan_receive_monotonic",99.9},{"request_dispatched_monotonic",99.92},
 {"worker_solve_end_monotonic",99.98},{"planner_result_monotonic",100.},
 {"command",{{"linear_x",.1},{"angular_z",.2}}},
 {"trajectory",{{"step_sec",.25},{"controls",{{.1,.2},{.15,-.1},{.2,0.}}}}}});
 auto first=parseSource(j,100.);EXPECT_EQ(first.trajectory.size(),3u);EXPECT_DOUBLE_EQ(first.trajectory_step,.25);
 auto altered=j;altered["trajectory"]["controls"][1][0]=.9;
 EXPECT_FALSE(first.sameContents(parseSource(altered,100.)));
 altered=j;altered["trajectory"]["controls"][0][0]=.9;
 EXPECT_THROW(parseSource(altered,100.),std::runtime_error);
 altered=j;altered["trajectory"]["step_sec"]=0.;EXPECT_THROW(parseSource(altered,100.),std::runtime_error);
 TrajectoryProofVerifier v;v.install(first,first.trajectory_step,first.trajectory);
 EXPECT_TRUE(v.accept(proof(),100.33,.5));
}
TEST(TrajectoryProof, SourceActionAndPlanMustMatch) {
 TrajectoryProofVerifier v;EXPECT_THROW(v.install(source(),.25,{{.2,.2}}),std::invalid_argument);
}
}
