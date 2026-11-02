#include <gtest/gtest.h>
#include "r1_neupan_runtime/stall_monitor.hpp"
using r1_neupan_runtime::StallMonitor;
TEST(StallMonitor, NeedsFreshActualOdometryAndFinalNonzero) {
  StallMonitor m;
  EXPECT_FALSE(m.observe(1,true,1,0,0,0));
  EXPECT_FALSE(m.observe(4,true,1,0,0,0));
  EXPECT_TRUE(m.observe(4,true,4,0,0,0));
  EXPECT_FALSE(m.observe(5,false,5,0,0,0));
  EXPECT_FALSE(m.observe(6,true,6,0,0,0));
}
TEST(StallMonitor, LegitimateTurningAndTranslationAreProgress) {
  StallMonitor m;
  for (int i=1;i<10;++i) {EXPECT_FALSE(m.observe(i,true,i,0,0,.1*i));}
  m.reset();
  for (int i=1;i<10;++i) {EXPECT_FALSE(m.observe(i,true,i,.02*i,0,0));}
}
TEST(StallMonitor, WaitingAndClockRegressionResetWindow) {
  StallMonitor m;
  EXPECT_FALSE(m.observe(1,true,1,0,0,0));
  EXPECT_FALSE(m.observe(2,false,2,0,0,0));
  EXPECT_FALSE(m.observe(10,true,10,0,0,0));
  EXPECT_FALSE(m.observe(11,true,9,0,0,0));
  EXPECT_FALSE(m.observe(12,true,12,0,0,0));
}
