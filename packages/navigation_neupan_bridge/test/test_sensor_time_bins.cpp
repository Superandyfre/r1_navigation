#include <gtest/gtest.h>
#include "navigation_neupan_bridge/sensor_time_bins.hpp"
using namespace navigation_neupan_bridge;
TEST(SensorTimeBins, IndependentPointAndBeamTimes)
{
  const std::int64_t start=1793948998024088576LL;
  auto point=absoluteNanoseconds(static_cast<double>(start+17000000));
  ASSERT_TRUE(point);
  auto bin=sensorTimeBin(start,*point,5000000,100000000);
  ASSERT_TRUE(bin); EXPECT_EQ(bin->index,3u);
  EXPECT_LE(std::llabs(bin->representative_ns-*point),2500000);
  auto beam=scanBeamTime(start,1000,.0000283375);
  ASSERT_TRUE(beam);EXPECT_NEAR(*beam-start,28337500,1);
}
TEST(SensorTimeBins, RejectsCorruptAndFutureOutsideScan)
{
  constexpr std::int64_t start=1000000000000LL;
  EXPECT_FALSE(absoluteNanoseconds(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(sensorTimeBin(start,start-1,5000000,100000000));
  EXPECT_FALSE(sensorTimeBin(start,start+100000001,5000000,100000000));
  EXPECT_FALSE(sensorTimeBin(start,start,0,100000000));
  EXPECT_FALSE(scanBeamTime(start,1,-1.));
}
