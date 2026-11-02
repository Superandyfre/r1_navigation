#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace navigation_neupan_bridge {
struct SidePoint { double x, y; };
struct SampledSideClearance {
  double left{std::numeric_limits<double>::infinity()};
  double right{std::numeric_limits<double>::infinity()};
  unsigned left_samples{0}, right_samples{0};
};
// Lateral scan-to-polygon-section margins, NOT wall fits or 3D clearance.
// Compare each return with the polygon at the SAME longitudinal coordinate.
inline SampledSideClearance sampledSideClearance(
  const std::vector<SidePoint> & polygon, const std::vector<SidePoint> & scan)
{
  SampledSideClearance result;
  if (polygon.size() < 3) {return result;}
  for (const auto & point : scan) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {continue;}
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (size_t i=0; i<polygon.size(); ++i) {
      const auto & a=polygon[i]; const auto & b=polygon[(i+1)%polygon.size()];
      if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y)) {return SampledSideClearance{};}
      if (point.x < std::min(a.x,b.x) || point.x > std::max(a.x,b.x)) {continue;}
      if (std::abs(b.x-a.x)<1e-9) {
        low=std::min(low,std::min(a.y,b.y)); high=std::max(high,std::max(a.y,b.y));
      } else {
        const double y=a.y+(b.y-a.y)*(point.x-a.x)/(b.x-a.x);
        low=std::min(low,y); high=std::max(high,y);
      }
    }
    if (!std::isfinite(low) || !std::isfinite(high)) {continue;}
    if (point.y>0) {result.left=std::min(result.left,point.y-high); ++result.left_samples;}
    else if (point.y<0) {result.right=std::min(result.right,low-point.y); ++result.right_samples;}
  }
  return result;
}
}  // namespace navigation_neupan_bridge
