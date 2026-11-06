#pragma once
#include <cmath>
#include <vector>

namespace navigation_neupan_bridge
{
// Exact conservative containment, not a collision margin. Invalid/nonconvex
// outer shapes are rejected rather than granting motion on uncertain geometry.
inline bool convexFootprintContains(const std::vector<double> & outer,
  const std::vector<double> & inner, double x, double y, double yaw)
{
  if (outer.size() < 6 || inner.size() < 6 || outer.size()%2 || inner.size()%2 ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return false;}
  for (double v:outer) {if (!std::isfinite(v)) {return false;}}
  for (double v:inner) {if (!std::isfinite(v)) {return false;}}
  const size_t count=outer.size()/2;
  int orientation=0;
  for (size_t i=0;i<count;++i) {
    const size_t j=(i+1)%count,k=(i+2)%count;
    const double cross=(outer[2*j]-outer[2*i])*(outer[2*k+1]-outer[2*j+1])-
      (outer[2*j+1]-outer[2*i+1])*(outer[2*k]-outer[2*j]);
    if (std::abs(cross)<=1e-12) {continue;}
    const int sign=cross>0 ? 1 : -1;
    if (orientation && sign!=orientation) {return false;}
    orientation=sign;
  }
  if (!orientation) {return false;}
  // Consistent local turns alone do not exclude self-intersecting stars.
  for (size_t i=0;i<count;++i) {
    const size_t j=(i+1)%count;
    for (size_t k=0;k<count;++k) {
      const double cross=(outer[2*j]-outer[2*i])*(outer[2*k+1]-outer[2*i+1])-
        (outer[2*j+1]-outer[2*i+1])*(outer[2*k]-outer[2*i]);
      if (orientation*cross < -1e-12) {return false;}
    }
  }
  const double c=std::cos(yaw),s=std::sin(yaw);
  for (size_t v=0;v<inner.size();v+=2) {
    const double px=x+c*inner[v]-s*inner[v+1];
    const double py=y+s*inner[v]+c*inner[v+1];
    for (size_t i=0;i<count;++i) {
      const size_t j=(i+1)%count;
      const double cross=(outer[2*j]-outer[2*i])*(py-outer[2*i+1])-
        (outer[2*j+1]-outer[2*i+1])*(px-outer[2*i]);
      if (orientation*cross < -1e-12) {return false;}
    }
  }
  return true;
}
}
