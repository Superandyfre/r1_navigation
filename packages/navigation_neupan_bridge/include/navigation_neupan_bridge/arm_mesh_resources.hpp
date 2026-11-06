// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
namespace navigation_neupan_bridge
{
struct ArmMeshResource {std::string frame, uri; double scale;};
inline std::vector<ArmMeshResource> armMeshResources()
{
  std::vector<ArmMeshResource> result;
  for (const auto & side : {std::string("left_"),std::string("right_")}) {
    const std::string model=side=="left_" ? "AR5-5_08L-W4C1C5" : "AR5-5_08R-W4C1C5";
    const std::string root="package://rokae_description/meshes/"+model+"/visual/";
    result.push_back({side+"base",root+model+"_base.stl",.001});
    for (int i=1;i<=7;++i) result.push_back({side+"link"+std::to_string(i),root+model+"_link"+std::to_string(i)+".stl",.001});
  }
  for (const auto & side : {std::string("left_"),std::string("right_")}) {
    result.push_back({side+"moduan_link","package://r1_description/meshes/moduan_link.STL",1.});
    const std::string prefix=side=="left_" ? "L_" : "R_";
    const std::string root="package://rpp_components_description/meshes/gripper/agibot/omniPicker/";
    result.push_back({prefix+"omniPicker_base_link",root+"base_link.STL",1.});
    for (const auto & name : {"narrow1_Link","narrow2_Link","narrow3_Link","narrow_loop_Link",
      "wide1_Link","wide2_Link","wide3_Link","wide_loop_Link"}) {
      result.push_back({prefix+"omniPicker_hand_"+name,root+name+".STL",1.});
    }
  }
  return result;
}
inline void appendConfiguredMeshes(std::vector<ArmMeshResource> & resources,
  const std::vector<std::string> & frames, const std::vector<std::string> & uris,
  const std::vector<double> & scales)
{
  if (frames.size()!=uris.size() || frames.size()!=scales.size()) {
    throw std::invalid_argument("additional_mesh_arrays_mismatch");
  }
  for (std::size_t i=0;i<frames.size();++i) {
    if (frames[i].empty() || uris[i].empty() || !std::isfinite(scales[i]) || scales[i]<=0.) {
      throw std::invalid_argument("invalid_additional_mesh");
    }
  }
  for (std::size_t i=0;i<frames.size();++i) {
    resources.push_back({frames[i],uris[i],scales[i]});
  }
}
}
