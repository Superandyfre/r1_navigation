// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause
//
// Experimental SHADOW ONLY: subscribes to raw sensors and publishes only
// /r1_self_filter_3d_shadow/* for replay and accuracy/performance evaluation.
// It is not wired to navigation or hardware.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometric_shapes/mesh_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>
#include "navigation_neupan_bridge/arm_mesh_resources.hpp"
#include "navigation_neupan_bridge/mesh_surface_query.hpp"
#include "navigation_neupan_bridge/sensor_time_bins.hpp"

namespace navigation_neupan_bridge
{
using Steady = std::chrono::steady_clock;
using Cloud = sensor_msgs::msg::PointCloud2;
using Scan = sensor_msgs::msg::LaserScan;
struct MeshEntry
{
  std::string frame;
  std::unique_ptr<MeshSurfaceQuery> mesh;
};
template<class Message> struct Pending
{
  std::shared_ptr<const Message> msg;
  Steady::time_point received;
  std::uint64_t sequence;
};
enum class Outcome {OK, WAIT_TF, INVALID};

class ArmSelfFilter3dShadow : public rclcpp::Node
{
public:
  ArmSelfFilter3dShadow()
  : Node("r1_arm_self_filter_3d_shadow"), tf_(get_clock()), listener_(tf_)
  {
    const double tolerance=declare_parameter<double>("surface_tolerance_m", .02);
    bin_ns_=static_cast<std::int64_t>(declare_parameter<double>("time_bin_sec", .005)*1e9);
    span_ns_=static_cast<std::int64_t>(declare_parameter<double>("max_scan_span_sec", .20)*1e9);
    tf_wait_=declare_parameter<double>("tf_wait_sec", .12);
    max_reuse_age_=declare_parameter<double>("max_reuse_age_sec", .20);
    reuse_after_=declare_parameter<double>("reuse_after_sec", .04);
    stable_fallback_after_=declare_parameter<double>("stable_fallback_after_sec", .08);
    stable_fallback_span_=declare_parameter<double>("stable_fallback_span_sec", .10);
    if (!std::isfinite(tolerance) || tolerance<=0 || tolerance>.10 ||
      bin_ns_<=0 || span_ns_<=0 || bin_ns_>span_ns_ ||
      !std::isfinite(tf_wait_) || tf_wait_<=0 || tf_wait_>2. ||
      !std::isfinite(max_reuse_age_) || max_reuse_age_<=0 || max_reuse_age_>.5 ||
      !std::isfinite(reuse_after_) || reuse_after_<0 || reuse_after_>=tf_wait_ ||
      !std::isfinite(stable_fallback_after_) || stable_fallback_after_<0 || stable_fallback_after_>=tf_wait_ ||
      !std::isfinite(stable_fallback_span_) || stable_fallback_span_<=0 || stable_fallback_span_>.5)
    {throw std::invalid_argument("invalid shadow filter parameters");}
    for (const auto & resource : armMeshResources()) {
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
        resource.uri,Eigen::Vector3d(resource.scale,resource.scale,resource.scale)));
      if (!mesh || !mesh->vertex_count || !mesh->triangle_count) {
        throw std::runtime_error("mesh_load_failed:"+resource.uri);
      }
      std::vector<fcl::Vector3d> vertices;
      vertices.reserve(mesh->vertex_count);
      for (unsigned i=0; i<mesh->vertex_count; ++i) {
        vertices.emplace_back(mesh->vertices[3*i],mesh->vertices[3*i+1],mesh->vertices[3*i+2]);
      }
      std::vector<fcl::Triangle> triangles;
      triangles.reserve(mesh->triangle_count);
      for (unsigned i=0; i<mesh->triangle_count; ++i) {
        triangles.emplace_back(mesh->triangles[3*i],mesh->triangles[3*i+1],mesh->triangles[3*i+2]);
      }
      auto geometry=std::make_unique<MeshSurfaceQuery>(vertices,triangles,tolerance);
      if (geometry->hasClosedSurface()) {++closed_meshes_;}
      else {open_mesh_frames_.push_back(resource.frame);}
      meshes_.push_back({resource.frame,std::move(geometry)});
    }
    cloud_pub_=create_publisher<Cloud>("/r1_self_filter_3d_shadow/cloud",rclcpp::SensorDataQoS());
    scan_pub_=create_publisher<Scan>("/r1_self_filter_3d_shadow/rear_scan",rclcpp::SensorDataQoS());
    status_pub_=create_publisher<std_msgs::msg::String>("/r1_self_filter_3d_shadow/status",20);
    inputs_=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    processing_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;options.callback_group=inputs_;
    cloud_sub_=create_subscription<Cloud>("/lidar/pointcloud",rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr m) {enqueue(clouds_,m,cloud_drops_);},options);
    scan_sub_=create_subscription<Scan>("/scan",rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr m) {enqueue(scans_,m,scan_drops_);},options);
    timer_=rclcpp::create_wall_timer(std::chrono::milliseconds(5),
      std::bind(&ArmSelfFilter3dShadow::process,this),processing_,
      get_node_base_interface().get(),get_node_timers_interface().get());
    RCLCPP_INFO(get_logger(),"shadow-only 3D filter ready: %zu mesh parts, %zu closed",meshes_.size(),closed_meshes_);
    if (!open_mesh_frames_.empty()) {
      std::string names;for (const auto & frame:open_mesh_frames_) {names+=frame+" ";}
      RCLCPP_WARN(get_logger(),"open meshes: interior classification disabled for %s",names.c_str());
    }
  }
private:
  template<class M> void enqueue(std::deque<Pending<M>> & q, std::shared_ptr<const M> msg,
    std::uint64_t & drops)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (q.size()>=4) {q.pop_front();++drops;}
    q.push_back({std::move(msg),Steady::now(),++sequence_});
  }

  using Poses=std::vector<fcl::Transform3d>;
  using PoseMap=std::map<std::uint32_t,Poses>;
  bool posesFor(const std::string & frame,std::int64_t ns,Poses & result)
  {
    if (frame.empty() || ns<=0) {return false;}
    const rclcpp::Time time(ns,get_clock()->get_clock_type());
    for (const auto & mesh : meshes_) {
      try {
        geometry_msgs::msg::TransformStamped t;
        try {t=tf_.lookupTransform(mesh.frame,frame,time);}
        catch (const tf2::TransformException &) {
          if (!allow_stable_fallback_) {return false;}
          t=tf_.lookupTransform(mesh.frame,frame,rclcpp::Time(0,0,get_clock()->get_clock_type()));
          const std::int64_t latest_ns=static_cast<std::int64_t>(t.header.stamp.sec)*1000000000LL+
            static_cast<std::int64_t>(t.header.stamp.nanosec);
          // Never substitute a future pose for an older sensor sample.
          // Only the latest earlier pose may bridge a short missing update.
          if (latest_ns<=50000000LL || latest_ns>ns ||
            ns-latest_ns>static_cast<std::int64_t>(stable_fallback_span_*1e9)) {return false;}
          const auto previous=tf_.lookupTransform(mesh.frame,frame,
            rclcpp::Time(latest_ns-50000000LL,get_clock()->get_clock_type()));
          const auto & a=t.transform;const auto & b=previous.transform;
          Eigen::Quaterniond qa(a.rotation.w,a.rotation.x,a.rotation.y,a.rotation.z);
          Eigen::Quaterniond qb(b.rotation.w,b.rotation.x,b.rotation.y,b.rotation.z);
          if (!qa.coeffs().allFinite() || !qb.coeffs().allFinite() || qa.norm()<1e-9 || qb.norm()<1e-9) {return false;}
          qa.normalize();qb.normalize();
          const double angle=2.*std::acos(std::clamp(std::abs(qa.dot(qb)),0.,1.));
          const double dx=a.translation.x-b.translation.x,dy=a.translation.y-b.translation.y,
            dz=a.translation.z-b.translation.z;
          if (!std::isfinite(dx)||!std::isfinite(dy)||!std::isfinite(dz)||
            std::sqrt(dx*dx+dy*dy+dz*dz)>.01 || angle>.02) {return false;}
          used_stable_fallback_=true;
          max_pose_time_error_sec_=std::max(max_pose_time_error_sec_,
            static_cast<double>(ns-latest_ns)/1e9+static_cast<double>(bin_ns_)/2e9);
        }
        const auto & q=t.transform.rotation;const auto & p=t.transform.translation;
        Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !rotation.coeffs().allFinite() || rotation.norm()<1e-9) {return false;}
        rotation.normalize();
        fcl::Transform3d v=fcl::Transform3d::Identity();
        v.linear()=rotation.toRotationMatrix();v.translation()=fcl::Vector3d(p.x,p.y,p.z);
        result.push_back(v);
      } catch (const tf2::TransformException &) {return false;}
    }
    return true;
  }
  bool nearAnySurface(const fcl::Vector3d & point,const Poses & poses) const
  {
    for (std::size_t i=0;i<meshes_.size();++i) {
      if (meshes_[i].mesh->nearSurface(poses[i]*point)) {return true;}
    }
    return false;
  }
  static std::int64_t ns(const builtin_interfaces::msg::Time & t)
  {return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;}

  void status(const char * sensor,const char * state,const char * reason,
    std::size_t input,std::size_t removed,double ms,double age,std::uint64_t dropped,bool published,
    std::int64_t input_stamp,std::int64_t output_stamp,double pose_error)
  {
    std_msgs::msg::String m;
    m.data=nlohmann::json{{"schema","r1_self_filter_3d_shadow_v1"},
      {"sensor",sensor},{"state",state},{"reason",reason},{"input_points",input},
      {"filtered_points",removed},{"processing_ms",ms},{"queue_age_sec",age},
      {"dropped_frames",dropped},{"output_published",published},
      {"input_stamp_ns",input_stamp},{"output_stamp_ns",output_stamp},
      {"max_pose_time_error_sec",pose_error},{"mesh_parts",meshes_.size()},{"closed_meshes",closed_meshes_},
      {"open_mesh_frames",open_mesh_frames_},{"motion_authorized",false}}.dump();
    status_pub_->publish(m);
  }

  struct FieldOffsets {int x=-1,y=-1,z=-1,stamp=-1;};
  bool cloudFields(const Cloud & msg,FieldOffsets & o) const
  {
    if (msg.is_bigendian || msg.point_step==0 || msg.width==0 || msg.height==0 ||
      msg.row_step < static_cast<std::uint64_t>(msg.width)*msg.point_step ||
      msg.data.size() < static_cast<std::uint64_t>(msg.row_step)*msg.height) {return false;}
    for (const auto & f:msg.fields) {
      if (f.count!=1) {continue;}
      if (f.name=="x" && f.datatype==sensor_msgs::msg::PointField::FLOAT32) o.x=f.offset;
      if (f.name=="y" && f.datatype==sensor_msgs::msg::PointField::FLOAT32) o.y=f.offset;
      if (f.name=="z" && f.datatype==sensor_msgs::msg::PointField::FLOAT32) o.z=f.offset;
      if (f.name=="timestamp" && f.datatype==sensor_msgs::msg::PointField::FLOAT64) o.stamp=f.offset;
    }
    return o.x>=0 && o.y>=0 && o.z>=0 && o.stamp>=0 &&
      static_cast<std::uint64_t>(std::max({o.x,o.y,o.z})+4)<=msg.point_step &&
      static_cast<std::uint64_t>(o.stamp+8)<=msg.point_step;
  }
  template<class T> static T read(const std::uint8_t * p)
  {T value;std::memcpy(&value,p,sizeof(value));return value;}
  static void invalidate(std::uint8_t * p,int offset)
  {const float value=std::numeric_limits<float>::quiet_NaN();std::memcpy(p+offset,&value,sizeof(value));}

  Outcome cloud(const Cloud & input,std::size_t & removed,std::size_t & samples)
  {
    FieldOffsets offsets;
    if (!cloudFields(input,offsets)) {return Outcome::INVALID;}
    const auto start=ns(input.header.stamp);
    std::map<std::uint32_t,std::int64_t> bins;
    for (std::uint32_t row=0;row<input.height;++row) for (std::uint32_t col=0;col<input.width;++col) {
      const auto * p=input.data.data()+static_cast<std::size_t>(row)*input.row_step+
        static_cast<std::size_t>(col)*input.point_step;
      const float x=read<float>(p+offsets.x),y=read<float>(p+offsets.y),z=read<float>(p+offsets.z);
      if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) {continue;}
      ++samples;
      auto stamp=absoluteNanoseconds(read<double>(p+offsets.stamp));
      if (!stamp) {return Outcome::INVALID;}
      // Float64 epoch nanoseconds are quantised to roughly 256 ns.
      if (*stamp<start && start-*stamp<=1024) {*stamp=start;}
      auto bin=sensorTimeBin(start,*stamp,bin_ns_,span_ns_);
      if (!bin) {return Outcome::INVALID;}
      bins[bin->index]=bin->representative_ns;
    }
    PoseMap poses;
    for (const auto & [index,time]:bins) {
      Poses values;if (!posesFor(input.header.frame_id,time,values)) {return Outcome::WAIT_TF;}
      poses.emplace(index,std::move(values));
    }
    auto output=input;
    for (std::uint32_t row=0;row<output.height;++row) for (std::uint32_t col=0;col<output.width;++col) {
      auto * p=output.data.data()+static_cast<std::size_t>(row)*output.row_step+
        static_cast<std::size_t>(col)*output.point_step;
      const float x=read<float>(p+offsets.x),y=read<float>(p+offsets.y),z=read<float>(p+offsets.z);
      if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) {continue;}
      auto stamp=*absoluteNanoseconds(read<double>(p+offsets.stamp));
      if (stamp<start && start-stamp<=1024) {stamp=start;}
      const auto bin=*sensorTimeBin(start,stamp,bin_ns_,span_ns_);
      if (nearAnySurface({x,y,z},poses.at(bin.index))) {
        invalidate(p,offsets.x);invalidate(p,offsets.y);invalidate(p,offsets.z);++removed;
      }
    }
    output.is_dense=false;
    last_cloud_=std::make_shared<Cloud>(output);cloud_pub_->publish(output);return Outcome::OK;
  }
  Outcome scan(const Scan & input,std::size_t & removed,std::size_t & samples)
  {
    const auto start=ns(input.header.stamp);
    if (start<=0 || input.header.frame_id.empty() || !std::isfinite(input.time_increment) ||
      input.time_increment<=0 || !std::isfinite(input.angle_min) ||
      !std::isfinite(input.angle_increment)) {return Outcome::INVALID;}
    std::map<std::uint32_t,std::int64_t> bins;
    for (std::size_t i=0;i<input.ranges.size();++i) {
      const auto range=input.ranges[i];
      if (!std::isfinite(range)||range<input.range_min||range>input.range_max) {continue;}
      ++samples;auto time=scanBeamTime(start,i,input.time_increment);
      if (!time) {return Outcome::INVALID;}
      auto bin=sensorTimeBin(start,*time,bin_ns_,span_ns_);
      if (!bin) {return Outcome::INVALID;}
      bins[bin->index]=bin->representative_ns;
    }
    PoseMap poses;
    for (const auto & [index,time]:bins) {
      Poses values;if (!posesFor(input.header.frame_id,time,values)) {return Outcome::WAIT_TF;}
      poses.emplace(index,std::move(values));
    }
    auto output=input;
    for (std::size_t i=0;i<output.ranges.size();++i) {
      const auto range=output.ranges[i];
      if (!std::isfinite(range)||range<input.range_min||range>input.range_max) {continue;}
      const auto bin=*sensorTimeBin(start,*scanBeamTime(start,i,input.time_increment),bin_ns_,span_ns_);
      const double angle=input.angle_min+static_cast<double>(i)*input.angle_increment;
      if (nearAnySurface({range*std::cos(angle),range*std::sin(angle),0.},poses.at(bin.index))) {
        output.ranges[i]=std::numeric_limits<float>::infinity();++removed;
      }
    }
    last_scan_=std::make_shared<Scan>(output);scan_pub_->publish(output);return Outcome::OK;
  }

  template<class M,class Function> void processOne(
    std::deque<Pending<M>> & queue,const char * sensor,std::uint64_t & drops,Function function,
    typename rclcpp::Publisher<M>::SharedPtr publisher,std::shared_ptr<M> & last,
    std::uint64_t & reused_sequence,double & last_pose_error)
  {
    Pending<M> item;
    std::uint64_t lost;
    {std::lock_guard<std::mutex> lock(mutex_);if(queue.empty())return;item=queue.front();lost=drops;}
    const auto start=Steady::now();std::size_t removed=0,samples=0;
    allow_stable_fallback_=std::chrono::duration<double>(start-item.received).count() >= stable_fallback_after_;
    used_stable_fallback_=false;
    max_pose_time_error_sec_=static_cast<double>(bin_ns_)/2e9;
    const Outcome result=(this->*function)(*item.msg,removed,samples);
    const double age=std::chrono::duration<double>(Steady::now()-item.received).count();
    if (result==Outcome::WAIT_TF && age<tf_wait_) {
      // Preserve the original sensor stamp and bytes. Never restamp or
      // prolong the lifetime of cached observations while TF catches up.
      if (last && age>=reuse_after_ && reused_sequence!=item.sequence) {
        const double original_age=(get_clock()->now()-
          rclcpp::Time(last->header.stamp,get_clock()->get_clock_type())).seconds();
        if (std::isfinite(original_age) && original_age>=0. && original_age<=max_reuse_age_) {
          publisher->publish(*last);reused_sequence=item.sequence;
          status(sensor,"DEGRADED","reused_last_filtered_original_stamp",0,0,
            std::chrono::duration<double,std::milli>(Steady::now()-start).count(),age,lost,true,
            ns(item.msg->header.stamp),ns(last->header.stamp),last_pose_error);
        }
      }
      return;
    }
    {std::lock_guard<std::mutex> lock(mutex_);if (!queue.empty() && queue.front().sequence==item.sequence) queue.pop_front();}
    const double ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
    if (result==Outcome::OK) {last_pose_error=max_pose_time_error_sec_;}
    status(sensor,result==Outcome::OK&&!used_stable_fallback_?"READY":"DEGRADED",
      result==Outcome::OK?(used_stable_fallback_?"stable_tf_fallback":"filtered"):
      result==Outcome::WAIT_TF?"tf_wait_expired":"invalid_sensor_data",
      samples,removed,ms,age,lost,result==Outcome::OK,ns(item.msg->header.stamp),
      result==Outcome::OK?ns(item.msg->header.stamp):0,max_pose_time_error_sec_);
  }
  void process()
  {
    processOne(clouds_,"front_cloud",cloud_drops_,&ArmSelfFilter3dShadow::cloud,
      cloud_pub_,last_cloud_,cloud_reused_sequence_,last_cloud_pose_error_);
    processOne(scans_,"rear_scan",scan_drops_,&ArmSelfFilter3dShadow::scan,
      scan_pub_,last_scan_,scan_reused_sequence_,last_scan_pose_error_);
  }
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  std::vector<MeshEntry> meshes_;
  std::size_t closed_meshes_{0};std::vector<std::string> open_mesh_frames_;
  std::int64_t bin_ns_{},span_ns_{};double tf_wait_{},max_reuse_age_{},reuse_after_{};
  std::shared_ptr<Cloud> last_cloud_;std::shared_ptr<Scan> last_scan_;
  std::uint64_t cloud_reused_sequence_{0},scan_reused_sequence_{0};
  double stable_fallback_after_{},stable_fallback_span_{};
  bool allow_stable_fallback_{false},used_stable_fallback_{false};
  double max_pose_time_error_sec_{0.},last_cloud_pose_error_{0.},last_scan_pose_error_{0.};
  std::mutex mutex_;std::uint64_t sequence_{0},cloud_drops_{0},scan_drops_{0};
  std::deque<Pending<Cloud>> clouds_;std::deque<Pending<Scan>> scans_;
  rclcpp::CallbackGroup::SharedPtr inputs_,processing_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;rclcpp::Publisher<Scan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);auto node=std::make_shared<navigation_neupan_bridge::ArmSelfFilter3dShadow>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
  executor.add_node(node);executor.spin();rclcpp::shutdown();return 0;
}
