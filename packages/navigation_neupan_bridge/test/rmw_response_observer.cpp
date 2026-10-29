// Test-only observer: forwards the original call exactly once, without retries.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <sstream>
#include <unistd.h>
#include "rmw_fastrtps_shared_cpp/custom_service_info.hpp"
#include "rmw/rmw.h"
#include "rmw_fastrtps_shared_cpp/custom_client_info.hpp"
namespace rmw_fastrtps_shared_cpp {
rmw_ret_t __rmw_send_response(const char * id, const rmw_service_t * service,
                             rmw_request_id_t * header, void * response) {
  using Fn=rmw_ret_t(*)(const char*,const rmw_service_t*,rmw_request_id_t*,void*);
  static auto original=reinterpret_cast<Fn>(dlsym(dlopen("/opt/ros/humble/lib/librmw_fastrtps_shared_cpp.so",RTLD_LAZY | RTLD_NOLOAD),"_ZN23rmw_fastrtps_shared_cpp19__rmw_send_responseEPKcPK13rmw_service_sP16rmw_request_id_sPv"));
  if (!original) {std::abort();}
  const auto domain=std::getenv("ROS_DOMAIN_ID"), opt=std::getenv("NEUPAN_ISOLATED_ROS_TEST");
  if (!domain || std::string(domain)!="192" || !opt || std::string(opt)!="1") {
    return original(id,service,header,response);
  }
  auto * info=static_cast<CustomServiceInfo*>(service->data);
  eprosima::fastrtps::rtps::GUID_t reader;
  copy_from_byte_array_to_fastrtps_guid(header->writer_guid,&reader);
  const auto start=std::chrono::steady_clock::now();
  const bool before=info->pub_listener_->wait_for_subscription(reader,std::chrono::nanoseconds(0));
  const auto ret=original(id,service,header,response);
  const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  const bool after=info->pub_listener_->wait_for_subscription(reader,std::chrono::nanoseconds(0));
  std::vector<eprosima::fastrtps::rtps::InstanceHandle_t> handles;
  const auto rc=info->response_writer_->get_matched_subscriptions(handles);
  bool actual_match=false;
  for (const auto & handle:handles) {actual_match |= eprosima::fastrtps::rtps::iHandle2GUID(handle)==reader;}
  if (ret!=0) {fprintf(stderr,"RMW_WRITER_MATCHES ret=%u count=%zu target_present=%d\n",rc(),handles.size(),actual_match);}
  std::ostringstream guid;guid<<reader;
  const auto monotonic=std::chrono::duration<double>(start.time_since_epoch()).count();
  fprintf(stderr,"RMW_RESPONSE_OBSERVE pid=%d monotonic=%.9f service=%s sequence=%ld reader=%s matched_before=%d matched_after=%d ret=%d seconds=%.6f\n",getpid(),monotonic,service->service_name,header->sequence_number,guid.str().c_str(),before,after,ret,elapsed);
  return ret;
}
rmw_ret_t __rmw_send_request(const char * id, const rmw_client_t * client,
                            const void * request, int64_t * sequence) {
  using Fn=rmw_ret_t(*)(const char*,const rmw_client_t*,const void*,int64_t*);
  static auto original=reinterpret_cast<Fn>(dlsym(dlopen("/opt/ros/humble/lib/librmw_fastrtps_shared_cpp.so",RTLD_LAZY | RTLD_NOLOAD),"_ZN23rmw_fastrtps_shared_cpp18__rmw_send_requestEPKcPK12rmw_client_sPKvPl"));
  if (!original) {std::abort();}
  const auto domain=std::getenv("ROS_DOMAIN_ID"), opt=std::getenv("NEUPAN_ISOLATED_ROS_TEST");
  if (!domain || std::string(domain)!="192" || !opt || std::string(opt)!="1") {
    return original(id,client,request,sequence);
  }
  const auto start=std::chrono::steady_clock::now();
  const auto request_before=static_cast<CustomClientInfo*>(client->data)->request_publisher_matched_count_.load();
  const auto response_before=static_cast<CustomClientInfo*>(client->data)->response_subscriber_matched_count_.load();
  const auto ret=original(id,client,request,sequence);
  auto * info=static_cast<CustomClientInfo*>(client->data);
  std::ostringstream cached,actual;cached<<info->reader_guid_;actual<<info->response_reader_->guid();
  const auto monotonic=std::chrono::duration<double>(start.time_since_epoch()).count();
  fprintf(stderr,"RMW_REQUEST_OBSERVE pid=%d monotonic=%.9f service=%s seq=%ld cached_reader=%s actual_reader=%s request_before=%zu response_before=%zu request_matches=%zu response_matches=%zu ret=%d\n",getpid(),monotonic,client->service_name,*sequence,cached.str().c_str(),actual.str().c_str(),request_before,response_before,info->request_publisher_matched_count_.load(),info->response_subscriber_matched_count_.load(),ret);
  return ret;
}

}
