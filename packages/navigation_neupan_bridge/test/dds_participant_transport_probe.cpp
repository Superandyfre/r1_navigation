// Test-only participant QoS observer/transport isolation, never built/installed.
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"
#include "fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h"

namespace eprosima::fastdds::dds {
namespace {
nlohmann::json describe(const DomainParticipantQos & qos)
{
  nlohmann::json transports=nlohmann::json::array();
  for (const auto & descriptor:qos.transport().user_transports) {
    if (auto udp=std::dynamic_pointer_cast<rtps::UDPv4TransportDescriptor>(descriptor)) {
      transports.push_back({{"kind","UDPv4"},{"whitelist",udp->interfaceWhiteList},
        {"send_buffer",udp->sendBufferSize},{"receive_buffer",udp->receiveBufferSize}});
    } else if (std::dynamic_pointer_cast<rtps::SharedMemTransportDescriptor>(descriptor)) {
      transports.push_back({{"kind","SHM"}});
    } else {transports.push_back({{"kind","UNKNOWN"}});}
  }
  return {{"builtin",qos.transport().use_builtin_transports},{"user_transports",transports}};
}
}

DomainParticipant * DomainParticipantFactory::create_participant(
  DomainId_t domain,const DomainParticipantQos & qos,
  DomainParticipantListener * listener,const StatusMask & mask)
{
  using Fn=DomainParticipant*(*)(DomainParticipantFactory*,DomainId_t,
    const DomainParticipantQos&,DomainParticipantListener*,const StatusMask&);
  static auto original=reinterpret_cast<Fn>(dlsym(
    dlopen("/opt/ros/humble/lib/libfastrtps.so",RTLD_LAZY|RTLD_NOLOAD),
    "_ZN8eprosima7fastdds3dds24DomainParticipantFactory18create_participantEjRKNS1_20DomainParticipantQosEPNS1_25DomainParticipantListenerERKNS1_10StatusMaskE"));
  if (!original) {std::abort();}
  const auto mode_value=std::getenv("NEUPAN_DDS_PARTICIPANT_DIAGNOSTIC");
  if (!mode_value) {return original(this,domain,qos,listener,mask);}
  const auto opt=std::getenv("NEUPAN_ISOLATED_ROS_TEST");
  const auto local=std::getenv("ROS_LOCALHOST_ONLY");
  if (domain!=192 || !opt || std::string(opt)!="1" || !local || std::string(local)!="1") {
    throw std::runtime_error("participant diagnostic requires isolated localhost domain192");
  }
  const std::string mode(mode_value);
  if (mode!="observe" && mode!="udp-loopback") {
    throw std::runtime_error("unknown participant diagnostic mode");
  }
  DomainParticipantQos outgoing=qos;
  if (mode=="udp-loopback") {
    outgoing.transport().use_builtin_transports=false;
    outgoing.transport().user_transports.clear();
    for (const auto & descriptor:qos.transport().user_transports) {
      if (auto udp=std::dynamic_pointer_cast<rtps::UDPv4TransportDescriptor>(descriptor)) {
        auto copy=std::make_shared<rtps::UDPv4TransportDescriptor>(*udp);
        copy->interfaceWhiteList={"127.0.0.1"};
        outgoing.transport().user_transports.push_back(copy);
      } else if (!std::dynamic_pointer_cast<rtps::SharedMemTransportDescriptor>(descriptor)) {
        throw std::runtime_error("unsupported diagnostic transport");
      }
    }
    if (outgoing.transport().user_transports.empty()) {
      throw std::runtime_error("no existing UDP transport to isolate");
    }
  }
  auto participant=original(this,domain,outgoing,listener,mask);
  nlohmann::json record{{"pid",getpid()},{"domain",domain},{"mode",mode},
    {"input",describe(qos)},{"outgoing",describe(outgoing)},{"created",participant!=nullptr}};
  if (participant) {record["actual"]=describe(participant->get_qos());}
  fprintf(stderr,"DDS_PARTICIPANT_TRANSPORT %s\n",record.dump().c_str());
  return participant;
}
}  // namespace eprosima::fastdds::dds
