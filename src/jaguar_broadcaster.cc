#include <cstring>
#include <jaguar/jaguar_broadcaster.h>
#include <ros/ros.h>

namespace jaguar {

JaguarBroadcaster::JaguarBroadcaster(can::CANBridge &can)
	: can_(can)
{
}

JaguarBroadcaster::~JaguarBroadcaster(void)
{
}

void JaguarBroadcaster::system_reset(void)
{
	broadcast(SystemControl::kSystemReset);
}

void JaguarBroadcaster::system_halt(void)
{
	broadcast(SystemControl::kSystemHalt);
}

void JaguarBroadcaster::system_resume(void)
{
	broadcast(SystemControl::kSystemResume);
}

void JaguarBroadcaster::heartbeat(void)
{
	broadcast(SystemControl::kHeartbeat);
}

void JaguarBroadcaster::device_assignment(uint8_t id)
{
	broadcast(SystemControl::kDeviceAssignment, id);
}

void JaguarBroadcaster::synchronous_update(uint8_t group)
{
	broadcast(SystemControl::kSynchronousUpdate, group);
}

void JaguarBroadcaster::broadcast(SystemControl::Enum api)
{
    std::vector<uint8_t> payload;
    ROS_INFO("Packing...");
    uint32_t id = pack_id(0, Manufacturer::kBroadcastMessage, DeviceType::kBroadcastMessage,
    	                  APIClass::kBroadcastMessage, api);
    ROS_INFO("...sending...");
    can_.transaction(can::CANMessage(id, payload));
    ROS_INFO("...sent");
}

template <typename T>
void JaguarBroadcaster::broadcast(SystemControl::Enum api, T const &payload_raw)
{
    std::vector<uint8_t> payload(sizeof(T));
    memcpy(&payload[0], &payload_raw, sizeof(T));


    uint32_t id = pack_id(0, Manufacturer::kBroadcastMessage, DeviceType::kBroadcastMessage,
    	                  APIClass::kBroadcastMessage, api);
   
    can_.transaction(can::CANMessage(id, payload));
   
}

};
