//
// Created by ryan on 5/21/17.
//

#include <boost/signals2.hpp>
#include <atomic>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

void block(can::TokenPtr t) {
//    ROS_INFO("Sending...");
//    t->block();
    t->timed_block(boost::posix_time::seconds(2));
//    ROS_INFO("...Sent");
}

void callback_diag(jaguar::LimitStatus::Enum limits, jaguar::Fault::Enum faults,
                   double voltage, double temp) {
    ROS_INFO("Diag: Voltage: %f, Temp: %f", voltage, temp);
}

std::atomic<double> gPos;
std::atomic<double> gVel;

void callback_odom(double pos, double vel) {
    ROS_INFO("Odom: Pos:%f Vel:%f", pos, vel);
    gPos.store(pos);
    gVel.store(vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jag_test");
    ros::NodeHandle nh("jag");

    ros::Publisher posPub = nh.advertise<std_msgs::Float32>("pos", 100); 
    ros::Publisher velPub = nh.advertise<std_msgs::Float32>("vel", 100);
    gPos.store(0);
    gVel.store(0);


//    can::JaguarBridge bridge("/dev/ttyUSB0");
    can::JaguarBridge bridge("/dev/ttyS0");
    jaguar::JaguarBroadcaster jag_broadcaster(bridge);

    jag_broadcaster.heartbeat();

    jaguar::Jaguar jag(bridge, 4);

    // Needs to be performed to resume communication with the jaguars
    ROS_INFO("Resetting jaguar");
    jag_broadcaster.system_reset();
    ROS_INFO("Wait for it to come back up");

    ROS_INFO("Configuring coast override");
    block(jag.config_brake_set(jaguar::BrakeCoastSetting::kOverrideCoast));

    block(jag.speed_set_reference(jaguar::SpeedReference::kQuadratureEncoder));
    block(jag.position_set_reference(jaguar::PositionReference::kQuadratureEncoder));
    block(jag.config_encoders_set(360));
    block(jag.periodic_config_odom(0, boost::bind(&callback_odom, _1, _2) ));
    block(jag.periodic_enable(0, 100));

    block(jag.periodic_config_diag(1, boost::bind(&callback_diag, _1, _2, _3, _4)));
    block(jag.periodic_enable(1, 500));

//    block(jag.speed_enable());
    block(jag.voltage_enable());

    ROS_INFO("Enabling");
    jag_broadcaster.system_resume();

    ros::Rate heartbeat_rate(50);
    while (ros::ok()) {
//        ROS_INFO("Starting loop");
//        block(jag.speed_set(10));
        ROS_INFO("Cmding...");
        block(jag.voltage_set(0.15));
	ROS_INFO("...cmd sent");

        //Send heartbeat
        ROS_INFO("Sending heartbeat...");
        jag_broadcaster.heartbeat();
        ROS_INFO("...sent heartbeat");

        std_msgs::Float32 posMsg;
        posMsg.data = gPos.load();
        posPub.publish(posMsg);

        std_msgs::Float32 velMsg;
        velMsg.data = gVel.load();
        velPub.publish(velMsg);

	ROS_INFO("Spinning...");
        ros::spinOnce();
	ROS_INFO("...done spinning");
        heartbeat_rate.sleep();
//        ROS_INFO("Ending loop");
    }
}
