//
// Created by ryan on 5/21/17.
//

#include <boost/signals2.hpp>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>
#include <ros/ros.h>

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

void callback_odom(double pos, double vel) {
    ROS_INFO("Odom: Pos:%f Vel:%f", pos, vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jag_test");
    ros::NodeHandle nh;


//    can::JaguarBridge bridge("/dev/ttyUSB0");
    can::JaguarBridge bridge("/dev/ttyUSB1");
    jaguar::JaguarBroadcaster jag_broadcaster(bridge);

    jaguar::Jaguar jag(bridge, 4);

//    jag_broadcaster.system_reset();

    block(jag.config_brake_set(jaguar::BrakeCoastSetting::kOverrideCoast));

    block(jag.speed_set_reference(jaguar::SpeedReference::kQuadratureEncoder));
    block(jag.position_set_reference(jaguar::PositionReference::kQuadratureEncoder));
    block(jag.config_encoders_set(360));
    block(jag.periodic_config_odom(0, boost::bind(&callback_odom, _1, _2) ));
    block(jag.periodic_enable(0, 50));

    block(jag.periodic_config_diag(1, boost::bind(&callback_diag, _1, _2, _3, _4)));
    block(jag.periodic_enable(1, 500));

    block(jag.speed_enable());
//    block(jag.voltage_enable());
    jag_broadcaster.system_resume();

    ros::Rate heartbeat_rate(50);
    while (ros::ok()) {
//        ROS_INFO("Starting loop");
        block(jag.speed_set(10));
//        block(jag.voltage_set(0.5));

        //Send heartbeat
//        ROS_INFO("Sending heartbeat");
        jag_broadcaster.heartbeat();
//        ROS_INFO("Sent heartbeat");

        ros::spinOnce();
        heartbeat_rate.sleep();
//        ROS_INFO("Ending loop");
    }
}