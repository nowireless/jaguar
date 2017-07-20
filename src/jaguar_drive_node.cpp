//
// Created by ryan on 7/12/17.
//

#include <boost/signals2.hpp>

#include <jaguar/jaguar.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <atomic>

std::atomic<double> leftVal;
std::atomic<double> rightVal;

constexpr int kLeftJaguarID = 4;
constexpr int kRightJaguarID = 2;

inline double limit(double v, double limit) {
    return (fabs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    double rotateValue = msg->axes[0];
    double moveValue = msg->axes[1];
    bool squaredInputs = true;

    // local variables to hold the computed PWM values for the motors
    double leftMotorOutput;
    double rightMotorOutput;

    moveValue = limit(moveValue, 1.0);
    rotateValue = limit(rotateValue, 1.0);

    // square the inputs (while preserving the sign) to increase fine control
    // while permitting full power
    if(squaredInputs) {
        moveValue = std::copysign(moveValue * moveValue, moveValue);
        rotateValue = std::copysign(rotateValue * rotateValue, rotateValue);
    }

    if(moveValue > 0.0) {
        if(rotateValue > 0.0) {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = std::max(moveValue, rotateValue);
        } else {
            leftMotorOutput = std::max(moveValue, -rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        }
    } else {
        if(rotateValue > 0.0) {
            leftMotorOutput = -std::max(-moveValue, rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        } else {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = -std::max(-moveValue, -rotateValue);
        }
    }

    leftVal = leftMotorOutput;
    rightVal = rightMotorOutput;
}

void block(can::TokenPtr l, can::TokenPtr r) {
    l->timed_block(boost::posix_time::seconds(2));
    r->timed_block(boost::posix_time::seconds(2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jaguar_drive");
    ros::NodeHandle nh("drive");

    ros::Subscriber joySub = nh.subscribe("/joy", 10, joyCallback);

    can::JaguarBridge bridge("/dev/ttyUSB0");
    jaguar::JaguarBroadcaster broadcaster(bridge);

    broadcaster.heartbeat();

    jaguar::Jaguar left(bridge, kLeftJaguarID);
    jaguar::Jaguar right(bridge, kRightJaguarID);

    // Needs to be performed to resume communication with the jaguars
    ROS_INFO("Resetting jaguars");
    broadcaster.system_reset();
    ROS_INFO("Waiting for them to reset");

    sleep(1);

    ROS_INFO("Configuring break override");
    block(left.config_brake_set(jaguar::BrakeCoastSetting::kOverrideBrake),
          right.config_brake_set(jaguar::BrakeCoastSetting::kOverrideBrake));

    block(left.voltage_enable(), right.voltage_enable());

    ROS_INFO("Enabling");
    broadcaster.system_resume();

    leftVal = 0;
    rightVal = 0;

    ros::Rate rate(50);

    while(ros::ok()) {
        double leftSpeed = leftVal;
        double rightSpeed = rightVal;

        ROS_INFO("L: %f, R: %f", leftSpeed, rightSpeed);

        block(left.voltage_set(leftSpeed), right.voltage_set(rightSpeed));

        broadcaster.heartbeat();

        ros::spinOnce();
        rate.sleep();
    }

}