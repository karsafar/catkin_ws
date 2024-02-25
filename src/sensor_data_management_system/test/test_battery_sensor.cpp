#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestBatterySensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher battery_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() {
        battery_pub = nh.advertise<std_msgs::Int32>("/battery_level", 1);
        status_sub = nh.subscribe("/car_status", 20, &TestBatterySensor::statusCallback, this);
    }

    void publishBattery(int battery) {
        std_msgs::Int32 msg;
        msg.data = battery;
        battery_pub.publish(msg);
        ROS_WARN_STREAM(msg.data);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM(last_status_msg->data);
    }
};

TEST_F(TestBatterySensor, BatteryErrorState) {
    ros::Duration(1).sleep();
    publishBattery(49);
    // ros::Duration(1).sleep(); // Simulate gradual increase
    ros::spinOnce();

    // Check system's response at 49 percent
    ASSERT_TRUE(last_status_msg != nullptr);
    ROS_WARN_STREAM(last_status_msg->data);
    EXPECT_EQ(last_status_msg->data, "ERROR");

    // // Gradually decrease to 51 percent over 30 seconds
    for (int battery = 55; battery <= 51; --battery) {
        publishBattery(battery);
        ros::Duration(1).sleep(); // Simulate gradual increase
    }

    // Check system's response at 49 percent
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "ERROR");

    // Spike to 55 percent
    publishBattery(55);

    // Check system's response at 55 percent
    ros::Duration(1).sleep();
    ros::spinOnce();

    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "RUNNING");
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "test_battery_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
