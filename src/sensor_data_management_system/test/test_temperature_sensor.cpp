#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestTempSensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher temp_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() {
        temp_pub = nh.advertise<std_msgs::Int32>("temp_level", 10);
        status_sub = nh.subscribe("/car_status", 20, &TestTempSensor::statusCallback, this);
    }

    void publishTemp(int temp) {
        std_msgs::Int32 msg;
        msg.data = temp;
        temp_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM(last_status_msg->data);
    }
};

TEST_F(TestTempSensor, TemperatureErrorState) {
    publishTemp(30);
    ros::Duration(1).sleep(); // Simulate gradual increase
    ros::spinOnce();

    // Check system's response at 30° C
    ASSERT_TRUE(last_status_msg != nullptr);
    ROS_WARN_STREAM(last_status_msg->data);
    EXPECT_EQ(last_status_msg->data, "RUNNING");

    // Gradually increase to 55° C over 30 seconds
    for (int temp = 50; temp <= 55; ++temp) {
        publishTemp(temp);
        ros::Duration(1).sleep(); // Simulate gradual increase
    }

    // Check system's response at 55° C
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "ERROR");

    // Spike to 60° C
    publishTemp(60);

    // Check system's response at 60° C
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "ERROR");

    publishTemp(30);
    ros::Duration(1).sleep(); // Simulate gradual increase
    ros::spinOnce();

    // Check system's response at 30° C
    ASSERT_TRUE(last_status_msg != nullptr);
    ROS_WARN_STREAM(last_status_msg->data);
    EXPECT_EQ(last_status_msg->data, "RUNNING");
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "test_temp_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
