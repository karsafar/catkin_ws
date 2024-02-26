#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestGPSSensor : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher gps_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() {
        gps_pub = nh.advertise<std_msgs::Int32>("/gps_accuracy", 10);
        status_sub = nh.subscribe("/car_status", 10, &TestGPSSensor::statusCallback, this);
    }

    void publishGPSAccuracy(int accuracy) {
        std_msgs::Int32 msg;
        msg.data = accuracy;
        gps_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
    }
};

TEST_F(TestGPSSensor, GPSErrorState) {
    // Start with high accuracy
    publishGPSAccuracy(100);
    ros::Duration(1).sleep(); // Allow time for message processing
    
    // Check system's response
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "RUNNING");

    // Intermittently drop accuracy to more than 200 mm for periods of 10 to 20 seconds
        int period = rand() % 11 + 10; // Random period between 10 and 20 seconds
        for (int sec = 0; sec < period; ++sec) {
            publishGPSAccuracy(200 + rand() % 801); // Accuracy drops between 200 and 1000 mm
            ros::Duration(1).sleep();
        }

    // Check system's response
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "ERROR");

    // reset the error
    publishGPSAccuracy(199);
    ros::Duration(1).sleep(); // Allow time for message processing

    // Check system's response
    ros::spinOnce();
    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "RUNNING");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gps_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
