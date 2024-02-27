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
        status_sub = nh.subscribe("/car_status", 1, &TestBatterySensor::statusCallback, this);
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

    void waitForStatus(const std::string& expected_status, const ros::Duration& timeout) {
        auto start = ros::Time::now();
        ros::Rate rate(10); // Check at 10Hz
        while (ros::Time::now() - start < timeout && ros::ok()) {
            ros::spinOnce();
            if (last_status_msg && last_status_msg->data == expected_status) {
                return;
            }
            rate.sleep();
        }
        ASSERT_TRUE(last_status_msg != nullptr) << "No status message received within timeout.";
        EXPECT_EQ(last_status_msg->data, expected_status) << "Expected status '" << expected_status << "', but got '" << (last_status_msg ? last_status_msg->data : "none") << "'.";
    }
};


TEST_F(TestBatterySensor, BatteryErrorState) {
    ros::Duration(5).sleep();
    // Gradually decrease from 100 to 51 percent over 30 seconds
    int start_battery = 100;
    int end_battery = 51;
    int duration = 30; // seconds
    for (int battery = start_battery; battery >= end_battery; --battery) {
        publishBattery(battery);
        ros::Duration(duration / double(start_battery - end_battery)).sleep(); // Simulate gradual decrease
        waitForStatus("RUNNING", ros::Duration(2)); // Expect system to be in RUNNING state during discharge
    }

    // Test various battery levels and their expected responses
    std::vector<std::pair<int, std::string>> test_cases = {
        {49, "ERROR"},  // Battery level under 50% should trigger ERROR
        {55, "RUNNING"}, // Battery level above 50% should be RUNNING
        {-1, "ERROR"},  // Invalid battery level should trigger ERROR
        {101, "ERROR"}  // Battery level above 100% should trigger ERROR
    };

    for (const auto& test_case : test_cases) {
        publishBattery(test_case.first);
        waitForStatus(test_case.second, ros::Duration(2));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_battery_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}