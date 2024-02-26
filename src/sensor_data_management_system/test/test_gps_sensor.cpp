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

    virtual void SetUp() override {
        gps_pub = nh.advertise<std_msgs::Int32>("/gps_accuracy", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestGPSSensor::statusCallback, this);
    }

    void publishGPSAccuracy(int accuracy) {
        std_msgs::Int32 msg;
        msg.data = accuracy;
        gps_pub.publish(msg);
        ROS_WARN_STREAM("Published GPS Accuracy: " << msg.data);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM("Received Car Status: " << last_status_msg->data);
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

    void simulateGPSAccuracyChange(int start_accuracy, int high_accuracy, int duration_sec) {
        // Start with high accuracy
        publishGPSAccuracy(start_accuracy);
        ros::Duration(1).sleep(); // Allow time for initial high accuracy

        // Jump to high accuracy for duration_sec
        for (int sec = 0; sec < duration_sec; ++sec) {
            publishGPSAccuracy(high_accuracy);
            ros::Duration(1).sleep(); // Maintain high accuracy for each second
        }
    }
};

TEST_F(TestGPSSensor, GPSErrorState) {
    ros::Duration(1).sleep();
    // Jump to high accuracy (above 200) for 14 seconds, no error expected
    simulateGPSAccuracyChange(100, 300, 14);
    waitForStatus("RUNNING", ros::Duration(1));

    // Jump to high accuracy (above 200) for 16 seconds, error expected
    simulateGPSAccuracyChange(100, 300, 16);
    waitForStatus("ERROR", ros::Duration(1));

    // Test values outside the valid range
    publishGPSAccuracy(-1); // Below valid range
    waitForStatus("ERROR", ros::Duration(1));

    publishGPSAccuracy(1001); // Above valid range
    waitForStatus("ERROR", ros::Duration(1));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gps_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
