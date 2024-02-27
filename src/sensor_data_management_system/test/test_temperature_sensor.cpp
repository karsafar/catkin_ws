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

    virtual void SetUp() override {
        temp_pub = nh.advertise<std_msgs::Int32>("/temp_level", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestTempSensor::statusCallback, this);
    }

    void publishTemp(int temp) {
        std_msgs::Int32 msg;
        msg.data = temp;
        temp_pub.publish(msg);
        ROS_WARN_STREAM("Published Temperature: " << msg.data);
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

    void simulateTemperatureChange(int start_temp, int end_temp, int duration_sec) {
        int step = (end_temp > start_temp) ? 1 : -1;
        int steps = std::abs(end_temp - start_temp);
        for (int i = 0; i <= steps; ++i) {
            publishTemp(start_temp + i * step);
            if (i < steps) {
                ros::Duration(duration_sec / static_cast<double>(steps)).sleep();
            }
        }
    }
};

TEST_F(TestTempSensor, TemperatureErrorState) {
    ros::Duration(1).sleep();
    // Gradually increase temperature from 30 to 55 over 30 seconds
    simulateTemperatureChange(30, 55, 30);

    // Check for ERROR state at 55° C
    waitForStatus("ERROR", ros::Duration(1));

    // Spike temperature to 60° C
    publishTemp(60);
    ros::Duration(1).sleep(); // Allow time for message processing

    // Verify ERROR state is maintained at 60° C
    waitForStatus("ERROR", ros::Duration(5));

    // Drop temperature back to 30° C
    publishTemp(30);
    ros::Duration(1).sleep(); // Allow time for message processing

    // Verify system returns to RUNNING state at 30° C
    waitForStatus("RUNNING", ros::Duration(1));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_temp_sensor");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
