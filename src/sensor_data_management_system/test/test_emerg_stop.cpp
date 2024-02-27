#include <gtest/gtest.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "gem_navigation/StringStamped.h"

class TestEmergencyStop : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher emergency_stop_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() override {
        emergency_stop_pub = nh.advertise<std_msgs::Bool>("emergency_stop", 1);
        status_sub = nh.subscribe("/car_status", 1, &TestEmergencyStop::statusCallback, this);
    }

    void triggerEmergencyStop(bool activated) {
        std_msgs::Bool msg;
        msg.data = activated;
        emergency_stop_pub.publish(msg);
        ROS_WARN_STREAM("Emergency Stop Triggered: " << (activated ? "Activated" : "Deactivated"));
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
};

TEST_F(TestEmergencyStop, EmergencyStopActivation) {
    ros::Duration(5).sleep();
    // Deactivate emergency stop
    triggerEmergencyStop(false);

    // Wait for system to return to RUNNING state 
    waitForStatus("RUNNING", ros::Duration(5));

    // Trigger emergency stop
    triggerEmergencyStop(true);

    // Wait for ERROR state after emergency stop is triggered
    waitForStatus("ERROR", ros::Duration(5));

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_emergency_stop");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
