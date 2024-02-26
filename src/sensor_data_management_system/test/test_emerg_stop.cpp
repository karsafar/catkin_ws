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

    virtual void SetUp() {
        emergency_stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
        status_sub = nh.subscribe("/car_status", 20, &TestEmergencyStop::statusCallback, this);
    }

    void triggerEmergencyStop(bool activated) {
        std_msgs::Bool msg;
        msg.data = activated;
        emergency_stop_pub.publish(msg);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM(last_status_msg->data);
    }
};

TEST_F(TestEmergencyStop, EmergencyStopActivation) {
    ros::Duration(1).sleep();
    triggerEmergencyStop(true);
    ros::Duration(1).sleep();

    ros::spinOnce();

    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "ERROR");
    
    
    ros::Duration(1).sleep();
    triggerEmergencyStop(false);
    ros::Duration(1).sleep();
    ros::spinOnce();

    ASSERT_TRUE(last_status_msg != nullptr);
    EXPECT_EQ(last_status_msg->data, "RUNNING");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_emergency_stop");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
