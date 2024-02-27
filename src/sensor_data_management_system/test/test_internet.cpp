#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "gem_navigation/StringStamped.h"

class TestInternetConnection : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher internet_pub;
    ros::Subscriber status_sub;
    gem_navigation::StringStamped::ConstPtr last_status_msg;

    virtual void SetUp() {
        internet_pub = nh.advertise<std_msgs::Int32>("internet_connection", 1);
        status_sub = nh.subscribe("/car_status", 20, &TestInternetConnection::statusCallback, this);
    }

    void publishInternetConnection(int status) {
        std_msgs::Int32 msg;
        msg.data = status;
        internet_pub.publish(msg);
        ROS_WARN_STREAM("Published Internet Connection Status: " << msg.data);
    }

    void statusCallback(const gem_navigation::StringStamped::ConstPtr& msg) {
        last_status_msg = msg;
        ROS_WARN_STREAM("Received Car Status: " << last_status_msg->data);
    }

    void simulateSignalChange(int signal, int durationSeconds, const std::string& expectedStatus) {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> seconds_elapsed = end - start;

        while (seconds_elapsed.count() < durationSeconds && ros::ok()) {
            publishInternetConnection(signal);
            ros::Duration(1).sleep();
            end = std::chrono::system_clock::now();
            seconds_elapsed = end - start;
            ROS_WARN_STREAM("Time Elapsed: " << seconds_elapsed.count() << " seconds");
        }

        waitForStatus(expectedStatus, ros::Duration(2));
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

TEST_F(TestInternetConnection, InternetConnectivityErrorState) 
{
    // Start with a stable connection
    simulateSignalChange(1, 1, "RUNNING");
    // Drop to low signal for 10 seconds, then recover
    simulateSignalChange(0, 9, "RUNNING");

    // Recover back to stable
    simulateSignalChange(1, 1, "RUNNING");

    // Drop to low signal for 12 seconds
    simulateSignalChange(0, 15, "ERROR");

    // Recover back to stable
    simulateSignalChange(1, 1, "RUNNING");

    // Drop to no signal for 20 seconds
    simulateSignalChange(2, 19, "RUNNING");
    
    // Recover back to stable
    simulateSignalChange(1, 1, "RUNNING");
    
    // Drop to no signal for 22 seconds
    simulateSignalChange(2, 25, "ERROR");

    // Recover back to stable
    simulateSignalChange(1, 1, "RUNNING");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_internet_connection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}