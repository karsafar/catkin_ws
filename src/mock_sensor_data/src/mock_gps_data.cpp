#include "ros/ros.h"
#include <random>
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_gps_data");
    ros::NodeHandle n;

    ros::Publisher gps_accuracy_pub = n.advertise<std_msgs::Int32>("gps_accuracy", 10);
    ros::Rate loop_rate(10);

    int count = 100;

    std::default_random_engine generator;
    std::uniform_int_distribution<> distr_period(10, 20);
    std::uniform_int_distribution<> distr_accuracy(0, 250);
    std::uniform_int_distribution<> distr_bad_accu(200, 1000);
    while (ros::ok())
    {
        std_msgs::Int32 gps_accuracy_msg;
        int accuarcy = distr_accuracy(generator);
        gps_accuracy_msg.data = accuarcy;
        gps_accuracy_pub.publish(gps_accuracy_msg);
        if (accuarcy >= 200)
        {
            int period = distr_period(generator);
            ROS_WARN_STREAM("PERIOD - " << period);
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> seconds_elapsed = end - start;
            while (seconds_elapsed.count() < period && ros::ok())
            {
                std_msgs::Int32 gps_accuracy_msg;
                gps_accuracy_msg.data = distr_bad_accu(generator);
                gps_accuracy_pub.publish(gps_accuracy_msg);
                loop_rate.sleep();
                end = std::chrono::system_clock::now();
                seconds_elapsed = end - start;
                ROS_WARN_STREAM("TIME - " << seconds_elapsed.count());
            }
        }
        else 
        {
            loop_rate.sleep();
        }
        ros::spinOnce();
    }
}