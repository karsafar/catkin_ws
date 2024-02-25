#include "ros/ros.h"
#include <random>
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_internet_data");
    ros::NodeHandle n;

    ros::Publisher internet_connection_pub = n.advertise<std_msgs::Int32>("internet_connection", 10);
    ros::Rate loop_rate(1);
    std::default_random_engine generator;
    std::uniform_int_distribution<> distr_period(0, 30);
    std::uniform_int_distribution<> distr_conn(0, 2);
    std_msgs::Int32 internet_connection_msg;
    internet_connection_msg.data = 1;
    internet_connection_pub.publish(internet_connection_msg);
    int count = 1;
    while (ros::ok())
    {
        std_msgs::Int32 internet_connection_msg;
        internet_connection_msg.data = 1;
        if (count % 5 == 0)
        {
            internet_connection_msg.data = distr_conn(generator);
        }

        ROS_WARN_STREAM("Count - " << count << " data - " << internet_connection_msg.data);
        if ((internet_connection_msg.data == 0) || (internet_connection_msg.data == 2))
        {
            int period = distr_period(generator);
            ROS_WARN_STREAM("PERIOD - " << period);
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> seconds_elapsed = end - start;
            while (seconds_elapsed.count() < period && ros::ok())
            {
                internet_connection_pub.publish(internet_connection_msg);
                loop_rate.sleep();
                end = std::chrono::system_clock::now();
                seconds_elapsed = end - start;
                ROS_WARN_STREAM("TIME - " << seconds_elapsed.count());
            }
        }
        internet_connection_pub.publish(internet_connection_msg);
        loop_rate.sleep();
        count++;
    }
}