#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_battery_data");
    ros::NodeHandle n;

    ros::Publisher battery_level_pub = n.advertise<std_msgs::Int32>("battery_level", 10);
    ros::Rate loop_rate(1.5);

    int count = 100;

    while (count >= 0 && ros::ok())
    {
        std_msgs::Int32 battery_level_msg;
        if (count > 51)
        {
            count--;
        }
        else if (count < 51)
        {
            sleep(5);
            count = 100;
        }
        if (count == 51)
        {
            count = 49;
        }
        battery_level_msg.data = count;
        
        battery_level_pub.publish(battery_level_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}