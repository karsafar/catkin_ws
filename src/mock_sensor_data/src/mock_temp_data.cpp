#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_temp_data");
    ros::NodeHandle n;

    ros::Publisher temp_level_pub = n.advertise<std_msgs::Int32>("temp_level", 10);
    ros::Rate loop_rate(1.5);

    int count = 30;

    while (count <= 80 && ros::ok())
    {
        std_msgs::Int32 temp_level_msg;
        if (count < 55)
        {
            count++;
        }
        else if (count > 55)
        {
            sleep(5);
            count = 30;
        }
        if (count == 55)
        {
            count = 60;
        }
        temp_level_msg.data = count;
        temp_level_pub.publish(temp_level_msg);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
}