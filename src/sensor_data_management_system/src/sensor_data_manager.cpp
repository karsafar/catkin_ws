#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "gem_navigation/StringStamped.h"
#include <chrono>

class SensorDataManager {
public:
    SensorDataManager() {
        car_status_pub = nh.advertise<gem_navigation::StringStamped>("/car_status", 1, true);
        sub_battery = nh.subscribe("battery_level", 10, &SensorDataManager::battery_callback, this);
        sub_gps = nh.subscribe("gps_accuracy", 10, &SensorDataManager::gps_callback, this);
        sub_internet = nh.subscribe("internet_connection", 10, &SensorDataManager::internet_callback, this);
        sub_temp = nh.subscribe("temp_level", 10, &SensorDataManager::temp_callback, this);
        sub_emerg = nh.subscribe("emergency_stop", 10, &SensorDataManager::emerg_callback, this);
    }

    void battery_callback(const std_msgs::Int32::ConstPtr& battery_msg);
    void gps_callback(const std_msgs::Int32::ConstPtr& gps_msg);
    void internet_callback(const std_msgs::Int32::ConstPtr& internet_msg);
    void temp_callback(const std_msgs::Int32::ConstPtr& temp_msg);
    void emerg_callback(const std_msgs::Bool::ConstPtr& emerg_msg);   

private:
    ros::NodeHandle nh;
    ros::Publisher car_status_pub;
    ros::Subscriber sub_battery, sub_gps, sub_internet, sub_temp, sub_emerg;

    std::chrono::time_point<std::chrono::system_clock> start_gps, end_gps;
    int check_gps_time = 0;

    std::chrono::time_point<std::chrono::system_clock> start_internet, end_internet;
    int check_internet_time = 0;
    int check_internet_time_mode = 1;
};

void SensorDataManager::battery_callback(const std_msgs::Int32::ConstPtr& battery_msg) {
    if (battery_msg->data <= 50)
    {
        ROS_ERROR_STREAM("Battery Level -  " << battery_msg->data << ".Battery is less than or equal to 50./nERROR STATE");
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "ERROR";

        car_status_pub.publish(car_status);
    }
    else 
    {
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        car_status_pub.publish(car_status); 
    }
}

void SensorDataManager::gps_callback(const std_msgs::Int32::ConstPtr& gps_msg) {
    if (gps_msg->data >= 200)
    {
        if (check_gps_time == 0)
        {
            start_gps = std::chrono::system_clock::now();
        }
        check_gps_time = 1;
    }
    else 
    {
        check_gps_time = 0;
    }

    if (check_gps_time == 1)
    {
        end_gps = std::chrono::system_clock::now();
        std::chrono::duration<double> seconds_elapsed = end_gps - start_gps;
        ROS_INFO_STREAM("CHECKING TIME - " << seconds_elapsed.count());
        if (seconds_elapsed.count() >= 15)
        {
            ROS_ERROR_STREAM("GPS ACCURACY more than 200 for 15s./nERROR STATE");
            gem_navigation::StringStamped car_status;
            car_status.header.stamp = ros::Time::now();
            car_status.data = "ERROR";

            car_status_pub.publish(car_status);
        }
    }
    else 
    {
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        car_status_pub.publish(car_status); 
    }
}

void SensorDataManager::internet_callback(const std_msgs::Int32::ConstPtr& internet_msg) {
    if (internet_msg->data == 0 || internet_msg->data == 2)
    {
        if (check_internet_time == 0)
        {
            start_gps = std::chrono::system_clock::now();
        }
        check_internet_time = 1;
        check_internet_time_mode = internet_msg->data;
    }
    else 
    {
        check_internet_time = 0;
    }

    if (check_internet_time == 1)
    {
        end_gps = std::chrono::system_clock::now();
        std::chrono::duration<double> seconds_elapsed = end_gps - start_gps;
        ROS_INFO_STREAM("CHECKING TIME - " << seconds_elapsed.count());

        if (check_internet_time_mode == 0)
        {
            if (seconds_elapsed.count() >= 10)
            {
                ROS_ERROR_STREAM("Internet not connected for 10s./nERROR STATE");
                gem_navigation::StringStamped car_status;
                car_status.header.stamp = ros::Time::now();
                car_status.data = "ERROR";

                car_status_pub.publish(car_status);
            }
        }
        
        if (check_internet_time_mode == 2)
        {
            if (seconds_elapsed.count() >= 20)
            {
                ROS_ERROR_STREAM("Internet low for 20s./nERROR STATE");
                gem_navigation::StringStamped car_status;
                car_status.header.stamp = ros::Time::now();
                car_status.data = "ERROR";

                car_status_pub.publish(car_status);
            }
        }
    }
    else 
    {
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        car_status_pub.publish(car_status); 
    }
}

void SensorDataManager::temp_callback(const std_msgs::Int32::ConstPtr& temp_msg) {
    if (temp_msg->data >= 55)
    {
        ROS_ERROR_STREAM("Temperature -  " << temp_msg->data << "C.Temperature is greater than or equal to 55./n ERROR STATE");
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "ERROR";

        car_status_pub.publish(car_status);
    }
    else 
    {
        ROS_WARN_STREAM("Temperature -  " << temp_msg->data << "C.Temperature is less than 55./n RUNNING STATE");
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        car_status_pub.publish(car_status); 
    }
}

void SensorDataManager::emerg_callback(const std_msgs::Bool::ConstPtr& emerg_msg){
        if (emerg_msg->data == true)
    {
        ROS_ERROR_STREAM("Emergency status -  " << emerg_msg->data << ".Emergency button is pressed./nERROR STATE");
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "ERROR";

        car_status_pub.publish(car_status);
    }
    else 
    {
        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        car_status_pub.publish(car_status); 
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_data_management_node");
    SensorDataManager manager;
    ros::spin();
    return 0;
}
