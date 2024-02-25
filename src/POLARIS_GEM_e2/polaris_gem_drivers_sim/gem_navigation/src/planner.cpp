#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "gem_navigation/StringStamped.h"
#include <fstream>

class NavigationTaskPlanner {
public:
    NavigationTaskPlanner() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Initialize subscribers and publishers
        waypoints_pub = nh.advertise<geometry_msgs::PoseArray>("planner_waypoints", 1, true);
        car_status_pub = nh.advertise<gem_navigation::StringStamped>("/car_status", 1, true);
        
        // Load waypoints
        loadWaypoints();
    }

    void loadWaypoints() {
        std::string path = "/catkin_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_navigation/waypoints/wps.csv";
        std::ifstream file(path);
        std::string line;

        while (std::getline(file, line)) {
            std::stringstream linestream(line);
            std::string x, y, yaw;
            std::getline(linestream, x, ',');
            std::getline(linestream, y, ',');
            std::getline(linestream, yaw);

            geometry_msgs::Pose waypoint;
            waypoint.position.x = std::stod(x);
            waypoint.position.y = std::stod(y);
            waypoint.position.z = std::stod(yaw);
            waypoints.poses.push_back(waypoint);
        }

        gem_navigation::StringStamped car_status;
        car_status.header.stamp = ros::Time::now();
        car_status.data = "RUNNING";

        waypoints_pub.publish(waypoints);
        car_status_pub.publish(car_status);

    }

    void run() {
        while (ros::ok()) {
            ros::spinOnce();
        }
    }

private:
    ros::Publisher waypoints_pub;
    ros::Publisher car_status_pub;

    geometry_msgs::PoseArray waypoints;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_task_planner");
    NavigationTaskPlanner planner;
    planner.run();
    return 0;
}