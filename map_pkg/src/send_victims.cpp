
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"
#include "map_pkg/utilities.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unistd.h>

class Sendvictims
{
public:
    Sendvictims(ros::NodeHandle &nh)
    {
        pub_ = nh.advertise<obstacles_msgs::ObstacleArrayMsg>("victims", 10);
        timer_ = nh.createTimer(ros::Duration(1.0), &Sendvictims::timerCallback, this);
    }

private:
    ros::Publisher pub_;
    ros::Timer timer_;

    void timerCallback(const ros::TimerEvent &)
    {
        obstacles_msgs::ObstacleArrayMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";

        // Dummy obstacle for demonstration
        obstacles_msgs::ObstacleMsg obstacle;
        obstacle.polygon.points.resize(4);
        obstacle.polygon.points[0].x = 0.0;
        obstacle.polygon.points[0].y = 0.0;
        obstacle.polygon.points[1].x = 1.0;
        obstacle.polygon.points[1].y = 0.0;
        obstacle.polygon.points[2].x = 1.0;
        obstacle.polygon.points[2].y = 1.0;
        obstacle.polygon.points[3].x = 0.0;
        obstacle.polygon.points[3].y = 1.0;

        msg.obstacles.push_back(obstacle);

        pub_.publish(msg);
        ROS_INFO("Published victims");
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_victims");
    ros::NodeHandle nh;
    Sendvictims node(nh);
    ros::spin();
    return 0;
}
