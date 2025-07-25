
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

class Sendborders
{
public:



    Sendborders(ros::NodeHandle &nh)
    {
    
    ROS_INFO("Configuring node.");

    // Declare/read parameters
    nh.param<std::string>("map", data.map_name, "hexagon");
    nh.param<double>("dx", data.dx, 5.0);
    nh.param<double>("dy", data.dy, 5.0);

    if (data.map_name != "hexagon" && data.map_name != "rectangle")
    {
      ROS_ERROR("Map name %s not recognized", data.map_name.c_str());
      ros::shutdown();  // Or handle error differently
      return;
    }

    ROS_INFO("Map name: %s", data.map_name.c_str());
    ROS_INFO("dx: %f", data.dx);
    ROS_INFO("dy: %f", data.dy);

 
   
        pub_ = nh.advertise<geometry_msgs::Polygon>("/map_borders", 10);
        pub_stamped_ = nh.advertise<geometry_msgs::PolygonStamped>("/borders", 10);
 
        timer_ = nh.createTimer(ros::Duration(1.0), &Sendborders::timerCallback, this);
    ROS_INFO("Sendborders Node configured.");
 
    }

private:
    ros::Publisher pub_;
    ros::Publisher pub_stamped_;      
    ros::Timer timer_;
   
    geometry_msgs::Polygon  pol;
    geometry_msgs::PolygonStamped pol_stamped;


    
    // Data obtained from parameters
	struct Data {
	std::string map_name;
	double dx;
	double dy;
	} data;

    void timerCallback(const ros::TimerEvent &)
    {
      

        
  

      if (this->data.map_name == "hexagon") {
        pol = create_hexagon(this->data.dx);
      } else if (this->data.map_name == "rectangle") {
        pol = create_rectangle(this->data.dx, this->data.dy);
      } else {
        ROS_ERROR( "Map name %s not recognized",
	           this->data.map_name.c_str());
      }
        
        
      pol_stamped.header.stamp = ros::Time::now();
      pol_stamped.header.frame_id = "map";
      pol_stamped.polygon = pol;

      // Publish borders
      this->pub_.publish(pol);
      this->pub_stamped_.publish(pol_stamped);

        
      ROS_INFO("Published borders 1");
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_borders");
    ros::NodeHandle nh;
    Sendborders node(nh);
    ros::spin();
    return 0;
}
