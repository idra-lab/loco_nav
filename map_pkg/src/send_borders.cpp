// -*- coding: utf-8 -*-
#include <ros/ros.h>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

#include "map_pkg/utilities.hpp"

/**
 * @brief This node publishes the borders of the map.
 * It mirrors the ROS2 node’s parameters and topics using ROS1.
 */
class BordersPublisher
{
private:
  ros::NodeHandle nh_;  // private (~) namespace
  ros::Publisher pub_map_borders_;      // /map_borders (Polygon)
  ros::Publisher pub_borders_stamped_;  // /borders (PolygonStamped)

  struct Data {
    std::string map_name;
    double dx;
    double dy;
  } data;

public:
  explicit BordersPublisher() : nh_("~")
  {
    ROS_INFO("Node created.");
    // Use latched publishers to emulate ROS2 QoS KeepLast(1) durability for static data
    pub_map_borders_     = nh_.advertise<geometry_msgs::Polygon>("/map_borders", 1, /*latch=*/true);
    pub_borders_stamped_ = nh_.advertise<geometry_msgs::PolygonStamped>("/borders", 1, /*latch=*/true);
  }

  bool configure()
  {
    ROS_INFO("Configuring node.");
     // Wait until the parameter exists
     while (!nh_.hasParam("/_/ros__parameters/map")) {
        ros::Duration(0.1).sleep();  // 100 ms
     }

    nh_.param<std::string>("/_/ros__parameters/map", data.map_name, std::string("hexagon"));
    nh_.param("/_/ros__parameters/dx", data.dx, 5.0);
    nh_.param("/_/ros__parameters/dy", data.dy, 5.0);

    if (data.map_name != "hexagon" && data.map_name != "rectangle") {
      ROS_ERROR("Map name %s not recognized", data.map_name.c_str());
      return false;
    }

    ROS_INFO("Map name: %s", data.map_name.c_str());
    ROS_INFO("dx: %f", data.dx);
    ROS_INFO("dy: %f", data.dy);

    ROS_INFO("Node configured.");
    return true;
  }

  bool activate()
  {
    ROS_INFO("Activating node.");

    std_msgs::Header hh;
    hh.stamp = ros::Time::now();
    hh.frame_id = "map";

    geometry_msgs::Polygon pol;

    if (data.map_name == "hexagon") {
      pol = create_hexagon(data.dx);
    } else if (data.map_name == "rectangle") {
      pol = create_rectangle(data.dx, data.dy);
    } else {
      ROS_ERROR("Map name %s not recognized", data.map_name.c_str());
      return false;
    }

    geometry_msgs::PolygonStamped pol_stamped;
    pol_stamped.header = hh;
    pol_stamped.polygon = pol;

    // Publish borders
    pub_map_borders_.publish(pol);
    pub_borders_stamped_.publish(pol_stamped);

    ROS_INFO("Node active.");
    return true;
  }
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "send_borders");
  BordersPublisher node;

  if (!node.configure()) {
    ROS_FATAL("Configuration failed.");
    return 1;
  }
  if (!node.activate()) {
    ROS_FATAL("Activation failed.");
    return 1;
  }

  ros::spin();
  return 0;
}
