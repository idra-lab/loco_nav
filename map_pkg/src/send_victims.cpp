// -*- coding: utf-8 -*-
#include <ros/ros.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <fstream>
#include <vector>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

/*
 * @brief This class is responsible for generating and publishing victims in the map.
 * It mirrors the ROS2 Lifecycle node semantics with configure()/activate() methods.
 */
class VictimPublisher
{
private:
  ros::NodeHandle nh_;  // private namespace (~)
  ros::Publisher victims_pub_;
  ros::Publisher markers_pub_;

  struct Data {
    double max_weight;
    double min_weight;
    std::vector<double> vect_x;
    std::vector<double> vect_y;
    std::vector<double> vect_weight;
  } data;

public:
  VictimPublisher() : nh_("~")
  {
    ROS_INFO("Node created.");
  }

  bool configure()
  {
    ROS_INFO("Configuring node.");

    // Victims parameters
 // Wait until the parameter exists
     while (!nh_.hasParam("/_/send_victims/ros__parameters/max_weight")) {
        ros::Duration(0.1).sleep();  // 100 ms
     }
    nh_.param("/_/send_victims/ros__parameters/max_weight", data.max_weight, 100.0);
    nh_.param("/_/send_victims/ros__parameters/min_weight", data.min_weight, 1.0);
    if (!nh_.getParam("/_/send_victims/ros__parameters/vect_x", data.vect_x)) data.vect_x.clear();
    if (!nh_.getParam("/_/send_victims/ros__parameters/vect_y", data.vect_y)) data.vect_y.clear();
    if (!nh_.getParam("/_/send_victims/ros__parameters/vect_weight", data.vect_weight)) data.vect_weight.clear();

    // Print parameters values
    ROS_INFO("max_weight: %f", data.max_weight);
    ROS_INFO("min_weight: %f", data.min_weight);
    ROS_INFO("vect_x: %s", to_string(data.vect_x).c_str());
    ROS_INFO("vect_y: %s", to_string(data.vect_y).c_str());
    ROS_INFO("vect_weight: %s", to_string(data.vect_weight).c_str());

    if (!equal_sizes({data.vect_x.size(), data.vect_y.size(), data.vect_weight.size()})) {
      ROS_ERROR("vect_x and vect_y and vect_weigths must have the same size.");
      return false;
    }

    // Create publishers (queue size 1, non-latched to mirror ROS2 default)
    victims_pub_ = nh_.advertise<obstacles_msgs::ObstacleArrayMsg>("/victims", 1, true);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers/victims", 1, true);

    return true;
  }

  bool activate()
  {
    // Build victims message
    obstacles_msgs::ObstacleArrayMsg msg;
    std_msgs::Header hh;
    hh.stamp = ros::Time::now();
    hh.frame_id = "map";
    // (ROS2 code didn’t assign msg.header = hh; we keep parity.)

    for (size_t i = 0; i < data.vect_x.size(); ++i) {
      Victim vict(data.vect_x[i], data.vect_y[i], data.vect_weight[i]);
      publish_victim(vict, msg);
    }

    publish_victims(msg);
    return true;
  }

private:
  void rand_victims(std::vector<Obstacle>& victims); // declared for parity

  void publish_victim(const Victim& vict, obstacles_msgs::ObstacleArrayMsg& msg)
  {
    ROS_INFO("Adding victim at x=%f, y=%f", vict.x, vict.y);

    obstacles_msgs::ObstacleMsg obs;
    geometry_msgs::Polygon pol;

    geometry_msgs::Point32 point;
    point.x = static_cast<float>(vict.x);
    point.y = static_cast<float>(vict.y);
    point.z = 0.0f;
    pol.points.push_back(point);

    obs.polygon = pol;

    // While physically the radius won't be touched, the assigned value indicates the weight
    obs.radius = vict.radius;

    msg.obstacles.push_back(obs);
  }

  void publish_victims(const obstacles_msgs::ObstacleArrayMsg& msg)
  {
    ROS_INFO("[1] Publishing victims.");
    visualization_msgs::MarkerArray markers;

    for (size_t vict_id = 0; vict_id < msg.obstacles.size(); ++vict_id) {
      std_msgs::Header hh = msg.header;
      geometry_msgs::Pose pose;
      const geometry_msgs::Point32& point = msg.obstacles[vict_id].polygon.points[0];
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = point.y;  // keep parity with original
      double radius = msg.obstacles[vict_id].radius;

      // Cylinder marker for victim position
      visualization_msgs::Marker pos_marker;
      pos_marker.header = hh;
      pos_marker.header.frame_id = "map";
      pos_marker.ns = "victims";
      pos_marker.id = static_cast<int>(vict_id);
      pos_marker.action = visualization_msgs::Marker::ADD;
      pos_marker.type = visualization_msgs::Marker::CYLINDER;
      pos_marker.pose = pose;
      pos_marker.scale.x = radius / data.max_weight * 1.0;
      pos_marker.scale.y = radius / data.max_weight * 1.0;
      pos_marker.scale.z = 0.1;
      pos_marker.color.a = 0.5;
      pos_marker.color.r = 0.0;
      pos_marker.color.g = 0.0;
      pos_marker.color.b = 1.0;
      markers.markers.push_back(pos_marker);

      // Text marker for victim weight
      visualization_msgs::Marker weight_marker;
      weight_marker.header = hh;
      weight_marker.header.frame_id = "map";
      weight_marker.ns = "victims_weight";
      weight_marker.id = static_cast<int>(vict_id);
      weight_marker.action = visualization_msgs::Marker::ADD;
      weight_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      weight_marker.pose = pose;
      weight_marker.scale.z = 0.9;
      weight_marker.color.a = 1.0;
      weight_marker.color.r = 1.0;
      weight_marker.color.g = 0.5;
      weight_marker.color.b = 1.0;
      weight_marker.text = std::to_string(static_cast<int>(radius));
      markers.markers.push_back(weight_marker);
    }

    victims_pub_.publish(msg);
    ROS_INFO("[2] Victims published, publishing markers.");
    markers_pub_.publish(markers);
    ROS_INFO("[3] Markers published.");
  }
};


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "victims_sender");
  VictimPublisher node;

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
