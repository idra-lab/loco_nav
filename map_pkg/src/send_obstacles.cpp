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
#include <sstream>
#include <vector>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "obstacles_msgs/ObstacleArrayMsg.h"
#include "obstacles_msgs/ObstacleMsg.h"
#include "std_msgs/Header.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

class ObstaclesPublisher
{
private:
  ros::NodeHandle nh_;   // private (~) namespace
  ros::Publisher  pub_;  // /obstacles

  // Data obtained from parameters
  struct Data {
    std::vector<std::string> vect_type;
    std::vector<double> vect_x;
    std::vector<double> vect_y;
    std::vector<double> vect_yaw;
    std::vector<double> vect_dim_x;
    std::vector<double> vect_dim_y;
  } data;

  std::vector<Obstacle> gates; // kept for parity

public:
  ObstaclesPublisher() : nh_("~")
  {
    ROS_INFO("Node created.");
    pub_ = nh_.advertise<obstacles_msgs::ObstacleArrayMsg>("/obstacles", 1, /*latch=*/true);
    ros::Duration(0.3).sleep();
  }

  bool configure()
  {
    // Obstacle parameters
    // (Declare/get equivalents in ROS1)
    int n_obstacles = 3;
    bool no_cylinders = false, no_boxes = false;
    double min_size = 0.5, max_size = 1.5;

    // Wait until the parameter exists
     while (!nh_.hasParam("/_/send_obstacles/ros__parameters/n_obstacles")) {
        ros::Duration(0.1).sleep();  // 100 ms
     }

    nh_.param("/_/send_obstacles/ros__parameters/n_obstacles", n_obstacles, n_obstacles);
    nh_.param("/_/send_obstacles/ros__parameters/no_cylinders", no_cylinders, no_cylinders);
    nh_.param("/_/send_obstacles/ros__parameters/no_boxes", no_boxes, no_boxes);
    nh_.param("/_/send_obstacles/ros__parameters/min_size", min_size, min_size);
    nh_.param("/_/send_obstacles/ros__parameters/max_size", max_size, max_size);

    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_type", data.vect_type)) data.vect_type.clear();
    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_x",    data.vect_x))    data.vect_x.clear();
    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_y",    data.vect_y))    data.vect_y.clear();
    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_yaw",  data.vect_yaw))  data.vect_yaw.clear();
    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_dim_x",data.vect_dim_x))data.vect_dim_x.clear();
    if (!nh_.getParam("/_/send_obstacles/ros__parameters/vect_dim_y",data.vect_dim_y))data.vect_dim_y.clear();

    // Print parameters values
    ROS_INFO("Parameters:");
    ROS_INFO("vect_type: %s", to_string(data.vect_type).c_str());
    ROS_INFO("vect_x: %s",    to_string(data.vect_x).c_str());
    ROS_INFO("vect_y: %s",    to_string(data.vect_y).c_str());
    ROS_INFO("vect_yaw: %s",  to_string(data.vect_yaw).c_str());
    ROS_INFO("vect_dim_x: %s",to_string(data.vect_dim_x).c_str());
    ROS_INFO("vect_dim_y: %s",to_string(data.vect_dim_y).c_str());

    if (!equal_sizes({data.vect_type.size(), data.vect_x.size(), data.vect_y.size(),
                      data.vect_yaw.size(), data.vect_dim_x.size(), data.vect_dim_y.size()}))
    {
      ROS_ERROR("The number of elements in the vectors type, x (%zu), y (%zu), yaw (%zu) and the dimensions must be the same.",
                data.vect_x.size(), data.vect_y.size(), data.vect_yaw.size());
      return false;
    }

    return true;
  }

  bool activate()
  {
    ROS_INFO("Activating node send_obstacles.");

    std::vector<Obstacle> obstacles;
    obstacles.reserve(data.vect_x.size());

    for (size_t i = 0; i < data.vect_type.size(); ++i) {
      const std::string& t = data.vect_type[i];

      if (t == "cylinder") {
        Obstacle obs;
        obs.radius = data.vect_dim_x[0];   // keep parity with ROS2 code
        obs.x = data.vect_x[i];
        obs.y = data.vect_y[i];
        obs.dx = 0.0;
        obs.dy = 0.0;
        obs.yaw = 0.0;
        obs.type = OBSTACLE_TYPE::CYLINDER;
        obstacles.push_back(obs);
      } else if (t == "box") {
        Obstacle obs;
        obs.radius = 0.0;
        obs.x = data.vect_x[i];
        obs.y = data.vect_y[i];
        obs.dx = data.vect_dim_x[i];
        obs.dy = data.vect_dim_y[i];
        obs.yaw = data.vect_yaw[i];
        obs.type = OBSTACLE_TYPE::BOX;
        obstacles.push_back(obs);
      } else {
        ROS_ERROR("Type %s is not valid. It must be either cylinder or box.", t.c_str());
        return false;
      }

      // If you have valid_position(...) util, keep parity with the original (commented out)
      // if (!valid_position(this->data.map_name, this->data.dx, this->data.dy, obstacles.back(), {obstacles, this->gates})) {
      //   ROS_ERROR("The Obstacle %s is not valid.", t.c_str());
      //   return false;
      // }
    }

    ROS_INFO("Publishing %zu obstacles", obstacles.size());
    publish_obstacles(obstacles);
    return true;
  }

private:
  void publish_obstacles(std::vector<Obstacle>& obstacles)
  {
    obstacles_msgs::ObstacleArrayMsg msg;

    std_msgs::Header hh;
    hh.stamp = ros::Time::now();
    hh.frame_id = "map";
    msg.header = hh;

    for (const auto& o : obstacles) {
      ROS_INFO("Publishing Obstacle: %s x=%f, y=%f, yaw=%f, radius=%f, dx=%f, dy=%f",
               (o.type == OBSTACLE_TYPE::BOX ? "box" : "cylinder"),
               o.x, o.y, o.yaw, o.radius, o.dx, o.dy);

      obstacles_msgs::ObstacleMsg obs;
      if (o.type == OBSTACLE_TYPE::CYLINDER) {
        geometry_msgs::Point32 point;
        point.x = static_cast<float>(o.x);
        point.y = static_cast<float>(o.y);
        point.z = 0.0f;

        geometry_msgs::Polygon pol;
        pol.points.push_back(point);

        obs.polygon = pol;
        obs.radius = o.radius;
      } else {
        geometry_msgs::Polygon pol = create_rectangle(o.dx, o.dy, o.x, o.y, o.yaw);
        obs.polygon = pol;
      }
      msg.obstacles.push_back(obs);
    }

    pub_.publish(msg);
    ROS_INFO("Published %zu obstacles on /obstacles", msg.obstacles.size());
  }
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "send_obstacles");
  ObstaclesPublisher node;

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
