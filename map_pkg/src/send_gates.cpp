// -*- coding: utf-8 -*-
#include <ros/ros.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <random>
#include <vector>

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

class GatesPublisher
{
private:
  ros::NodeHandle nh_;                 // private (~) namespace
  ros::Publisher  publisher_;          // /gates publisher (latched)

  struct Data {
    std::string map_name;
    double dx;
    double dy;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> th;
  } data;

public:
  explicit GatesPublisher()
  : nh_("~")
  {
    ROS_INFO("Node created.");
    // Latched publisher to mimic ROS2 QoS durability/transient_local for one-shot publish
    publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/gates", 1, /*latch=*/true);
  }

  bool configure()
  {
    ROS_INFO("Configuring node.");
   // Wait until the parameter exists
     while (!nh_.hasParam("/_/ros__parameters/map")) {
        ros::Duration(0.1).sleep();  // 100 ms
     }
    // Map parameters
    nh_.param<std::string>("/_/ros__parameters/map", data.map_name, std::string("hexagon"));
    nh_.param("/_/ros__parameters/dx", data.dx, 10.0);
    nh_.param("/_/ros__parameters/dy", data.dy, 10.0);

    // Gate parameters
    if (!nh_.getParam("/_/send_gates/ros__parameters/x", data.x)) data.x.clear();
    if (!nh_.getParam("/_/send_gates/ros__parameters/y", data.y)) data.y.clear();
    if (!nh_.getParam("/_/send_gates/ros__parameters/yaw", data.th)) data.th.clear();

    if (data.map_name != "hexagon" && data.map_name != "rectangle"){
      ROS_ERROR("Map parameter must be either hexagon or rectangle.");
      return false;
    }

    if (data.x.size() != data.y.size() || data.x.size() != data.th.size()){
      ROS_ERROR("The number of x, y, and th must be the same.");
      return false;
    }

    for (size_t i = 0; i < data.x.size(); ++i){
      ROS_INFO("Gate %zu: x=%f, y=%f, th=%f", i, data.x[i], data.y[i], data.th[i]);
    }

    ROS_INFO("Node configured.");
    return true;
  }

  bool activate()
  {
    ROS_INFO("Activating node.");

    std::vector<geometry_msgs::Pose> pose_array_temp;
    pose_array_temp.reserve(data.x.size());

    for (size_t i = 0; i < data.x.size(); ++i){
      publish_gate(data.x[i], data.y[i], data.th[i], pose_array_temp);
    }

    geometry_msgs::PoseArray msg;
    // Set headers of messages
    std_msgs::Header hh;
    hh.stamp = ros::Time::now();
    hh.frame_id = "map";
    msg.header = hh;

    // Add gates to the message
    msg.poses = pose_array_temp;

    // Publish message (latched)
    publisher_.publish(msg);

    ROS_INFO("Node activated.");
    return true;
  }

private:
  void publish_gate(double x, double y, double th, std::vector<geometry_msgs::Pose>& pose_array_temp)
  {
    geometry_msgs::Pose pose;

    // Set position of the gate
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.05;

    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    pose_array_temp.push_back(pose);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "send_gates");
  GatesPublisher node;

  if (!node.configure()) {
    ROS_FATAL("Configuration failed.");
    return 1;
  }
  if (!node.activate()) {
    ROS_FATAL("Activation failed.");
    return 1;
  }

  // Keep the node alive so latched topic is available to late subscribers
  ros::spin();
  return 0;
}
