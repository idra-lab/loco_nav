#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include <cmath>
#include <tuple>
#include <sstream>
#include <vector>
#include "map_pkg/obstacle_struct.hpp"

// Removed rmw_qos_profile_custom – not used in ROS 1

inline bool equal_sizes (std::vector<size_t> sizes){
  for (uint i=0; i<sizes.size()-1; i++){
    for (uint j=i+1; j<sizes.size(); j++){
      if (sizes[i] != sizes[j])
        return false;
    }
  }
  return true;
}

template<typename T = double>
inline std::string to_string(std::vector<T> vect){
  std::stringstream ss;
  ss << "[";
  for (const auto& v : vect){ ss << v << ", "; }
  ss << "]";
  return ss.str();
}

using Point = std::tuple<double, double>;

std::vector<Point> create_hexagon_v(double dx);
geometry_msgs::Polygon create_hexagon(double dx);
std::vector<Point> create_rectangle_v(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);
geometry_msgs::Polygon create_rectangle(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);