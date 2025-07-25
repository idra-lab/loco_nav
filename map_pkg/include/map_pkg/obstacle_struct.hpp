#pragma once

#include <ostream>
#include <string>
#include <vector>

enum OBSTACLE_TYPE {
  CYLINDER,
  BOX
};

struct Obstacle {
  double radius = 0.0;
  double x = 0.0, y = 0.0;
  double dx = 0.0, dy = 0.0;
  double yaw = 0.0;
  OBSTACLE_TYPE type = CYLINDER;
  std::string xml_file = "";

  friend std::ostream& operator<<(std::ostream& os, const Obstacle& obs) {
    os << "Obstacle: ";
    if (obs.type == OBSTACLE_TYPE::CYLINDER) {
      os << "Cylinder: ";
      os << "x: " << obs.x << " y: " << obs.y << " radius: " << obs.radius;
    } else if (obs.type == OBSTACLE_TYPE::BOX) {
      os << "Box: ";
      os << "x: " << obs.x << " y: " << obs.y << " dx: " << obs.dx << " dy: " << obs.dy << " yaw: " << obs.yaw;
    } else {
      os << "Unknown type";
    }
    return os;
  }
};

struct Victim : public Obstacle {
  Victim(double x = 0, double y = 0) {
    this->x = x;
    this->y = y;
    this->type = CYLINDER;
  }
};

