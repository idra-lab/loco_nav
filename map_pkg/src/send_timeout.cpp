// -*- coding: utf-8 -*-
#include <ros/ros.h>

#include "std_msgs/Int32.h"

#include <string>

#include "map_pkg/utilities.hpp"

using MsgType = std_msgs::Int32;

/**
 * @brief Class to publish the timeout of the victims in seconds.
 */
class SendTimeout
{
private:
  int32_t timeout_;
  ros::NodeHandle nh_;                 // private (~) namespace
  ros::Publisher publisher_;           // /victims_timeout

public:
  explicit SendTimeout() : timeout_(0), nh_("~") {}

  bool configure()
  {
    ros::Duration(1.0).sleep();  // sleep for 1 seconds to wait map is generated
     // Wait until the parameter exists
     while (!nh_.hasParam("/_/send_timeout/ros__parameters/victims_timeout")) {
        ros::Duration(0.1).sleep();  // 100 ms
     }
    nh_.param("/_/send_timeout/ros__parameters/victims_timeout", timeout_, 0);

    // if (this->timeout_ == 0){
    //   this->find_timeout();
    // }

    // Latched publisher to emulate ROS2 QoS KeepLast(1) for a one-shot value
    publisher_ = nh_.advertise<MsgType>("/victims_timeout", 1, /*latch=*/true);

    ROS_INFO("send_timeout configured with timeout %d", timeout_);
    return true;
  }

  bool activate()
  {
    ROS_INFO("Activating node send_timeout.");
    publish_timeout();
    ROS_INFO("Timeout published, node activated.");
    return true;
  }

  bool deactivate()
  {
    ROS_INFO("Deactivating node send_timeout.");
    return true;
  }

  bool cleanup()
  {
    ROS_INFO("Cleaning up node send_timeout.");
    return true;
  }

  bool shutdown()
  {
    ROS_INFO("Shutting down node send_timeout.");
    return true;
  }

private:
  void publish_timeout()
  {
    MsgType msg;
    msg.data = timeout_;
    publisher_.publish(msg);
  }

  void find_timeout()
  {
    timeout_ = 10;
  }
};


int main (int argc, char * argv[])
{
  ros::init(argc, argv, "send_timeout");

  SendTimeout node;
  if (!node.configure()) return 1;
  if (!node.activate())  return 1;

  // Keep alive so late subscribers get the latched message
  ros::spin();
  return 0;
}
