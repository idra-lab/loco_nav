#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
from geometry_msgs.msg import Quaternion
import numpy as np


class GroundTruthOdom:
    def __init__(self):

        self.br = TransformBroadcaster()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.ROBOT_NAME = rospy.get_param('~robot_name', 'turtlebot3_burger')
        self.odom_pub = rospy.Publisher(self.ROBOT_NAME+'/odom', Odometry, queue_size=1)
    

        self.tf_rate = rospy.get_param('~rate', 30.0)
        self.publish_period = rospy.Duration(1.0 / self.tf_rate)
        self.last_pub = rospy.Time(0)
        self.last_stamp = rospy.Time(0)

    def callback(self, msg):
        try:
            idx = msg.name.index(self.ROBOT_NAME)
        except ValueError:
            rospy.logwarn(f"Robot model {self.ROBOT_NAME} not found in /gazebo/model_states")
            return

        #go at lower rate
        now = rospy.Time.now()

        # throttle
        if now - self.last_pub < self.publish_period:
            return
        self.last_pub = now

        # enforce monotonic stamps (handles sim-time repeats)
        if now <= self.last_stamp:
            now = self.last_stamp + rospy.Duration.from_sec(1e-4)
        self.last_stamp = now



        pose = msg.pose[idx]
        twist = msg.twist[idx]

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = f"{self.ROBOT_NAME}/odom"
        odom_msg.child_frame_id = f"{self.ROBOT_NAME}/base_link"

        odom_msg.pose.pose = pose
        odom_msg.twist.twist = twist

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        self.br.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            odom_msg.header.stamp,
            f"{self.ROBOT_NAME}/base_link",
            f"{self.ROBOT_NAME}/odom"
        )


if __name__ == '__main__':
    rospy.init_node('ground_truth_odom')
    gto = GroundTruthOdom()
    rospy.spin()

