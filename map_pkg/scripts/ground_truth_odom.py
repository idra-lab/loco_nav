#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
from geometry_msgs.msg import Quaternion

ROBOT_NAME = rospy.get_param('~robot_name', 'turtlebot3_burger')


class GroundTruthOdom:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.br = TransformBroadcaster()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

    def callback(self, msg):
        try:
            idx = msg.name.index(ROBOT_NAME)
        except ValueError:
            rospy.logwarn("Robot model not found in /gazebo/model_states")
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose = pose
        odom_msg.twist.twist = twist

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        self.br.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            odom_msg.header.stamp,
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    rospy.init_node('ground_truth_odom')
    gto = GroundTruthOdom()
    rospy.spin()

