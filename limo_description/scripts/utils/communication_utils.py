from termcolor import colored
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy as ros

def getInitialStateFromOdom(robot_name = None):
    try:
        odom0 = ros.wait_for_message("/" + robot_name + "/odom", Odometry, timeout=10.0)
    except ros.ROSException:
        ros.logerr(f"Timed out waiting for /{robot_name}/odom")
        return

    p0 = odom0.pose.pose.position
    q0 = odom0.pose.pose.orientation
    yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])[2]
    print(colored(f"{robot_name}: Init. desired state from first /odom: x0: {p0.x},y0: {p0.y},yaw0: {yaw0}", "red"))

    return p0.x, p0.y, yaw0