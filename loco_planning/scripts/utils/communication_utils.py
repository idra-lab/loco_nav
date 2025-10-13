from termcolor import colored
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy as ros
import subprocess, roslaunch, rospkg, rosnode,rosgraph
from roslaunch.parent import ROSLaunchParent
import sys, os
import numpy as np

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


def checkRosMaster():
    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        print(colored('ROS MASTER is Online','red'))
    else:
        print(colored('ROS MASTER is NOT Online, Starting roscore!!','red'))
        parent = ROSLaunchParent("roscore", [], is_core=True)  # run_id can be any string
        parent.start()

def launchFileNode(package,launch_file, additional_args=None):
    launch_file = rospkg.RosPack().get_path(package) + '/launch/'+launch_file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = [launch_file]
    if additional_args is not None:
        cli_args.extend(additional_args)

    roslaunch_args = cli_args[1:]
    print(colored(f"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA {roslaunch_args}", "red"))
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

def launchFileGeneric(launch_file):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

def startNode(package, executable, args=''):
    nodes = rosnode.get_node_names()
    #kill previous instances
    if package in nodes:
        print(colored("Re Starting ref generator","red"))
        os.system("rosnode kill /"+package)
    package = package
    executable = executable
    name = package
    namespace = ''
    node = roslaunch.core.Node(package, executable, name, namespace, args=args, output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)


def loadXacro(package_name, model_name):
    print(colored(f"Loading xacro for  {model_name} inside {package_name}", "blue"))
    # first generate robot description
    xacro_path = rospkg.RosPack().get_path(package_name) + '/robots/' + model_name + '.urdf.xacro'
    if not os.path.isfile(xacro_path):
        print(colored(f"Xacro file {model_name}.urdf.xacro does not exist!", "red"))
    command_string = "rosrun xacro xacro "+xacro_path

    try:
        robot_description_param = subprocess.check_output(command_string,shell=True,  stderr=subprocess.STDOUT).decode("utf-8") # shell=True is fundamental to load env variables!
    except subprocess.CalledProcessError as process_error:
        ros.logfatal('Failed to run xacro command with error: \n%s', process_error.output)
        sys.exit(1)

    # put on param server
    ros.set_param('/'+model_name, robot_description_param)

def spawnModel(package_name, model_name='',  spawn_pos=np.array([0.,0.,0.]), spawn_orient = np.array([0.,0.,0.]) ):
    #loads the xacro of model in the parameter server
    loadXacro(package_name, model_name)
    print(colored(f"Spawning {model_name}", "blue"))
    package = 'gazebo_ros'
    executable = 'spawn_model'
    name = model_name
    namespace = '/'
    args = '-urdf -param ' +model_name +' -model ' + model_name +' -x '+ str(spawn_pos[0])+ ' -y ' + str(spawn_pos[1]) +' -z ' + str(spawn_pos[2]) \
           + ' -R ' + str(spawn_orient[0]) + ' -P ' + str(spawn_orient[1]) + ' -Y ' + str(spawn_orient[2])
    node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
