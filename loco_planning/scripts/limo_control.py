# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""
from __future__ import print_function
import rospy as ros
from utils.math_tools import unwrap_angle
import numpy as np
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from  utils.common_functions import plotFrameLinear
import params as conf
import os
import rospkg
from numpy import nan
import threading
from matplotlib import pyplot as plt
from utils import limo_constants as robot_constants
from controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from utils.trajectory_generator import Trajectory, ModelsList
from utils.velocity_generator import VelocityGenerator
from termcolor import colored
from utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from utils.math_tools import euler_from_quaternion, unwrap_vector, Math
from utils.communication_utils import getInitialStateFromOdom, getInitialStateFromJoints
robotName = "limo0" # needs to inherit BaseController
import roslaunch

class GenericSimulator(threading.Thread):
    def __init__(self, robot_name="limo0"):
        print("Initialized limo controller---------------------------------------------------------------")
        threading.Thread.__init__(self)
        self.robot_name = robot_name
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.math_utils = Math()
        self.real_robot = conf.robot_params[self.robot_name].get('real_robot', False)
        self.torque_control = False

        # user config
        self.ControlType = 'CLOSED_LOOP' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.ODOMETRY = 'true' #'true',  'false' (optitrack node)
        self.SENSORS = 'false' #'true',  'false' (lidar)
        # initial pose to spawn (sim)
        self.p0 = np.array([0., 0., 0.])
        self.SAVE_BAGS = False

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.comTwistsW = np.zeros(6) 
        self.q = np.zeros(4)
        p.q_old = np.zeros(4)
        self.qd = np.zeros(4)
        self.tau = np.zeros(4)
        self.tau_fb = np.zeros(4)
        self.q_des = np.zeros(4)
        self.quaternion = np.array([0., 0., 0., 1.])  # fundamental otherwise receivepose gets stuck
        self.euler_old = np.zeros(3)
        self.q_des = np.zeros(4)
        self.qd_des = np.zeros(4)
        self.tau_ffwd = np.zeros(4)
        self.gravity_comp = np.zeros(4)
        self.b_R_w = np.eye(3)
        self.basePoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comPoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_des_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_des_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_ffwd_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.grForcesW_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.time_log = np.full((conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.constr_viol_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        self.time = np.zeros(1)
        self.loop_time = conf.robot_params[self.robot_name]['dt']
        self.log_counter = 0

        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V= 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(4)
        self.ctrl_v_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.ctrl_omega_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.v_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.omega_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_dot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.des_x = 0.
        self.des_y = 0.
        self.des_theta = 0.
        self.beta_l= 0.
        self.beta_r= 0.
        self.alpha= 0.
        self.alpha_control= 0.
        self.radius = 0.
        self.beta_l_control = 0.
        self.beta_r_control = 0.
        self.log_exy = []
        self.log_e_theta = []
        self.euler = np.zeros(3)
        self.basePoseW_des = np.zeros(6) * np.nan
        self.b_base_vel = np.zeros(2)
        self.state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.basePoseW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.b_base_vel_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.out_of_frequency_counter=0

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                self.ctrl_v_log[self.log_counter] = self.ctrl_v
                self.ctrl_omega_log[self.log_counter] = self.ctrl_omega
                self.v_d_log[self.log_counter] = self.v_d
                self.omega_d_log[self.log_counter] = self.omega_d
                self.V_log[self.log_counter] = self.V
                self.V_dot_log[self.log_counter] = self.V_dot
                self.des_state_log[0, self.log_counter] = self.des_x
                self.des_state_log[1, self.log_counter] = self.des_y
                self.des_state_log[2, self.log_counter] = self.des_theta
                self.state_log[0, self.log_counter] = self.basePoseW[0]
                self.state_log[1, self.log_counter] = self.basePoseW[1]
                self.state_log[2, self.log_counter] =  self.basePoseW[5]

                self.basePoseW_des_log[:, self.log_counter] = self.basePoseW_des #basepose is logged in base controller
                self.b_base_vel_log[:, self.log_counter] = self.b_base_vel  # basepose is logged in base controller

                self.basePoseW_log[:, self.log_counter] = self.basePoseW
                self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
                self.q_des_log[:, self.log_counter] = self.q_des
                self.q_log[:, self.log_counter] = self.q
                self.qd_des_log[:, self.log_counter] = self.qd_des
                self.qd_log[:, self.log_counter] = self.qd
                self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
                self.tau_log[:, self.log_counter] = self.tau

                self.time_log[self.log_counter] = self.time
                self.log_counter += 1


    def startFramework(self):
        if self.real_robot:
            # 1) Set ROS_IP using the HOSTCOMPUTER environment variable
            host = os.environ.get("HOSTCOMPUTER_IP")
            if host is None:
                raise RuntimeError("HOSTCOMPUTER is not set in your environment!")
            os.environ["ROS_IP"] = host
            print("ROS_IP set to:", os.environ["ROS_IP"])
        else:
            # Unset ROS_IP
            os.environ.pop("ROS_IP", None)
            print("ROS_IP unset (simulation mode)")

        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")

        self.decimate_publish = 1
        world_name = None #'ramps.world'
        additional_args = ['spawn_x:=' + str(p.p0[0]),'spawn_y:=' + str(p.p0[1]),'spawn_Y:=' + str(p.p0[2]),'sensors:='+self.SENSORS, 'odometry:='+self.ODOMETRY]
        launch_file = rospkg.RosPack().get_path('limo_description') + '/launch/start_robot.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file,
                    'robot_name:=' + self.robot_name,
                    'real_robot:=' + str(self.real_robot)]
        if world_name is not None:
            print(colored("Setting custom model: " + str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))
        if additional_args is not None:
            cli_args.extend(additional_args)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadPublishers(self):
        ros.init_node(f'limo_control_node', anonymous=False, log_level=ros.FATAL)
        # instantiating objects
        #self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.cmd_vel_pub = ros.Publisher("/"+self.robot_name+"/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)

        if self.SAVE_BAGS:
            bag_name = f"log.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)


    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def _receive_pose(self, msg):
        self.quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.euler = np.array(euler_from_quaternion(self.quaternion))
        #unwrap
        self.euler, self.euler_old = unwrap_vector(self.euler, self.euler_old)

        self.basePoseW[0] = msg.pose.pose.position.x
        self.basePoseW[1] = msg.pose.pose.position.y
        self.basePoseW[2] = msg.pose.pose.position.z
        self.basePoseW[3] = self.euler[0]
        self.basePoseW[4] = self.euler[1]
        self.basePoseW[5] = self.euler[2]

        self.baseTwistW[0] = msg.twist.twist.linear.x
        self.baseTwistW[1] = msg.twist.twist.linear.y
        self.baseTwistW[2] = msg.twist.twist.linear.z
        self.baseTwistW[3] = msg.twist.twist.angular.x
        self.baseTwistW[4] = msg.twist.twist.angular.y
        self.baseTwistW[5] = msg.twist.twist.angular.z

        # compute orientation matrix
        self.b_R_w = self.math_utils.rpyToRot(self.euler)

    def checkLoopFrequency(self):
        # check frequency of publishing
        if hasattr(self, 'check_time'):
            loop_time = ros.Time.now().to_sec() - self.check_time  # actual publishing time interval
            ros_loop_time = self.slow_down_factor * conf.robot_params[p.robot_name]['dt'] * self.decimate_publish  # ideal publishing time interval
            if loop_time > 1.3 * (ros_loop_time):
                loop_real_freq = 1 / loop_time  # actual publishing frequency
                freq_ros = 1 / ros_loop_time  # ideal publishing frequency
                print(colored(f"freq mismatch beyond 30%: loop is running at {loop_real_freq} Hz while it should run at {freq_ros} Hz, freq error is {(freq_ros - loop_real_freq) / freq_ros * 100} %", "red"))
                self.out_of_frequency_counter += 1
                if self.out_of_frequency_counter > 10:
                    original_slow_down_factor = self.slow_down_factor
                    self.slow_down_factor *= 2
                    self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))
                    print(colored(f"increasing slow_down_factor from {original_slow_down_factor} to {self.slow_down_factor}", "red"))
                    self.out_of_frequency_counter = 0

        self.check_time = ros.Time.now().to_sec()

    def deregister_node(self):
        print( "deregistering nodes"     )
        os.system(" rosnode kill /gazebo")
        os.system("pkill rosmaster")
        os.system("killall rosmaster gzserver gzclient rviz coppeliaSim")


    def startupProcedure(self):
        self.slow_down_factor = 1
        # loop frequency
        self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))

    def plotData(self):
        if conf.plotting:
            #xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :], p.des_state_log[1, :], "-r", label="desired")
            plt.plot(p.state_log[0, :], p.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.title(f"XY plot: {p.ControlType}")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            # # command plot
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.ctrl_v_log, "-b", label="REAL")
            plt.plot(p.time_log, p.v_d_log, "-r", label="desired")
            plt.legend()
            plt.title("v and omega")
            plt.ylabel("command linear velocity[m/s]")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.ctrl_omega_log, "-b", label="REAL")
            plt.plot(p.time_log, p.omega_d_log, "-r", label="desired")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("command angular velocity[rad/s]")
            plt.grid(True)

            #joint velocities with limits
            fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 8))  # Create all 3 subplots at once
            axs[0].plot(p.time_log, p.qd_log[0, :], "-b", linewidth=3)
            axs[0].plot(p.time_log, p.qd_des_log[0, :], "-r", linewidth=4)
            axs[0].plot(p.time_log, robot_constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[0].plot(p.time_log, -robot_constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[0].set_ylabel("WHEEL_L")
            axs[0].grid(True)
            axs[1].plot(p.time_log, p.qd_log[1, :], "-b", linewidth=3)
            axs[1].plot(p.time_log, p.qd_des_log[1, :], "-r", linewidth=4)
            axs[1].plot(p.time_log, robot_constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[1].plot(p.time_log, -robot_constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[1].set_ylabel("WHEEL_R")
            axs[1].grid(True)

            plt.xlabel("Time [s]")
            plt.tight_layout()
            plt.show()

            #states plot
            plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log, custom_labels=(["X","Y","THETA"]))

            #plot velocities in the base frame
            # plt.figure()
            # ax1 = plt.subplot(2, 1, 1)
            # plt.plot(self.time_log, self.b_base_vel_log[0, :], "-b", label="vx")
            # plt.ylabel("b_vx")
            # plt.legend()
            # plt.grid(True)
            # plt.subplot(2, 1, 2, sharex=ax1)
            # plt.plot(self.time_log, self.b_base_vel_log[1, :], "-b", label="vy")
            # plt.ylabel("b_vy")
            # plt.legend()
            # plt.grid(True)
            if p.ControlType != 'OPEN_LOOP':
                # tracking errors
                p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(np.sqrt(np.power(self.log_e_x,2) +np.power(self.log_e_y,2)), "-b")
                plt.ylabel("exy")
                plt.title("tracking errors")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(self.log_e_theta, "-b")
                plt.ylabel("eth")
                plt.grid(True)

    def mapFromWheels(self, wheel_l, wheel_r):
        if not np.isscalar(wheel_l):
            v = np.zeros_like(wheel_l)
            omega = np.zeros_like(wheel_l)
            for i in range(len(wheel_l)):
                v[i] = robot_constants.SPROCKET_RADIUS*(wheel_l[i] + wheel_r[i])/2
                omega[i] = robot_constants.SPROCKET_RADIUS/robot_constants.TRACK_WIDTH*(wheel_r[i] -wheel_l[i])
            return v, omega

    def mapToWheels(self, v_des,omega_des):
        qd_des = np.zeros(4)
        qd_des[0] = (v_des - omega_des * robot_constants.TRACK_WIDTH / 2) / robot_constants.SPROCKET_RADIUS  # left front
        qd_des[1] = (v_des + omega_des * robot_constants.TRACK_WIDTH / 2) / robot_constants.SPROCKET_RADIUS  # right front
        qd_des[2] = qd_des[0].copy()
        qd_des[3] = qd_des[1].copy()
        return qd_des

    def publishControlCommand(self, v_des,omega_des):
        msg = Twist()
        msg.linear.x = v_des
        msg.angular.z = omega_des
        self.cmd_vel_pub.publish(msg)

    #unwrap the joints states
    def unwrap(self):
        for i in range(4):
            self.q[i], self.q_old[i] =unwrap_angle(self.q[i], self.q_old[i])

    def generateWheelTraj(self, wheel_l = -4.5):
        ####################################
        # OPEN LOOP wl , wr (from -IDENT_MAX_WHEEL_SPEED to IDENT_MAX_WHEEL_SPEED)
        ####################################
        wheel_l_vec = []
        wheel_r_vec = []
        change_interval = 0.5
        if wheel_l <= 0.: #this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-self.IDENT_MAX_WHEEL_SPEED, self.IDENT_MAX_WHEEL_SPEED, 32) #it if passes from 0 for some reason there is a non linear
                #behaviour in the long slippage
        else:
            wheel_r =np.linspace(self.IDENT_MAX_WHEEL_SPEED, -self.IDENT_MAX_WHEEL_SPEED, 32)
        time = 0
        i = 0
        while True:
            time = np.round(time + conf.robot_params[p.robot_name]['dt'], 4)
            wheel_l_vec.append(wheel_l)
            wheel_r_vec.append(wheel_r[i])
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(wheel_r):
                break
        wheel_l_vec.append(0.0)
        wheel_r_vec.append(0.0)
        return wheel_l_vec,wheel_r_vec

    def generateOmegaTraj(self, omega_initial=0.51, omega_final=0.21, increment=0.3,  dt = 0.005, long_v = 0.1, direction="left"):
        # only around 0.3
        change_interval = 1.
        increment = increment
        ang_w_vec = np.arange(omega_initial, omega_final,increment)
        if direction == 'right':
            ang_w_vec *= -1

        omega_vec = []
        v_vec = []
        time = 0
        i = 0

        long_v_rampup = np.linspace(0, long_v, 100)
        for ramp_count in range(100):
            time = np.round(time + dt, 3)
            omega_vec.append(0)
            v_vec.append(long_v_rampup[ramp_count])
        while True:
            time = np.round(time + dt, 3)
            omega_vec.append(ang_w_vec[i])
            v_vec.append(long_v)
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(ang_w_vec):
                break
        v_vec.append(0.0)
        omega_vec.append(0.0)
        return v_vec, omega_vec


    
    def monitor_time(self):
        self.checkLoopFrequency()
        if np.mod(self.time,1) == 0:
            print(colored(f"TIME: {self.time}","red"))

    def initSubscribers(self):
        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)
        self.sub_pose_limo = ros.Subscriber("/" + self.robot_name + "/odom", Odometry, callback=self._receive_pose,
                                            queue_size=1, tcp_nodelay=True)

        if self.real_robot:
            print(colored("IMPORTANT: Real robot ON,  be sure param use_sim_time = false","red"))
            # for limo the publisher in on limo0/odom not groundtruth
            self.p0[0], self.p0[1], self.p0[2] = getInitialStateFromOdom(self.robot_name)
            self.q_des = getInitialStateFromJoints(robot_name=self.robot_name, joint_names=self.joint_names)

def talker(p):
    p.start()

    p.startFramework()
    p.loadPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()

    robot_state = Robot()
    if p.real_robot:
        ros.sleep(2.)
    if p.SAVE_BAGS:
        p.recorder.start_recording_srv()

    # OPEN loop control
    if p.ControlType == 'OPEN_LOOP':
        counter = 0
        # generic open loop test for comparison with matlab
        v_ol = np.linspace(0.5, 0.5, np.int32(10./conf.robot_params[p.robot_name]['dt']))
        omega_ol = np.linspace(0., 0., np.int32(10./conf.robot_params[p.robot_name]['dt']))
        traj_length = len(v_ol)
        p.des_x = p.p0[0]  # +0.1
        p.des_y = p.p0[1]  # +0.1
        p.des_theta = p.p0[2]  # +0.1
        p.traj = Trajectory(ModelsList.UNICYCLE, p.des_x, p.des_y, p.des_theta, DT=conf.robot_params[p.robot_name]['dt'], v=v_ol, omega=omega_ol)

        while not ros.is_shutdown():
            _, _, _, p.v_d, p.omega_d, _, _, traj_finished = p.traj.evalTraj(p.time)
            p.qd_des = p.mapToWheels(p.v_d, p.omega_d)
            p.publishControlCommand(p.v_d, p.omega_d)
            if traj_finished:
                break
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, _ = p.traj.evalTraj(p.time)
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.monitor_time()
            #p.ros_pub.publishVisual(delete_markers=False)

            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  4)  # to avoid issues of dt 0.0009999
    else:
        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=10.,    DT=conf.robot_params[p.robot_name]['dt'])
        p.des_x = p.p0[0]
        p.des_y = p.p0[1]
        p.des_theta = p.p0[2]
        v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_chicane(v_max_=0.5, omega_max_=0.7)
        p.traj = Trajectory(ModelsList.UNICYCLE, start_x=p.des_x, start_y=p.des_y, start_theta=p.des_theta, DT=conf.robot_params[p.robot_name]['dt'],
                            v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)

        # Lyapunov controller parameters
        params = LyapunovParams(K_P=1., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt']) #high gains 15 5 / low gains 10 1 (default)
        p.controller = LyapunovController(params=params)#, matlab_engine = p.eng)

        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[0]
            robot_state.y = p.basePoseW[1]
            robot_state.theta = p.basePoseW[5]
            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, traj_finished = p.traj.evalTraj(p.time)
            if traj_finished:
                break
            p.ctrl_v, p.ctrl_omega  = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)
            #compute qd_des after control computation
            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

             # send command
            p.publishControlCommand(p.ctrl_v, p.ctrl_omega)

            #recompute qd_des after long slippage compensation for plot
            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            p.monitor_time()
            #p.ros_pub.publishVisual(delete_markers=False)

            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999

    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
    ros.signal_shutdown("killed")
    p.deregister_node()
    print("Plotting")
    p.plotData()


