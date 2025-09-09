#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

import matplotlib
matplotlib.use('TkAgg')
import rospy as ros
import numpy as np
from utils.velocity_generator import VelocityGenerator
from utils.trajectory_generator import Trajectory, ModelsList
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from limo_description.msg import Reference
from tf.transformations import euler_from_quaternion
from utils.math_tools import unwrap_vector, unwrap_angle
from matplotlib import pyplot as plt
from termcolor import colored
from controllers.lyapunov import LyapunovController, LyapunovParams, Robot
import params as conf
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import argparse
import sys
from utils.communication_utils import getInitialStateFromOdom

class Controller():

    def __init__(self, robot_name="limo1", debug=False):
        self.robot_name = robot_name
        self.DEBUG = debug

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.1
        self.omega_d = 0

        self.quaternion = np.array([0., 0., 0., 1.])# fundamental otherwise receivepose gets stuck
        self.euler_old = np.zeros(3)# fundamental otherwise receivepose gets stuck
        self.old_theta = 0
        self.time = np.zeros(1)
        self.log_counter = 0
        # log vars
        self.basePoseW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.time_log = np.full(
            (conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        self.state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        p0_x, p0_y, yaw0 = getInitialStateFromOdom(self.robot_name)
        #initialize with actual state
        self.des_x = p0_x
        self.des_y = p0_y
        self.des_theta = yaw0

        vel_gen = VelocityGenerator(simulation_time=20., DT=conf.robot_params[self.robot_name]['dt'])
        v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.1, omega_max_=0.05)
        self.trajectory = Trajectory(ModelsList.UNICYCLE, self.des_x, self.des_y, self.des_theta, DT=conf.robot_params[self.robot_name]['dt'],
                               v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)

    def startPublisherSubscribers(self):

        self.command_pub = ros.Publisher(
            "/" + self.robot_name + "/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)

        self.sub_odom = ros.Subscriber("/" + self.robot_name + "/odom", Odometry, callback=self.receive_pose,
                                       queue_size=1, tcp_nodelay=True)
        self.sub_reference = ros.Subscriber("/" + self.robot_name + "/ref", Reference, callback=self.receive_reference,
                                       queue_size=1, tcp_nodelay=True)

    def start_controller(self):
        ros.init_node(f'{self.robot_name}_controller', anonymous=False, log_level=ros.FATAL)
        ros.on_shutdown(self.on_shutdown)
        self.startPublisherSubscribers()
        self.initVars()
        # Lyapunov controller parameters
        params = LyapunovParams(K_P=conf.robot_params[self.robot_name]['k_p'], K_THETA=conf.robot_params[self.robot_name]['k_th'], DT=conf.robot_params[self.robot_name]['dt'])
        self.controller = LyapunovController(params=params)
        self.robot_state = Robot()
        rate = ros.Rate(1/conf.robot_params[self.robot_name]['dt'])  # 100Hz loop, adjust as needed

        while not ros.is_shutdown():
            try:
                # update kinematics
                self.robot_state.x = self.basePoseW[0]
                self.robot_state.y = self.basePoseW[1]
                self.robot_state.theta = self.basePoseW[5]
                # print(f"pos X: {self.robot_state.x} Y: {self.robot_state.y} th: {self.robot_state.theta}")

                if self.DEBUG:
                    self.des_x, self.des_y, self.des_theta, self.v_d, self.omega_d, self.v_dot_d, self.omega_dot_d, traj_finished = self.trajectory.evalTraj(self.time)
                    if traj_finished:
                        break
                #print(f"{self.robot_name} des_x: {self.des_x}, des_y: {self.des_y}")

                self.des_theta, self.old_theta = unwrap_angle(self.des_theta, self.old_theta)
                self.ctrl_v, self.ctrl_omega  = self.controller.control_unicycle(self.robot_state, self.time, self.des_x, self.des_y, self.des_theta, self.v_d, self.omega_d, False)

                self.send_commands(self.ctrl_v, self.ctrl_omega)
                self.logData()
                # wait for synconization of the control loop
                rate.sleep()
                # to avoid issues of dt 0.0009999
                self.time = np.round(self.time + np.array([conf.robot_params[self.robot_name]['dt']]),  4)

                if self.DEBUG:
                    if self.time == 20:
                        print("Ending simulation...")
                        self.plotData()
                        # send zero
                        self.send_commands(0., 0.)
                        break

            except (ros.ROSInterruptException, ros.service.ServiceException):
                self.send_commands(0., 0.)
                self.plotData()
                break



    def receive_reference(self, msg : Reference):
        # if np.linalg.norm(np.array([msg.x_d, msg.y_d]) - np.array([self.robot_state.x,self.robot_state.y])) > 1.0:
        #     print(colored(f"Received reference {self.robot_name}:{self.des_x},{self.des_y} is too far from actual state, negleting it", "red"))
        #     return
        #
        if msg.plan_finished:
            self.plotData()
        else:
            self.des_x = msg.x_d
            self.des_y = msg.y_d
            self.des_theta = msg.theta_d
            self.v_d = msg.v_d
            self.omega_d = msg.omega_d
            print(colored(f"received {self.robot_name} des_x: {self.des_x}, des_y: {self.des_y}, des_theta: {self.des_theta}, des_v: {self.v_d}, des_omega: {self.omega_d}", "red"))


    def receive_pose(self, msg):
        self.quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.euler = np.array(euler_from_quaternion(self.quaternion))
        #unwrap
        self.euler, self.euler_old = unwrap_vector(self.euler, self.euler_old)
        #
        self.basePoseW[0] = msg.pose.pose.position.x
        self.basePoseW[1] = msg.pose.pose.position.y
        self.basePoseW[2] = msg.pose.pose.position.z
        self.basePoseW[3] = self.euler[0]
        self.basePoseW[4] = self.euler[1]
        self.basePoseW[5] = self.euler[2]
        #
        self.baseTwistW[0] = msg.twist.twist.linear.x
        self.baseTwistW[0] = msg.twist.twist.linear.y
        self.baseTwistW[0] = msg.twist.twist.linear.z
        self.baseTwistW[3] = msg.twist.twist.angular.x
        self.baseTwistW[4] = msg.twist.twist.angular.y
        self.baseTwistW[5] = msg.twist.twist.angular.z

    def send_commands(self, liv_vel, ang_vel):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = Twist()
        msg.linear.x = liv_vel
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = ang_vel
        self.command_pub.publish(msg)


    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.des_state_log[0, self.log_counter] = self.des_x
            self.des_state_log[1, self.log_counter] = self.des_y
            self.des_state_log[2, self.log_counter] = self.des_theta
            self.state_log[0, self.log_counter] = self.basePoseW[0]
            self.state_log[1, self.log_counter] = self.basePoseW[1]
            self.state_log[2, self.log_counter] = self.basePoseW[5]
            self.basePoseW_log[:, self.log_counter] = self.basePoseW
            self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
            self.time_log[self.log_counter] = self.time
            self.log_counter += 1


    def plotData(self):

        # xy plot
        plt.figure()
        plt.title(f'{self.robot_name}')
        plt.plot(self.des_state_log[0, :],
                 self.des_state_log[1, :], "-r", label="desired")
        plt.plot(self.state_log[0, :],
                 self.state_log[1, :], "-b", label="real")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)


        plt.figure()
        plt.title(f'{self.robot_name}')
        plt.subplot(3, 1, 1)
        plt.plot(self.time_log, self.des_state_log[0, :], "-r", label="desired")
        plt.plot(self.time_log, self.state_log[0, :], "-b", label="real")
        plt.legend()
        plt.xlabel("time[m]")
        plt.ylabel("x[m]")
        plt.axis("equal")
        plt.grid(True)


        plt.title(f'{self.robot_name}')
        plt.subplot(3, 1, 2)
        plt.plot(self.time_log, self.des_state_log[1, :], "-r", label="desired")
        plt.plot(self.time_log, self.state_log[1, :], "-b", label="real")
        plt.legend()
        plt.xlabel("time[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)


        plt.title(f'{self.robot_name}')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_log, self.des_state_log[2, :], "-r", label="desired")
        plt.plot(self.time_log, self.state_log[2, :], "-b", label="real")
        plt.legend()
        plt.xlabel("time[m]")
        plt.ylabel("theta[m]")
        plt.axis("equal")
        plt.grid(True)


        # tracking errors
        # self.log_e_x, self.log_e_y, self.log_e_theta = self.controller.getErrors()
        # plt.figure()
        # plt.title(f'{self.robot_name}')
        # exy = np.sqrt(np.power(self.log_e_x, 2) +
        #               np.power(self.log_e_y, 2))
        # plt.plot(exy, "-b")
        # plt.ylabel("exy")
        # plt.title("tracking error")
        # plt.grid(True)
        #
        # plt.figure()
        # plt.plot(self.log_e_theta, "-b")
        # plt.ylabel("eth")
        # plt.grid(True)
        # plt.show(block=True)

        plt.show(block=True)

    def on_shutdown(self):
        print(f"[{self.robot_name}] received shutdown signal.")
        ros.signal_shutdown("killed")
            

def parse_args():
    # Remove ROS remappings (e.g., __name:=) so argparse won't choke
    argv = ros.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", type=int, default=1)
    parser.add_argument("--debug", type=str2bool, default=False, metavar="{true|false}")
    return parser.parse_args(argv[1:])

# --- helper for parsing "true/false" strings ---
def str2bool(v):
    if isinstance(v, bool):
        return v
    v = v.lower()
    if v in ("yes", "y", "true", "t", "1"):
        return True
    elif v in ("no", "n", "false", "f", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("expected a boolean (true/false)")

if __name__ == '__main__':
    main_args = parse_args()
    ctrl = Controller(robot_name=f"limo{main_args.robot_id}", debug=main_args.debug)
    ctrl.start_controller()

