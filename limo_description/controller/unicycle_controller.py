# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
import rosgraph
import numpy as np
import signal
import sys
import time

from multiprocessing import Process
from termcolor import colored
from lyapunov import LyapunovController, LyapunovParams
import params as conf
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)

# Lyapunov Controller node
class Controller():

    def __init__(self, robot_name="limo1"):
        self.robot_name = robot_name
        # Lyapunov controller parameters
        params = LyapunovParams(K_P=10., K_THETA=1., DT=0.01)
        self.controller = LyapunovController(params=params)

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)

        # fundamental otherwise receivepose gets stuck
        self.quaternion = np.array([0., 0., 0., 1.])
        self.q_des = conf.robot_params[self.robot_name]['q_0']

        self.time = np.zeros(1)
        self.log_counter = 0

        # log vars
        self.basePoseW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comPoseW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comTwistW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_des_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_des_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_ffwd_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.time_log = np.full(
            (conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        # add your variables to initialize here
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V = 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(2)
        self.ctrl_v_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.ctrl_omega_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.v_d_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.omega_d_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.V_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.V_dot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.des_x = 0.
        self.des_y = 0.
        self.des_theta = 0.

        self.state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        vel_gen = VelocityGenerator(
            simulation_time=20., DT=conf.robot_params[self.robot_name]['dt'])
        v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(
            v_max_=0.2, omega_max_=0.3)
        self.traj = Trajectory(ModelsList.UNICYCLE, self.pose_init[0], self.pose_init[1], self.pose_init[2], DT=conf.robot_params[self.robot_name]['dt'],
                               v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)


    def start_controller(self):
        ros.init_node(f'{self.robot_name}_controller', anonymous=False, log_level=ros.FATAL)
        ros.on_shutdown(self.on_shutdown)
        
        rate = ros.Rate(100)  # 100Hz loop, adjust as needed
        while not ros.is_shutdown():
            try:
                rate.sleep()
                # Add the control loop here!
                print(f"[{self.robot_name}] running...")
            except (ros.ROSInterruptException, ros.service.ServiceException):
                break
            
    def on_shutdown(self):
        print(f"[{self.robot_name}] received shutdown signal.")
        ros.signal_shutdown("killed")
            
def run_robot(robot_name):
    ctrl = Controller(robot_name)
    ctrl.start_controller()
        
                  
if __name__ == '__main__':
    
    n_robots = 3
    processes = []
    
    try:
        # Start children
        for robot in range(n_robots):
            p = Process(target=run_robot, args=(f'limo{robot}',))
            p.start()
            processes.append(p)

        # Keep parent alive until children exit
        for p in processes:
            p.join()

    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C caught! Shutting down all robot controllers...")

    finally:
        # Always try to clean up children
        for p in processes:
            if p.is_alive():
                p.terminate()
                p.join()   #wait for process to exit
                print(f"[MAIN] Terminated main process")