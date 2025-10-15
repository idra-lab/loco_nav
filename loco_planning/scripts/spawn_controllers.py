#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""
import time
import matplotlib
matplotlib.use('TkAgg')
from multiprocessing import Process, Event
from controller import Controller
from  utils.communication_utils import  checkRosMaster, launchFileNode
import os
import rospy

def kill_gazebo_processes():
    """
    Cleanly kill Gazebo and RViz processes left running.
    """
    print("[MAIN] Killing remaining Gazebo/RViz processes...")
    os.system("pkill -f gzserver")
    os.system("pkill -f gzclient")
    os.system("pkill -f rviz")
    os.system("pkill -f rosmaster")

def startSimulation(regenerate_map=True):
    kill_gazebo_processes()
    checkRosMaster()
    additional_args = ["start_controller:=false"]
    if not regenerate_map:
        additional_args.append("generate_new_config:=false")
    launchFileNode(package="loco_planning", launch_file="multiple_robots.launch", additional_args=additional_args)
    rospy.sleep(6.)

def run_robot(robot_name, debug, stop_event):
    ctrl = Controller(robot_name, debug)
    ctrl.start_controller()

if __name__ == '__main__':

    n_robots = 2
    debug = True
    processes = []
    stop_events = []
    try:
        startSimulation(regenerate_map=True)
        # Start children
        for robot in range(n_robots):
            stop_event = Event()
            p = Process(target=run_robot, args=(f'limo{robot}', debug, stop_event))
            p.start()
            processes.append(p)
            stop_events.append(stop_event)
        # Keep parent alive until children exit
        for p in processes:
            p.join() #will now return when child calls os._exit(0)

    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C caught! Shutting down all robot controllers...")

    finally:
        # Always try to clean up children
        for p in processes:
            if p.is_alive():
                p.terminate()
                p.join()   #wait for process to exit
                print(f"[MAIN] Terminated main process")
        kill_gazebo_processes()

