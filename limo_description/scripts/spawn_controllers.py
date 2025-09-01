#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""
import time
import matplotlib
matplotlib.use('TkAgg')
from multiprocessing import Process
from controller import Controller

def run_robot(robot_name, debug):
    ctrl = Controller(robot_name, debug)
    ctrl.start_controller()
if __name__ == '__main__':

    n_robots = 2
    debug = True
    processes = []
    
    try:
        # Start children
        for robot in range(n_robots):
            p = Process(target=run_robot, args=(f'limo{robot}', debug))
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
