#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, yaml, random
import numpy as np
import subprocess
import rospy
from termcolor import colored
from geo_utility import *
from spawn_borders import get_borders_points

L = 1.0  # Default size for gate
DELTA = L / 2.0 + 0.1  # 0.1 is half the border width + something


def spawn_gates():
    while  not rospy.has_param('/generate_config_file/gen_map_params_file'):
        rospy.sleep(0.1)
    gen_map_params_file = rospy.get_param('/generate_config_file/gen_map_params_file')

    while not rospy.has_param('/spawn_gates/elements_models_path'):
        rospy.sleep(0.1)
    elements_models_path = rospy.get_param('/spawn_gates/elements_models_path')

    while not os.path.exists(gen_map_params_file):
        rospy.sleep(0.1)
    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        gates = params["/_/send_gates"]['ros__parameters']

    gate_model_path = os.path.join(elements_models_path, 'gate', 'model.sdf')

    # Check that all vectors have the same number of gates
    assert len(gates['x']) == len(gates['y']) == len(gates['yaw']), \
        "The number of gates in the conf file is wrong. The script for generating the configuration made a mistake."

    print(colored(f"spawn_gates.py Spawing gates",'blue'))
    commands = []
    for gate in range(len(gates['x'])):
        print(f"spawn_gates.py: Spawning gate in {gates['x'][gate]}, {gates['y'][gate]} with yaw {gates['yaw'][gate]}")
        # If x and y are 0, then the gate is random
        yaw = gates['yaw'][gate]
        gate_polygon = square(gates['x'][gate], gates['y'][gate], L, gates['yaw'][gate])

        center = center = gate_polygon.centroid.coords[0]

        # ROS1 equivalent of ROS2 spawn_entity.py:
        # rosrun gazebo_ros spawn_model -file <path> -sdf -model gates<gate> -x <x> -y <y> -z 0.0001 -Y <yaw>
        cmd = [
            'rosrun', 'gazebo_ros', 'spawn_model',
            '-file', gate_model_path,
            '-sdf',
            '-model', f"gates{gate}",
            '-x', str(center[0]),
            '-y', str(center[1]),
            '-z', str(0.0001),
            '-Y', str(yaw)
        ]

        try:
            subprocess.check_call(cmd)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to spawn gate {gate}: {e}")

        commands.append(cmd)

    return commands


if __name__ == "__main__":
    rospy.init_node('spawn_gates_ros1')
    spawn_gates()
