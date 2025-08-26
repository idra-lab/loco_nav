#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, yaml, random
import numpy as np
from pathlib import Path
import subprocess
import rospy

from geo_utility import *
from spawn_borders import get_borders_points


def spawn_obstacles():
    # In ROS1 we fetch what ROS2 put in context.launch_configurations via ROS params.
    while  not rospy.has_param('/generate_config_file/gen_map_params_file'):
        rospy.sleep(0.1)
    gen_map_params_file = rospy.get_param('/generate_config_file/gen_map_params_file')

    while not rospy.has_param('/spawn_obstacles/elements_models_path'):
        rospy.sleep(0.1)
    elements_models_path = rospy.get_param('/spawn_obstacles/elements_models_path')

    while not os.path.exists(gen_map_params_file):
        rospy.sleep(0.1)
    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        obstacles = params["/_/send_obstacles"]['ros__parameters']

    box_model_path = os.path.join(elements_models_path, 'box', 'model.sdf')
    cylinder_model_path = os.path.join(elements_models_path, 'cylinder', 'model.sdf')

    # Check that all vectors have the same number of obstacles
    assert len(obstacles['vect_x']) == len(obstacles['vect_y']) == \
           len(obstacles['vect_yaw']) == len(obstacles["vect_type"]) == \
           len(obstacles['vect_dim_x']) == len(obstacles["vect_dim_y"]), \
        "The number of obstacles in the conf file is wrong. The script for generating the configuration made a mistake."

    commands = []
    for obs in range(len(obstacles['vect_x'])):
        # If x and y are 0, then the gate is random

        if obstacles['vect_type'][obs] == "box":
            obs_polygon = rectangle(obstacles['vect_x'][obs], obstacles['vect_y'][obs],
                                    obstacles['vect_dim_x'][obs], obstacles['vect_dim_y'][obs],
                                    obstacles['vect_yaw'][obs])
            with open(box_model_path, 'r') as in_file:
                obs_model = in_file.read().replace(
                    "<size>1 1 1</size>",
                    f"<size>{obstacles['vect_dim_x'][obs]} {obstacles['vect_dim_y'][obs]} 1</size>"
                )

        elif obstacles['vect_type'][obs] == "cylinder":
            obs_polygon = circle(obstacles['vect_x'][obs], obstacles['vect_y'][obs],
                                 obstacles['vect_dim_x'][obs])
            with open(cylinder_model_path, 'r') as in_file:
                obs_model = in_file.read().replace(
                    "<radius>0.5</radius>",
                    f"<radius>{obstacles['vect_dim_x'][obs]}</radius>"
                )
        else:
            raise Exception(f"The obstacle type {obstacles['vect_type'][obs]} is not supported")
        center = [obs_polygon.centroid.x, obs_polygon.centroid.y]
        #center = [centroid(obs_polygon).coords[0][0], centroid(obs_polygon).coords[0][1]]
        print("Spawning obstacle in ", center, abs(center[0]), abs(center[1]))
        if abs(center[0]) < 1e-8:
            center[0] = 0
        if abs(center[1]) < 1e-8:
            center[1] = 0

        # Write temporary SDF like in ROS2 version
        with open(f"obs{obs}_tmp.sdf", 'w') as out_file:
            out_file.write(obs_model)
            path = Path(out_file.name).resolve()

        # ROS1 equivalent of ROS2 spawn_entity.py:
        # rosrun gazebo_ros spawn_model -file <path> -sdf -model obstacles<obs> -x <x> -y <y> -z 0.0001 -Y <yaw>
        cmd = [
            'rosrun', 'gazebo_ros', 'spawn_model',
            '-file', str(path),
            '-sdf',
            '-model', f'obstacles{obs}',
            '-x', str(center[0]),
            '-y', str(center[1]),
            '-z', str(0.0001),
            '-Y', str(obstacles["vect_yaw"][obs])
        ]

        # Execute immediately (like launching the Node in ROS2)
        try:
            subprocess.check_call(cmd)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to spawn obstacle {obs}: {e}")

        commands.append(cmd)

    return commands


if __name__ == "__main__":
    rospy.init_node('spawn_obstacles_ros1_wrapper')
    spawn_obstacles()
