#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, math
from pathlib import Path
import yaml
import subprocess
import rospy

from geo_utility import *


def get_borders_points(yaml_file):
    try:
        if "hex" in yaml_file['/_']['ros__parameters']['map']:
            return gen_hex_points(float(yaml_file['/_']['ros__parameters']['dx']))
        elif "rect" in yaml_file['/_']['ros__parameters']['map']:
            return gen_rect_points(float(yaml_file['/_']['ros__parameters']['dx']),
                                   float(yaml_file['/_']['ros__parameters']['dy']))
        else:
            raise Exception("[{}] Map type `{}` not supported".format(__file__, yaml_file['/_']['ros__parameters']['map']))

    except Exception as e:
        print("The yaml file does not contain the map type")
        return []



def spawn_borders():
    # In ROS1, use parameters instead of context.launch_configurations
    while not rospy.has_param('/generate_config_file/gen_map_params_file'):
        rospy.sleep(0.1)
    gen_map_params_file = rospy.get_param('/generate_config_file/gen_map_params_file')

    while not rospy.has_param('/spawn_borders/gazebo_models_path'):
        rospy.sleep(0.1)
    gazebo_models_path = rospy.get_param('/spawn_borders/gazebo_models_path')

    while not os.path.exists(gen_map_params_file):
        rospy.sleep(0.1)

    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)


    map_type = params["/_"]['ros__parameters']['map']
    map_dx = params["/_"]['ros__parameters']['dx']
    map_dy = params["/_"]['ros__parameters']['dy']

    print("map_type, map_dx, map_dy", map_type, map_dx, map_dy)

    borders_model = ""

    if "hex" in map_type:
        L = map_dx
        sqrt3o2 = math.sqrt(3) / 2  # kept for parity with original

        [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)] = gen_hex_points(L)

        c0 = ((x0 + x1) / 2, (y0 + y1) / 2)
        c1 = ((x1 + x2) / 2, (y1 + y2) / 2)
        c2 = ((x2 + x3) / 2, (y2 + y3) / 2)
        c3 = ((x3 + x4) / 2, (y3 + y4) / 2)
        c4 = ((x4 + x5) / 2, (y4 + y5) / 2)
        c5 = ((x5 + x0) / 2, (y5 + y0) / 2)

        with open(os.path.join(gazebo_models_path, 'hexagon_new', 'model.sdf'), 'r') as file:
            borders_model = file.read()

        borders_model = borders_model.replace("##L##", str(L))
        borders_model = borders_model.replace("##C0x##", str(c0[0])).replace("##C0y##", str(c0[1]))
        borders_model = borders_model.replace("##C1x##", str(c1[0])).replace("##C1y##", str(c1[1]))
        borders_model = borders_model.replace("##C2x##", str(c2[0])).replace("##C2y##", str(c2[1]))
        borders_model = borders_model.replace("##C3x##", str(c3[0])).replace("##C3y##", str(c3[1]))
        borders_model = borders_model.replace("##C4x##", str(c4[0])).replace("##C4y##", str(c4[1]))
        borders_model = borders_model.replace("##C5x##", str(c5[0])).replace("##C5y##", str(c5[1]))

        borders_points = [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

    elif "rect" in map_type:
        width = 0.15
        L1 = (map_dx - width) / 2.0
        L2 = (map_dy - width) / 2.0

        print("L1, L2", L1, L2)

        with open(os.path.join(gazebo_models_path, 'rectangle_world', 'model.sdf'), 'r') as file:
            borders_model = file.read().replace("##dx##", str(map_dx)).replace("##dy##", str(map_dy)).replace("##width##", str(width))
            borders_model = borders_model.replace("##L1##", str(L1)).replace("##L2##", str(L2))

    else:
        raise Exception("[{}] Map type `{}` not supported".format(__file__, map_type))

    with open('tmp.sdf', 'w') as file:
        file.write(borders_model)
        path = Path(file.name).resolve()

    # ROS1 equivalent of ROS2 spawn_entity.py:
    # rosrun gazebo_ros spawn_model -file tmp.sdf -sdf -model borders1 -x 0 -y 0 -z 0
    cmd = [
        'rosrun', 'gazebo_ros', 'spawn_model',
        '-file', str(path),
        '-sdf',
        '-model', 'borders1',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.0'
    ]

    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to spawn borders: {}".format(e))


if __name__ == "__main__":
    rospy.init_node('spawn_borders_ros1')
    spawn_borders()
