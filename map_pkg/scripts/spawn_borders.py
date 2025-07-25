#!/usr/bin/env python

import os
import math
import rospy
import yaml
from pathlib import Path
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def gen_hex_points(L):
    sqrt3o2 = math.sqrt(3) / 2
    x0, y0 = L, 0
    x1, y1 = L/2, L*sqrt3o2
    x2, y2 = -L/2, L*sqrt3o2
    x3, y3 = -L, 0
    x4, y4 = -L/2, -L*sqrt3o2
    x5, y5 = L/2, -L*sqrt3o2
    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

def gen_rect_points(dx, dy):
    return [(dx/2, dy/2), (-dx/2, dy/2), (-dx/2, -dy/2), (dx/2, -dy/2)]

def load_yaml(path):
    parmas_file = None
    with open(path, 'r') as file:
        for line in file:
            print(line)
    while not os.path.isfile(path) and parmas_file is None:
        with open(path, 'r') as file:
            parmas_file = yaml.load(file, Loader=yaml.FullLoader)
    return parmas_file

def spawn_borders():
    rospy.init_node('spawn_borders_node')

    # Load params from rosparam or command-line args
    gen_map_params_file = rospy.get_param('~gen_map_params_file')
    gazebo_models_path = rospy.get_param('~gazebo_models_path')

    params = load_yaml(gen_map_params_file)
    map_type = params["/**"]['ros__parameters']['map']
    map_dx = params["/**"]['ros__parameters']['dx']
    map_dy = params["/**"]['ros__parameters']['dy']

    rospy.loginfo("map_type: %s, dx: %f, dy: %f", map_type, map_dx, map_dy)

    borders_model = ""

    if "hex" in map_type:
        L = map_dx
        hex_points = gen_hex_points(L)

        c = [((hex_points[i][0]+hex_points[(i+1)%6][0])/2,
              (hex_points[i][1]+hex_points[(i+1)%6][1])/2) for i in range(6)]

        sdf_path = os.path.join(gazebo_models_path, 'hexagon_new', 'model.sdf')
        with open(sdf_path, 'r') as f:
            borders_model = f.read()

        borders_model = borders_model.replace("##L##", str(L))
        for i in range(6):
            borders_model = borders_model.replace(f"##C{i}x##", str(c[i][0]))
            borders_model = borders_model.replace(f"##C{i}y##", str(c[i][1]))

    elif "rect" in map_type:
        width = 0.15
        L1 = (map_dx - width) / 2.0
        L2 = (map_dy - width) / 2.0

        rospy.loginfo("L1: %f, L2: %f", L1, L2)

        sdf_path = os.path.join(gazebo_models_path, 'rectangle_world', 'model.sdf')
        with open(sdf_path, 'r') as f:
            borders_model = f.read()
            borders_model = borders_model.replace("##dx##", str(map_dx))
            borders_model = borders_model.replace("##dy##", str(map_dy))
            borders_model = borders_model.replace("##width##", str(width))
            borders_model = borders_model.replace("##L1##", str(L1))
            borders_model = borders_model.replace("##L2##", str(L2))

    else:
        rospy.logerr("Unsupported map type: %s", map_type)
        return

    tmp_path = '/tmp/borders_model.sdf'
    with open(tmp_path, 'w') as file:
        file.write(borders_model)
    rospy.loginfo("Wrote SDF to %s", tmp_path)

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        pose = Pose()  # default at 0, 0, 0
        resp = spawn_model("borders1", borders_model, "", pose, "world")
        rospy.loginfo("Spawned model: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_borders()
    except rospy.ROSInterruptException:
        pass
