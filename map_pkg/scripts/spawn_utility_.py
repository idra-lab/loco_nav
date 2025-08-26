#!/usr/bin/env python

import rospy
import yaml
import math
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

def load_yaml(path):
    while not os.path.exists(path):
        rospy.sleep(0.1)
    with open(path, 'r') as f:
        return yaml.load(f, Loader=yaml.FullLoader)

def gen_hex_points(L):
    sqrt3o2 = math.sqrt(3) / 2
    x0, y0 = L, 0
    x1, y1 = L/2, L*sqrt3o2
    x2, y2 = -L/2, L*sqrt3o2
    x3, y3 = -L, 0
    x4, y4 = -L/2, -L*sqrt3o2
    x5, y5 = L/2, -L*sqrt3o2
    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

def gen_rect_points(L1, L2):
    return [(L1, L2), (-L1, L2), (-L1, -L2), (L1, -L2)]

def get_borders_points(yaml_data):
    try:
        m = yaml_data['/_']['ros__parameters']['map']
        dx = float(yaml_data['/_']['ros__parameters']['dx'])
        dy = float(yaml_data['/_']['ros__parameters']['dy'])
        if "hex" in m:
            return gen_hex_points(dx)
        elif "rect" in m:
            return gen_rect_points(dx, dy)
        else:
            raise Exception("Unsupported map type: {}".format(m))
    except Exception as e:
        rospy.logerr("Error reading YAML: %s", str(e))
        return []

def spawn_borders():
    map_env_params_file = rospy.get_param('~map_env_params_file')
    gazebo_models_path = rospy.get_param('~gazebo_models_path')

    with open(map_env_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        map_type = params["/_"]['ros__parameters']['map']
        map_dx = params["/_"]['ros__parameters']['dx']
        map_dy = params["/_"]['ros__parameters']['dy']

    borders_model = ""
    sdf_template_path = ""

    if "hex" in map_type:
        L = map_dx
        [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)] = gen_hex_points(L)

        c0 = ((x0+x1)/2, (y0+y1)/2)
        c1 = ((x1+x2)/2, (y1+y2)/2)
        c2 = ((x2+x3)/2, (y2+y3)/2)
        c3 = ((x3+x4)/2, (y3+y4)/2)
        c4 = ((x4+x5)/2, (y4+y5)/2)
        c5 = ((x5+x0)/2, (y5+y0)/2)

        sdf_template_path = os.path.join(gazebo_models_path, 'hexagon_new', 'model.sdf')
        with open(sdf_template_path, 'r') as file:
            borders_model = file.read()
            borders_model = borders_model.replace("##L##", str(L))
            borders_model = borders_model.replace("##C0x##", str(c0[0])).replace("##C0y##", str(c0[1]))
            borders_model = borders_model.replace("##C1x##", str(c1[0])).replace("##C1y##", str(c1[1]))
            borders_model = borders_model.replace("##C2x##", str(c2[0])).replace("##C2y##", str(c2[1]))
            borders_model = borders_model.replace("##C3x##", str(c3[0])).replace("##C3y##", str(c3[1]))
            borders_model = borders_model.replace("##C4x##", str(c4[0])).replace("##C4y##", str(c4[1]))
            borders_model = borders_model.replace("##C5x##", str(c5[0])).replace("##C5y##", str(c5[1]))

    elif "rect" in map_type:
        width = 0.15
        L1 = (map_dx + width) / 2
        L2 = (map_dy + width) / 2

        sdf_template_path = os.path.join(gazebo_models_path, 'rectangle_world', 'model.sdf')
        with open(sdf_template_path, 'r') as file:
            borders_model = file.read()
            borders_model = borders_model.replace("dx", str(map_dx))
            borders_model = borders_model.replace("dy", str(map_dy))
            borders_model = borders_model.replace("width", str(width))
            borders_model = borders_model.replace("L1", str(L1))
            borders_model = borders_model.replace("L2", str(L2))

    else:
        rospy.logerr("Unsupported map type: %s", map_type)
        return

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        pose = Pose()  # x=0, y=0, z=0 by default
        resp = spawn_model("borders1", borders_model, "", pose, "world")
        rospy.loginfo("Spawn response: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('spawn_utility')
    try:
        spawn_borders()
    except rospy.ROSInterruptException:
        pass
