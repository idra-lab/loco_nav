#!/usr/bin/env python3

import os
import rospy
import yaml
import numpy as np
from geo_utility import *
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point

L = 1.0  # Default gate size
DELTA = L / 2.0 + 0.1  # half the border width + padding

def load_yaml(path):
    parmas_file = None
    while not os.path.isfile(path) and parmas_file is None:
        with open(path, 'r') as file:
            parmas_file = yaml.load(file, Loader=yaml.FullLoader)
    return parmas_file

def spawn_gates():
    rospy.init_node('spawn_gates')

    param_file = rospy.get_param('~gen_map_params_file')
    models_path = rospy.get_param('~elements_models_path')

    # Load parameters
    params = load_yaml(param_file)
    gates = params["/**/send_gates"]['ros__parameters']

    gate_model_path = os.path.join(models_path, 'gate', 'model.sdf')

    N = len(gates['x'])
    assert all(len(gates[k]) == N for k in ['y', 'yaw'])

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for i in range(N):
        with open(gate_model_path, 'r') as f:
            model_xml = f.read()

        x, y, yaw = gates['x'][i], gates['y'][i], gates['yaw'][i]
        pose = Pose(
            position=Point(x=x, y=y, z=0),
            orientation=quaternion_from_yaw(yaw)
        )

        model_name = f"gate_{i}"
        spawn_model(model_name, model_xml, "", pose, "world")

if __name__ == '__main__':
    try:
        spawn_gates()
    except rospy.ROSInterruptException:
        pass
