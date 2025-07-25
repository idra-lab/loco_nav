#!/usr/bin/env python3

import os
import rospy
import yaml
from geo_utility import *


from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def load_yaml(path):
    parmas_file = None
    while not os.path.isfile(path) and parmas_file is None:
        with open(path, 'r') as file:
            parmas_file = yaml.load(file, Loader=yaml.FullLoader)
    return parmas_file

def spawn_obstacles():
    rospy.init_node('spawn_obstacles')

    param_file = rospy.get_param('~gen_map_params_file')
    models_path = rospy.get_param('~elements_models_path')

    params = load_yaml(param_file)
    obstacles = params["/**/send_obstacles"]['ros__parameters']

    # Load model paths
    box_model_path = os.path.join(models_path, 'box', 'model.sdf')
    cylinder_model_path = os.path.join(models_path, 'cylinder', 'model.sdf')

    # Ensure vectors are the same length
    N = len(obstacles['vect_x'])
    assert all(len(obstacles[k]) == N for k in ['vect_y', 'vect_yaw', 'vect_type', 'vect_dim_x', 'vect_dim_y'])

    # Wait for the service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for i in range(N):
        # Prepare model data
        model_path = box_model_path if obstacles['vect_type'][i] == 0 else cylinder_model_path
        with open(model_path, 'r') as f:
            model_xml = f.read()

        pose = Pose(
            position=Point(obstacles['vect_x'][i], obstacles['vect_y'][i], 0),
            orientation=quaternion_from_yaw(obstacles['vect_yaw'][i])
        )

        model_name = f"obstacle_{i}"
        spawn_model(model_name, model_xml, "", pose, "world")

if __name__ == '__main__':
    try:
        spawn_obstacles()
    except rospy.ROSInterruptException:
        pass

