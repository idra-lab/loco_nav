import numpy as np

robot_params = {}

robot_params['tractor0'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': -0.,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 30000}

robot_params['tractor1'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 1.,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

robot_params['tractor2'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': -1,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}


robot_params['tractor3'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 1,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

robot_params['tractor4'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 0,
                            'spawn_y': -1,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

exp_params = {0.09041: {"C1": -2.1725, "C2": -3.8977, "b_i_l": [0.2087, -3.5502], "b_o_l": [1.9365, -6.3664], "b_i_r": [0.2076,  3.5227], "b_o_r": [1.9361, 6.3658]},
              0.1: {"C1": -2.1835, "C2": -4.1271, "b_i_l": [0.2121,  -4.1594], "b_o_l": [1.9869, -6.7909], "b_i_r": [0.2120,   4.1570], "b_o_r": [1.9858,   6.7888]},
              0.13349: {"C1": -2.2145, "C2": -4.8659, "b_i_l": [0.2562, -7.3094], "b_o_l": [2.1652,  -8.2257], "b_i_r": [0.2559,  7.2989], "b_o_r": [2.1652, 8.2257]},
              0.1568: {"C1": -2.2149, "C2": -5.2914, "b_i_l": [0.3200,  -10.2641], "b_o_l": [2.2778, -9.1225], "b_i_r": [0.3199, 10.2591], "b_o_r": [2.2785,  9.1239]}}

global_dt = 0.01

verbose = False
plotting = True
