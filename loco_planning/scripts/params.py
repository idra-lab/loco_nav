import numpy as np

robot_params = {}

robot_params['limo0'] ={'dt': 0.01,
                         'k_p': 10.,
                         'k_th': 2,
                        'joint_names': ['limo0/front_left_wheel', 'limo0/front_right_wheel', 'limo0/rear_left_wheel',  'limo0/rear_right_wheel' ],
                        'real_robot': False,
                        'buffer_size': 50000}

robot_params['limo1'] = {'dt': 0.01,
                         'k_p': 10.,
                         'k_th': 2,
                        'joint_names': ['limo1/front_left_wheel', 'limo1/front_right_wheel', 'limo1/rear_left_wheel',  'limo1/rear_right_wheel' ],
                         'real_robot': False,
                         'buffer_size': 3000}

robot_params['limo2'] = {'dt': 0.01,
                         'k_p': 10.,
                         'k_th': 2,
                        'joint_names': ['limo2/front_left_wheel', 'limo2/front_right_wheel', 'limo2/rear_left_wheel',  'limo2/rear_right_wheel' ],
                         'real_robot': False,
                         'buffer_size': 3000}

plotting = True