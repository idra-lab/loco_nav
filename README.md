# Loco-nav

Michele Focchi, Enrico Saccon, Marco Camurri

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments for the [*Robot Planning and its applications*](https://unitn.coursecatalogue.cineca.it/insegnamenti/2025/50880_650738_94680/2025/50880/10883?annoOrdinamento=2025) course, taught at the University of Trento.



# Installing `loco_nav`

We strongly suggest to install a docker image to avoid compatibility issues. 
To see how to install the docker image, please follow the instructions below for your favourite operating system:

- 🐧 [Linux](https://github.com/mfocchi/loco_nav/tree/master/install_docker_linux.md)
- 🍏 [MacOS](https://github.com/mfocchi/loco_nav/tree/master/install_docker_mac.md)
- 🪟 [Windows](https://github.com/mfocchi/loco_nav/tree/master/install_docker_windows.md).

## Installing the code on the robot

To install the code and prepare the setup to use the real robot, please follow these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_real_robot.md).

# Running the Code in Simulation

## Closed loop simulation

> [!IMPORTANT]
> to run simulation be sure you have commented `bash export ROS_IP=$HOST_COMPUTER_IP` in  `HOSTCOMPUTER`'s `$HOME/trento_lab_home/.bashrc`

1. In a new terminal, start a docker image by running the alias **lab_planning**

```
lab_planning
```

2. To start a simulation with 2 LIMO in the hexagon arena

```
roslaunch loco_planning multiple_robots.launch
```

3. Open another docker terminal with the alias **dock-other** attached to the same image:

```
dock-other 
```

4. To send a specific reference for the first robot:

```
rostopic pub /limo0/ref loco_planning/Reference "{x_d: 0.4, y_d: 0.0, theta_d: 0.0, v_d: 0.01, omega_d: 0.0}"
```

5. To test an RRT planner (alternative to 4.) (first do point 2. or set StartSimulation=True)

```
python3 -i $LOCONAV_DIR/loco_planning/scripts/planner_base.py
```

### Alternative Options

1. To run the code in debug mode (2 robots follow a predefined chicane trajectory):

```
roslaunch loco_planning multiple_robots.launch start_controller:=true debug:=true
```

2. To generate always the same map (stored in `$(find map_pkg)/config/full_config.yaml`): 

```
roslaunch loco_planning multiple_robots.launch start_controller:=true generate_new_config:=false
```

### Acquire a Map of a Labyrinth

1.  launch the simulation with the Labyrinth, the `gmapping` package and the LIDAR sensor enabled: 

```
roslaunch loco_planning labyrinth_slam_toolbox.launch 
```

2. Move around with the keyboard till you have acquired the whole map, the control is done by the teleop package which sets desired speeds.
3. Finally, store the map inside map_pkg/maps folder

```
rosrun map_server map_saver -f $(rospack find map_pkg)/maps/labyrinth
```

4. Note that the map_server stores absolute path for the image location in labyrinth.yaml so you need to manually make it relative 

```
roscd map_pkg/maps
gedit labyrinth.yaml
```

5.  changes to:

```
image: labyrinth.pgm
```

6. As an alternative to 3., 4. and 5 (which was the standard way of doing it),  you can run directly the customized script that will fix this for you

```
rosrun map_pkg save_map.py labyrinth
```



### Navigate the Labyrinth

1.  launch the simulation with the Labyrinth, the **amcl** localization package and the LIDAR sensor enabled: 

```
roslaunch loco_planning labyrinth_amcl.launch sensors:=true
```



# Running the code on the real robot

Be sure you have setup the hostcomputer and the LIMO robot computer appropriately following these [instructions](https://https://github.com/idra-lab/loco_nav/blob/master/install_real_robot.md). 

1. Run docker on LIMO canning this in a HOSTCOMPUTER terminal:

```
connect_limo
```

2. In a new terminal in HOSTCOMPUTER start docker with the alias 

```
lab_planning
```

3. In the same terminal in HOSTCOMPUTER (inside docker) run the alias 

```
real_robot
```

4. run (without lidar)

```
roslaunch limo_description start_robot.launch real_robot:=true
```

5. setting these flags is possible to set:

```
odometry:=false => use optitrack node
sensors:=true => activate LIDAR
teleop_control:=true => start a teleop_keyboard node to issue velocity commands
```

N.B. if you want to switch back to sim run the alias **sim**

6. To test a predefined chicane trajectory run the code:

```
python3 -i limo_control.py
```

you can use this code both for simulation and real robot, just set the parameter real_robot: False/True in **params.yaml**. If you use limo_control.py he will take care of calling the alias real_robot and sim automatically, you do not have to do it.

7. to acquire a map with the real robot run

```
roslaunch limo_description start_robot.launch real_robot:=true sensors:=true mapping:=true teleop_control:=true
```

8. move around the robot till you acquired the whole map then save with 

```
rosrun map_pkg save_map.py real_arena
```

this will save in ros_ws/install/share/map_pkg/maps/real_arena.yaml 

9. To use the acquired map just run 

```
roslaunch limo_description start_robot.launch real_robot:=true sensors:=true
```

this will publish the discretized map as points in the topic /obstacles that you can use for your planner.



# Pycharm IDE

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

You can run separately the controllers for debugging, by running

```
ros_ws/src/loco_nav/loco_planning/scripts/spawn_controllers.py
```
