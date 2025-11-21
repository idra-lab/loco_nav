# Loco-nav




Michele Focchi, Enrico Saccon

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments
for the course of Robot Planning and applications.



# Installing Loco_nav code

We strongly suggest to install a docker image to avoid  compatibility issues. To see how to install the docker image follow:

LINUX:  these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker_linux.md).

MAC: these  [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker_mac.md).

WINDOWS: these  [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker_windows.md).



# **Installing Real Robot Code**

To install the code and prepare the setup to use the real robot follow these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_real_robot.md).



# **Running the Code** in Simulation

### **Closed loop simulation**

**IMPORTANT!** to run simulation be sure you have commented **export ROS_IP=$HOST_COMPUTER_IP** in  HOSTCOMPUTER $HOME/trento_lab_home/.bashrc 

1. In a new terminal start docker with the alias **lab**

```
lab
```

2. To start the simulation with 2 LIMO in the hexagon arena

```
roslaunch loco_planning multiple_robots.launch
```

3. Open another docker terminal with the alias **dock-other** attached to the same image:

```
dock-other 
```

4. To send a specific reference for the first robot:

```
rostopic pub /limo0/ref loco_planning/Reference "{x_d: 0.5, y_d: 0.0, theta_d: 0.0, v_d: 0.1, omega_d: 0.0}"
```

5. To test an RRT planner (alternative to 4.) (first do point 2. or set StartSimulation=True)

```
roslaunch loco_planning planner_rrt.py
```

6. To test an voronoi planner (alternative to 5., set StartSimulation=True)

```
roslaunch loco_planning planner_voronoi.py
```



### Options

O1. For debug mode (2 robots follow a predefined chicane trajectory):

```
roslaunch loco_planning multiple_robots.launch start_controller:=true debug:=true
```

O2. To generate always the same map (stored in $(find map_pkg)/config/full_config.yaml): 

```
roslaunch loco_planning multiple_robots.launch start_controller:=true generate_new_config:=false
```



### **Acquire a Map of a Labyrinth**

1.  launch the simulation with the Labyrinth, the **gmapping** package and the LIDAR sensor enabled: 

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



### **Navigate the Labyrinth**

1.  launch the simulation with the Labyrinth, the **amcl** localization package and the LIDAR sensor enabled: 

```
roslaunch loco_planning labyrinth_amcl.launch sensors:=true
```



# **Running the Code on Real Robot**

1. Run docker on LIMO canning this in a HOSTCOMPUTER terminal:

```
connect_limo
```

2. In a new terminal in HOSTCOMPUTER start docker with the alias 

```
lab_planning
```

3. in the same terminal in HOSTCOMPUTER (inside docker) run the alias 

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
teleop_contro => start a teleop_keyboard node to issue velocity commands
```

N.B. if you want to switch back to sim run the alias **sim**

6. To test a chicane trajectory run the code:

```
python3 -i limo_control.py
```

you can use this code both for simulation and real robot, just set the parameter real_robot: False/True in **params.yaml**. If you use limo_control.py he will take care of calling the alias real_robot and sim autonomatically, you do not have to do it.

### IDE Pycharm

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

You can run separately the controllers for debugging, by running

```
ros_ws/src/loco_nav/loco_planning/scripts/spawn_controllers.py
```

