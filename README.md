# Loco-nav




Michele Focchi, Enrico Saccon

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments
for the course of Robot Planning and applications.



# Installing the code

We strongly suggest to install a docker image to avoid  compatibility issues. To see how to install the docker image follow these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker.md). 



# **Running the Code**  



### **Closed loop simulation**

1. In a new terminal start docker with the alias **lab**

```
lab
```

2. Inside the docker run:

```
roslaunch 
```

3. To start the simulation

```
roslaunch limo_description multiple_robots.launch
```

4. To start the simulation + the controllers

```
roslaunch limo_description multiple_robots.launch start_controllers:=true
```

5. Open another docker terminal with the alias **dock-other** attached to the same image:

```
dock-other 
```

6. Publish the reference for the robots e.g.:

```
rostopic pub /limo1/ref limo_description/Reference "{x_d: 0.5, y_d: 0.0, theta_d: 0.0, v_d: 0.1, omega_d: 0.0}"
```

7. For debug mode (2 robots follow a predefined trajectory):

```
roslaunch limo_description multiple_robots.launch start_controllers:=true debug:=true
```

8. For debug mode (2 robots follow a predefined trajectory):

```
rostopic pub /limo1/ref limo_description/Reference "{x_d: 0.5, y_d: 0.0, theta_d: 0.0, v_d: 0.1, omega_d: 0.0}"
```

7. To test an RRT planner

```
roslaunch limo_description planner.py
```



### IDE Pycharm

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

You can run separately the controllers for debugging, by running

```
ros_ws/src/loco_nav/limo_description/scripts/spawn_controllers.py
```

