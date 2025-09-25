# Loco-nav




Michele Focchi, Enrico Saccon

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments
for the course of Robot Planning and applications.



# Installing Loco_nav code

We strongly suggest to install a docker image to avoid  compatibility issues. To see how to install the docker image follow these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker.md). 





# **Installing Real Robot Code**

1. connect both LIMO and HOSTCOMPUTER to the same WI-FI
2. add to both HOSTCOMPUTER and LIMO /etc/hosts

```
$LIMO_IP limo
$HOST_COMPUTER_IP hostcomputer
```

3. add to  HOSTCOMPUTER $HOME/trento_lab_home/.bashrc 

```
export ROS_IP=$HOST_COMPUTER_IP
```

4. add these aliases to HOSTCOMPUTER $HOME/.bashrc

```
alias connect_limo_nodocker='ssh -X -t agilex@limo'
alias connect_limo='ssh -X -t agilex@limo "bash -ic \"lab\" "'
alias attach_limo='ssh -X -t agilex@limo "bash -ic \"dock-other\" "'
```

5. copy docker file present in docker folder inside LIMO (pwd:agx)

```
cd $LOCONAV_FOLDER/docker_limo 
scp Dockerfile agilex@limo:~/docker
```

6. connect to LIMO

```
connect_limo_no_docker
```

7. compile docker inside LIMO

```
cd $HOME/docker
docker build -t introrob .
```

8. add these aliases inside LIMO $HOME/.bashrc

```
alias lab='xhost +local:docker; docker rm -f limo_docker || true; docker run --name limo_docker   --user $(id -u):$(id -g)  --workdir="/home/$USER" --volume="/etc/group:/etc/group:ro"   --volume="/etc/shadow:/etc/shadow:ro"  --volume="/etc/passwd:/etc/passwd:ro" --device=/dev/dri:/dev/dri  -e "QT_X11_NO_MITSHM=1" --network=host --hostname=docker -it  --device=/dev/ttyTHS1:/dev/ttyTHS1 --device=/dev/ttyUSB0:/dev/ydlidar   --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"  --env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY:rw" --volume $HOME/trento_lab_home:$HOME --env=HOME --env=USER  --privileged  -e SHELL --env="DISPLAY=$DISPLAY" --shm-size 2g --rm  --entrypoint /bin/bash introrob'

alias dock-other='docker exec -it limo_docker /bin/bash'
alias dock-root='docker exec -it --user root limo_docker /bin/bash'
```





# **Running the Code**  



### **Closed loop simulation**

**IMPORTANT!** to run simulation be sure you have commented **export ROS_IP=$HOST_COMPUTER_IP** in  HOSTCOMPUTER $HOME/trento_lab_home/.bashrc 

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
roslaunch limo_description multiple_robots.launch start_controller:=true debug:=true
```

8. To set a specific reference for the first robot

```
rostopic pub /limo0/ref limo_description/Reference "{x_d: 0.5, y_d: 0.0, theta_d: 0.0, v_d: 0.1, omega_d: 0.0}"
```

9. To test an RRT planner or a voronoi planner (alternative to 8.)

```
roslaunch limo_description planner_rrt.py
roslaunch limo_description planner_voronoi.py
```



### **Run on Real Robot**

1. Run docker on LIMO canning this in a HOSTCOMPUTER terminal:

```
connect_limo
```

2. In a new terminal in HOSTCOMPUTER start docker with the alias 

```
lab
```

3. in the same terminal in HOSTCOMPUTER (inside docker) run the alias 

```
real_robot
```

4. run (without lidar)

```
roslaunch limo_description labyrinth_mapping real_robot:=true 
```

5. run (with lidar)

```
roslaunch limo_description labyrinth_mapping real_robot:=true sensors:=true
```

N.B. if you want to switch back to sim run the alias **sim**



### IDE Pycharm

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

You can run separately the controllers for debugging, by running

```
ros_ws/src/loco_nav/limo_description/scripts/spawn_controllers.py
```

