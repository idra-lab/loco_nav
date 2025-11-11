# Loco-nav




Michele Focchi, Enrico Saccon

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments
for the course of Robot Planning and applications.



# Installing Loco_nav code

We strongly suggest to install a docker image to avoid  compatibility issues. To see how to install the docker image follow these [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker.md) for Linux users and these  [instructions](https://github.com/mfocchi/loco_nav/tree/master/install_docker_windows.md) for Windows users.



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

5. ROS TF requires all transforms to be within a **common clock**. If two machines disagree on the current time, TF will reject those transforms as “out of range”. Install and enable NTP or Chrony on both machines so they agree on the current time. Let's start to enable chrony on HOSTCOMPUTER for synchronization:

```
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl restart chrony (only once in life or just reboot)
```

6. copy docker file present in docker folder inside LIMO (pwd:agx)

```
cd $LOCONAV_FOLDER/docker_limo 
scp Dockerfile agilex@limo:~/docker
scp chrony.conf agilex@limo:~/docker
```

7. connect to LIMO robot computer

```
connect_limo_no_docker
```

8. compile docker inside LIMO

```
cd $HOME/docker
docker build -t loconav_robot .
```

9. add these aliases inside LIMO $HOME/.bashrc

```
alias lab='xhost +local:docker; docker rm -f limo_docker || true; docker run --name limo_docker   --user $(id -u):$(id -g)  --workdir="/home/$USER" --volume="/etc/group:/etc/group:ro"   --volume="/etc/shadow:/etc/shadow:ro"  --volume="/etc/passwd:/etc/passwd:ro" --device=/dev/dri:/dev/dri  -e "QT_X11_NO_MITSHM=1" --network=host --hostname=docker -it  --device=/dev/ttyTHS1:/dev/ttyTHS1 --device=/dev/ttyUSB0:/dev/ydlidar   --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"  --env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY:rw" --volume $HOME/trento_lab_home:$HOME --env=HOME --env=USER  --privileged  -e SHELL --env="DISPLAY=$DISPLAY" --shm-size 2g --rm  --entrypoint /bin/bash introrob'

alias dock-other='docker exec -it limo_docker /bin/bash'
alias dock-root='docker exec -it --user root limo_docker /bin/bash'
```

10. enable chrony on LIMO robot computer for synchronization:

```
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl restart chrony (only once in life or just reboot)
```



# **Running the Code**  



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
roslaunch loco_planning labyrinth_gmapping real_robot:=true sensors:=false
```

5. run (with lidar)

```
roslaunch loco_planning labyrinth_gmapping real_robot:=true sensors:=true
```

N.B. if you want to switch back to sim run the alias **sim**



### IDE Pycharm

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

You can run separately the controllers for debugging, by running

```
ros_ws/src/loco_nav/loco_planning/scripts/spawn_controllers.py
```

