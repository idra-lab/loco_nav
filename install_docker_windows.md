 

Install Docker on Windows 
================================================================================

1. First Install  Windows Subsystem for Linux (WSL). Open a command prompt with **administration** privileges and type. 

``` powershell
wsl --install
```

2. install Ubuntu 20.04.06 LTS from Microsoft Store. All the procedure is explained in detail here:

   https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overv

3. From Start run  Ubuntu 20.04.06 LTS, this is the procedure you should use to open a new terminal (do not use WSL).

5. To install the docker client in your Ubuntu and add to your user the privileges to run docker, you can run the following script:

```powershell
$sudo apt install curl
$curl -o install_docker.sh https://raw.githubusercontent.com/idra-lab/loco_nav/refs/heads/master/install_docker.sh
$sudo chmod +x install_docker.sh
$./install_docker.sh
```

5. This should have created the folder **trento_lab_home** in your $HOME. Now reboot the system.
6. now you can clone the loco_nav code inside the  **trento_lab_home/ros_ws/src** folder


```
$ cd ~/trento_lab_home/ros_ws/src
$ git clone https://github.com/idra-lab/loco_nav.git
```

7. Open a new Ubuntu terminal and download the docker image 

```powershell
$ docker pull mfocchi/trento_lab_framework:loco_nav
```

8. If you have any issue in downloading the image, create an Account on Docker Hub and login with your user and password typing:

```powershell
$ docker login
```

9. Edit the **.bashrc** inside the Ubuntu home folder:

```powershell
$ cd $HOME
$ gedit .bashrc
```

10. Copy the following alias inside the .bashrc and save.

```powershell
alias lab_planning='xhost +local:root; docker rm -f docker_container || true; \
docker run --name docker_container --gpus all \
--workdir="/root" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--device=/dev/dri:/dev/dri \
--network=host --hostname=docker -it \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--privileged --shm-size 2g --rm \
--volume $HOME/trento_lab_home:/root \
mfocchi/trento_lab_framework:loco_nav'
alias dock-other='docker exec -it docker_container /bin/bash'
```

11. Load the .bashrc script (next time you will open a terminal this will be automatically loaded).

```powershell
$ source .bashrc
```

12. From now on, follow the wiki on how to configure the ros environment in the "Configure Code" section: https://github.com/mfocchi/lab-docker#configure-code . 

**NOTE!:** The alias **lab** needs to be called only ONCE and opens the image. To link other terminals to the same image you should run **dock-other**, this second command will "**attach**" to the image opened previously by calling the **lab** alias.  You can call **lab** only once and **dock-other** as many times you need to open multiple terminals.

**NOTE!:** To perform git commands, be sure you have added an SSH key to your Github account (create one if you don't have it) following the procedure described   [here](https://github.com/mfocchi/lab-docker/blob/master/install_docker.md).



## **Code Management:**

To create your own code install the tool Visual Studio Code application together with the WSL extension as explained [here](https://code.visualstudio.com/docs/remote/wsl ). This will enable you to edit the code contained in the **trento_lab_home/ros_ws/src** folder.  

To be able to open more terminals on the same window install **terminator**:

```powershell
$ sudo apt install terminator
```



## **Nvidia Support**:

If you have a Nvidia GPU install the driver assiciated to your GPU model **directly** in windows downloading it from this [link]( https://www.nvidia.com/Download/index.aspx?lang=en-us ) (not inside Ubuntu!)

You can check if everything works with:

```
nvidia-smi
```

If you experiment any issue in using the Nvidia with OpenGL rendering (the symptom is that you cannot visualize STL meshes in RVIZ) then you should update to he latest mesa-driver:

```
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update
sudo apt install mesa-utils
```



**(Optional steps)**

13.  To install new packages open a terminal and call the alias "dock-root" and install with apt install **without** sudo. To store the changes in the local image, get the ASH (a number) of the active container with:

```powershell
$ docker ps 
```

14. Commit the docker image (next time you will open an new container it will retain the changes done to the image without loosing them):

```powershell
$ docker commit ASH mfocchi/trento_lab_framework:introrob
```

