Installation with Docker
================================================================================

- Run the script 

  [install_docker.sh]: https://github.com/mfocchi/loco_nav/tree/master/install_docker.sh

   This script is important because it installs the docker client on your machine and adds to your user the privileges to run the docker images

```
$ ./install_docker.sh
```
- If everything went smooth you should read: **To start docker, reboot the system!** You can now restart the PC so that all changes made can be applied.

- If you look into your **host** Ubuntu home directory, you will see that the **trento_lab_home** directory has been created with **/ros_ws/src** subfolders.

- now you can clone the loco_nav code inside the  **trento_lab_home/ros_ws/src** folder

- 


```
$ cd ~/trento_lab_home/ros_ws/src
$ git clone https://github.com/idra-lab/loco_nav.git
```

- how you have two options: 

  - A) Download the docker image from here:	

    ```
    docker pull mfocchi/trento_lab_framework:loco_nav
    ```

  - B) compile the docker image yourself:

    ```
    cd ~/trento_lab_home/ros_ws/src/loco_nav/docker/loco_nav
    docker build -t mfocchi/trento_lab_framework:loco_nav -f Dockerfile .
    ```

- Now, you need to configure the bash environment of your Ubuntu machine as follows. Open the `bashrc` file from your home folder:


  ```
  $ gedit ~/.bashrc
  ```

  -  and add the following lines at the bottom of the file:

  ```bash
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



  **NOTE!** If you do not have an Nvidia card in your computer, you should skip the parts about the installation of the drivers, and you can still run the docker **without** the **-nv** flag in the **lab** alias.

  - Open a terminal and run the "lab" alias:

  ```
  $ lab_planning
  ```

  - You should see your terminal change from `user@hostname` to `user@docker`. 
  - the **lab** script will mount the folder `~/trento_lab_home` on your **host** computer. Inside of all of the docker images this folder is mapped to `$HOME`.This means that any files you place   in your docker $HOME folder will survive the stop/starting of a new docker container. All other files and installed programs will disappear on the next run. 
  - The alias **lab** needs to be called only ONCE and opens the image. To link other terminals to the same image you should run **dock-other**, this second command will "**attach**" to the image opened previously by calling the **lab** alias.  You can call **lab** only once and **dock-other** as many times you need to open multiple terminals.

  **NOTE!** If you do not have an Nvidia card in your computer, you should skip the parts about the installation of the drivers, and you can still run the docker **without** the **--gpus all ** flag in the **lab** alias. 

  - Now you can compile the ROS workspace in the $HOME directory **inside** docker:


  ```
  $ cd  ~/ros_ws/
  $ catkin_make install
  ```



Installing NVIDIA drivers (optional)
--------------

If your PC is provided with an NVIDIA graphics card, you can install its drivers in Ubuntu by following these steps:

add the repository

```
sudo add-apt-repository ppa:graphics-drivers/ppa
```

update the repository list:

```
sudo apt-get update
```

Install the driver, note that for Ubuntu 20.04 the 515 version is ok, for Ubuntu 22.04 the 535 is ok, but you can use also other versions:

```
sudo apt-get install nvidia-driver-X
```

The reboot the system

```
sudo reboot
```

Now tell the system to use that driver:

* open the _Software & Updates_ application
* go to "Additional Drivers" and select the latest driver you just installed with "proprietary, tested" description
* press on "Apply Changes".

You can verify if the drivers are installed by opening a terminal and running:

```
nvidia-smi
```

If this does not work, and you are sure you correctly installed the drivers, you might need to deactivate the "safe boot" feature from your BIOS, that usually prevents to load the driver. 



## Docker Issues (optional)

--------------------------------------------------------------------------------

<a name="docker_issues"></a>

Check this section only if you had any issues in running the docker!

- When launching any graphical interface inside docker (e.g. pycharm or gedit) you get this error:

```
No protocol specified
Unable to init server: Could not connect: Connection refused

(gedit:97): Gtk-WARNING **: 08:21:29.767: cannot open display: :0.0
```

It means that docker is not copying properly the value of you DISPLAY environment variable, you could try to solve it in this way, in a terminal **outside docker** launch:

```
echo $DISPLAY
```

and you will obtain a **value**  (e.g. :0) if you run the same command in a docker terminal the value will be different, then in the .bashrc inside the docker add the following line:

```
export DISPLAY=value
```

- When installing docker using ./installation_tools/install_docker.sh you may have a pip3 syntax error. 

You could try to solve it in this way:

```
curl https://bootstrap.pypa.io/pip/3.5/get-pip.py -o get-pip.py
python3 get-pip.py
rm get-pip.py
```

- If you do not have Nvidia drivers installed, then make sure you are not using the `-nv` option when launching `lab-docker.py`. You may get a message in the terminal that looks like this:

  ![nvidia_issue](uploads/cd09602de0f7edd1e0432359754f495c/nvidia_issue.jpeg)

  

- Nvidia error: could not select device driver “” with capabilities:

You can solve this way:

```
sudo apt install -y nvidia-docker2
sudo systemctl daemon-reload
sudo systemctl restart docker
```







