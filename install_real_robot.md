# **Installing Real Robot Code**

1. connect both LIMO and HOSTCOMPUTER to the same WI-FI
2. add to both HOSTCOMPUTER and LIMO /etc/hosts the respective IPs (check the ip of the wifi board with ifconfig)

```
LIMO_IP limo
HOST_COMPUTER_IP hostcomputer
```

3. add these aliases to HOSTCOMPUTER $HOME/.bashrc

```
alias connect_limo_nodocker='ssh -X -t agilex@limo'
alias connect_limo='ssh -X -t agilex@limo "bash -ic \"lab\" "'
alias attach_limo='ssh -X -t agilex@limo "bash -ic \"dock-other\" "'
```

4. ROS TF requires all transforms to be within a **common clock**. If two machines disagree on the current time, TF will reject those transforms as “out of range”. Install and enable NTP or Chrony on both machines so they agree on the current time. Let's start to enable chrony on HOSTCOMPUTER for synchronization:

```
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl restart chrony (only once in life or just reboot)
```

5. copy docker file present in docker folder inside LIMO (pwd:agx)

```
cd $LOCONAV_FOLDER/docker_limo 
scp Dockerfile agilex@limo:~/docker
scp chrony.conf agilex@limo:~/docker
```

6. connect to LIMO robot computer

```
connect_limo_no_docker
```

7. compile docker inside LIMO

```
cd $HOME/docker
docker build -t loconav_robot .
```

8. add these aliases inside LIMO $HOME/.bashrc

```
alias lab='xhost +local:docker; docker rm -f limo_docker || true; docker run --name limo_docker   --user $(id -u):$(id -g)  --workdir="/home/$USER" --volume="/etc/group:/etc/group:ro"   --volume="/etc/shadow:/etc/shadow:ro"  --volume="/etc/passwd:/etc/passwd:ro" --device=/dev/dri:/dev/dri  -e "QT_X11_NO_MITSHM=1" --network=host --hostname=docker -it  --device=/dev/ttyTHS1:/dev/ttyTHS1 --device=/dev/ttyUSB0:/dev/ydlidar   --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"  --env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY:rw" --volume $HOME/trento_lab_home:$HOME --env=HOME --env=USER  --privileged  -e SHELL --env="DISPLAY=$DISPLAY" --shm-size 2g --rm  --entrypoint /bin/bash introrob'

alias dock-other='docker exec -it limo_docker /bin/bash'
alias dock-root='docker exec -it --user root limo_docker /bin/bash'
```

9. enable chrony on LIMO robot computer for synchronization:

```
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl restart chrony (only once in life or just reboot)
```

