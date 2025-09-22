#!/bin/bash
# Usage: ./remote_bringup.sh <namespace>
# Example: ./remote_bringup.sh limo0

NS=${1:-limo0}   # default = limo0

DOCKER_HOME=$(ssh agilex@limo "docker exec limo_docker bash -lc 'echo \$HOME'")


#-it … inside the ssh session is tryes to allocate a TTY twice — and over SSH so we remove the -it
# We don’t need an interactive TTY, because roslaunch isn’t an interactive shell

# kill the node before startup
#ssh agilex@limo "docker exec  -i limo_docker bash -ic 'rosnode kill /${NS}/limo_base_node; source /opt/ros/noetic/setup.bash;  source ${DOCKER_HOME}/ros_ws/devel/setup.bash;   roslaunch limo_base limo_base.launch  ns:=${NS}'"

echo "to avoid entering the pwd, firt time type ssh-copy-id agilex@limo"

# Pre-check: ensure /dev/ttyTHS1 exists inside the limo_docker container
echo "[INFO] Checking /dev/ttyTHS1 inside limo_docker..."
if ! ssh agilex@limo "docker exec limo_docker test -e /dev/ttyTHS1"; then
  echo "[ERROR] /dev/ttyTHS1 not found inside limo_docker!"
  echo "        Make sure you started the container with:"
  echo "        --device=/dev/ttyTHS1:/dev/ttyTHS1"
  exit 1
fi


ssh agilex@limo "docker exec  -i limo_docker bash -ic 'pkill -f limo_base_node; rosnode kill /${NS}/limo_base_node; source /opt/ros/noetic/setup.bash;  source ${DOCKER_HOME}/ros_ws/devel/setup.bash;   roslaunch limo_base limo_base.launch  ns:=${NS}'"
