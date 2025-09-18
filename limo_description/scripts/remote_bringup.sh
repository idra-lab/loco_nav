#!/bin/bash
# Usage: ./remote_bringup.sh <namespace>
# Example: ./remote_bringup.sh limo0

NS=${1:-limo0}   # default = limo0

DOCKER_HOME=$(ssh agilex@limo "docker exec limo_docker bash -lc 'echo \$HOME'")


#-it … inside the ssh session is tryes to allocate a TTY twice — and over SSH so we remove the -it
# We don’t need an interactive TTY, because roslaunch isn’t an interactive shell

# kill the node before startup
#ssh agilex@limo "docker exec  -i limo_docker bash -ic 'rosnode kill /${NS}/limo_base_node; source /opt/ros/noetic/setup.bash;  source ${DOCKER_HOME}/ros_ws/devel/setup.bash;   roslaunch limo_base limo_base.launch  ns:=${NS}'"

#use a trap to kill on exit
cleanup() {
  echo "Killing remote bringup for $NS..."
  ssh agilex@limo "docker exec -i limo_docker bash -ic 'rosnode kill /${NS}/limo_base_node'"
}
trap cleanup EXIT

ssh agilex@limo "docker exec  -i limo_docker bash -ic 'rosnode kill /${NS}/limo_base_node; source /opt/ros/noetic/setup.bash;  source ${DOCKER_HOME}/ros_ws/devel/setup.bash;   roslaunch limo_base limo_base.launch  ns:=${NS}'"
