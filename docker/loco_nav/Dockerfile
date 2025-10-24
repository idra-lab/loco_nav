FROM osrf/ros:noetic-desktop-focal

ENV DEBIAN_FRONTEND=noninteractive

ENV DISABLE_ROS1_EOL_WARNINGS=1

# Setup timezone
RUN apt-get update && apt-get install -y tzdata && \ 
ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && dpkg-reconfigure -f noninteractive tzdata 

# Install basics +Python3 
RUN apt-get update && apt-get install -y \ 
locales git curl gnupg2 lsb-release build-essential wget cmake python3 python3-pip python3-dev \ 
&& locale-gen en_US en_US.UTF-8

# Install extra ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    ros-noetic-urdfdom-py \
    ros-noetic-srdfdom \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-joint-state-controller \
    ros-noetic-gazebo-msgs \
    ros-noetic-control-toolbox \
    ros-noetic-gazebo-ros \
    ros-noetic-controller-manager \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-catkin \
    ros-noetic-openni2-launch \
    ros-noetic-openni2-camera \
    ros-noetic-realsense2-description \
    ros-noetic-eigen-conversions \
    ros-noetic-object-recognition-msgs \
    ros-noetic-roslint \
    ros-noetic-ur-msgs \
    ros-noetic-speed-scaling-interface \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-ur-client-library \
    ros-noetic-pass-through-controllers \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-transmission-interface \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-tools \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-navigation \
    ros-noetic-turtlebot3-* \
    ros-noetic-move-base \
    ros-noetic-slam-gmapping \
    ros-noetic-key-teleop \
    ros-noetic-imu-sensor-controller \
    ros-noetic-diff-drive-controller \
    ros-noetic-eigenpy \                      
 && rm -rf /var/lib/apt/lists/*

# Python extras
RUN pip3 install --no-cache-dir  pyclipper
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-scipy \
    python3-matplotlib \
    python3-termcolor \
 && rm -rf /var/lib/apt/lists/*

# cvxpy (pin the version you asked)
RUN pip install --no-cache-dir cvxpy==1.2.0


# Add gedit for debugging
#utils
RUN apt-get update && apt-get install -y --no-install-recommends  \ 	 
		 gedit\
		 gitg\
		 git-cola\
		libxext6 libxrender1 libxtst6 libxi6 libfreetype6 fonts-dejavu \ 
		libgl1-mesa-glx libgtk-3-0 

RUN apt-get update && apt-get install -y --no-install-recommends libfcl-dev python3-tk
 
RUN pip install --no-cache-dir shapely quadprog
 
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-pkg python3-rospkg python3-empy ros-noetic-catkin \
 && rm -rf /var/lib/apt/lists/*

RUN python3 - <<'PY'
import re, pathlib
paths = [
    pathlib.Path("/opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py"),
    pathlib.Path("/usr/lib/python3/dist-packages/roslaunch/nodeprocess.py"),
]
for p in paths:
    if p.exists():
        s = p.read_text()
        s = re.sub(r'(_TIMEOUT_SIGINT\s*=\s*)[0-9.]+', r'\g<1>0.1', s)
        s = re.sub(r'(_TIMEOUT_SIGTERM\s*=\s*)[0-9.]+', r'\g<1>0.1', s)
        p.write_text(s)
        print("Patched:", p)
        break
PY

# Source ROS automatically  use single quote to have it literal and avoid expansion of empty $PYTHONPATH
RUN echo '#!/bin/bash' >> /etc/bash.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /root/ros_ws/devel/setup.bash ] && source /root/ros_ws/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'export LOCONAV_DIR=/root/ros_ws/src/loco_nav' >> /etc/bash.bashrc && \
    echo 'export PYTHONPATH=$PYTHONPATH:$LOCONAV_DIR/loco_planning/scripts' >> /etc/bash.bashrc && \
    echo 'alias real_robot='\''export ROS_IP=$LIMO_IP'\''' >> /etc/bash.bashrc && \
    echo 'alias sim='\''unset ROS_IP'\''' >> /etc/bash.bashrc && \
    echo 'alias pycharm='\''/root/pycharm/bin/pycharm.sh'\''' >> /etc/bash.bashrc
RUN chmod +x /etc/bash.bashrc

# your container now always runs Bash as a login shell, /etc/bash.bashrc will be sourced automatically.
ENTRYPOINT []
# Start bash by default (interactive login shell)
CMD ["/bin/bash", "--login", "-i"]
