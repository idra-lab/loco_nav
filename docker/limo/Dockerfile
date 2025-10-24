FROM arm64v8/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic   

# Setup timezone
RUN apt-get update && apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

# Install basics
RUN apt-get update && apt-get install -y \
    locales git curl gnupg2 lsb-release build-essential \
    && locale-gen en_US en_US.UTF-8
    

# Add ROS Noetic repo with updated GPG key
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list

# Install minimal ROS (ros-core)
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-core \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-tools \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*



# Install basics + Python3 + pip
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release build-essential \
    python3 python3-pip python3-dev \
    && locale-gen en_US en_US.UTF-8

# Install limo_ros from PyPI
RUN pip3 install --no-cache-dir pylimo pyclipper

# utils
RUN apt-get install -y iputils-ping gedit 



# Define ROS workspace inside $HOME
ENV ROS_WS=$HOME/ros_ws
RUN mkdir -p $ROS_WS/src

# Clone loco_nav only if it doesn't exist
WORKDIR $ROS_WS/src
RUN if [ ! -d "loco_nav" ]; then \
      git clone https://github.com/idra-lab/limo_hardware_interface.git; \
    else \
      echo "loco_nav already exists, skipping clone."; \
    fi

# Go to workspace root and install dependencies
WORKDIR $ROS_WS
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


 
# Install chrony for sync of date between host and limo pc
RUN apt-get update && apt-get install -y --no-install-recommends \
    chrony && \
    rm -rf /var/lib/apt/lists/*

# Copy in chrony configuration (host PC as server, adjust IP!)
COPY chrony.conf /etc/chrony/chrony.conf

# Script that starts chrony, then execs your container command
RUN echo '#!/bin/bash\n' \
         'service chrony start\n' \
         'exec "$@"\n' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"] 
 

# Patch roslaunch/nodeprocess.py to reduce shutdown timeout
RUN sed -i 's/^_TIMEOUT_SIGINT = .*/_TIMEOUT_SIGINT = 0.1/' \
        /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py && \
    sed -i 's/^_TIMEOUT_SIGTERM = .*/_TIMEOUT_SIGTERM = 0.1/' \
        /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py	

# /etc/bash.bashrc is sourced by all interactive shells, regardless of which user you start as so I copy env there
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f \$HOME/ros_ws/devel/setup.bash ] && source \$HOME/ros_ws/devel/setup.bash" >> /etc/bash.bashrc && \
    echo "export ROS_MASTER_URI=http://hostcomputer:11311" >> /etc/bash.bashrc && \
    echo 'export ROS_IP=$(ip route get 1 | awk '/src/ {print $7; exit}')' >> /etc/bash.bashrc
    


CMD ["bash"]

