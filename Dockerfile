FROM osrf/ros:kinetic-desktop-full

# Prefer ROS repository over Ubuntu
# ros repository attributes can be retrieved with 'apt-cache policy'
# a priority above 1000 will allow even downgrades
RUN echo '\
Package: * \n\
Pin: release o=ROS \n\
Pin-Priority: 1001 \n\
' > /etc/apt/preferences

# ros-kinetic-librealsense.postinst fails to detect docker
RUN apt-get update \
&&  apt-get install -y \
      ros-kinetic-librealsense \
||  rm /var/lib/dpkg/info/ros-kinetic-librealsense.postinst -f \
&&  apt-get -f install \
&&  rm -rf /var/lib/apt/lists/*

RUN apt-get update \
&&  apt-get install -y \
      tmux \
      git \
      vim \
      ros-kinetic-turtlebot-gazebo \
      ros-kinetic-turtlebot-stage \
      python-scipy \
&&  apt-get upgrade -y \
&&  rm -rf /var/lib/apt/lists/*
