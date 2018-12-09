FROM osrf/ros:kinetic-desktop-full

# Prefer ROS repository over Ubuntu
# ros repository attributes can be retrieved with 'apt-cache policy'
# a priority above 1000 will allow even downgrades
RUN echo ' \n\
  Package: * \n\
  Pin: release o=ROS \n\
  Pin-Priority: 1001 \n\
  ' > /etc/apt/preferences

# ros-kinetic-librealsense.postinst fails to detect docker
RUN apt-get update
RUN apt-get install -y \
      ros-kinetic-librealsense \
||  rm /var/lib/dpkg/info/ros-kinetic-librealsense.postinst -f \
&&  apt-get -f install
RUN apt-get update
RUN apt-get install -y \
      tmux \
      git \
      ros-kinetic-turtlebot-gazebo \
&&  rm -rf /var/lib/apt/lists/*
RUN apt-get update
RUN apt-get install -y \
      ros-kinetic-turtlebot-stage \


