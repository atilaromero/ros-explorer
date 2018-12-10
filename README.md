# ROS-explorer is a ROS module that implements some algorithms in robotics, using turtlebot, stage and gazebo.

## Docker (optional)

The provided Dockerfile installs all required ROS dependencies.

To run it, first pull the docker image (about 1GB):
  
    docker pull atilaromero/ros-kinetic

The following bash function makes easier to launch a container able to display graphics (some of the options may be actually unrequired):
  
    dkr ()
    {
      xhost +
      docker run --rm -it \
              -w "`pwd`" -v "`pwd`":"`pwd`":Z            `# share current dir` \
              -e DISPLAY                                 `# share display` \
              -v /tmp/.X11-unix/:/tmp/.X11-unix/         `# share display` \
              -e GDK_BACKEND                             `#` \
              -e GDK_SCALE                               `#` \
              -e GDK_DPI_SCALE                           `#` \
              -e QT_DEVICE_PIXEL_RATIO                   `#` \
              --device /dev/dri                          `#` \
              --device /dev/snd                          `#` \
              -v /etc/localtime:/etc/localtime:ro        `# share timezone` \
              "$@"                                        # function arguments
    }
    export -f dkr

