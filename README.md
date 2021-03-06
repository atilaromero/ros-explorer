# ROS-explorer is a ROS module that implements some algorithms in robotics, using turtlebot, stage and gazebo.

## Docker (optional)

The provided Dockerfile installs all required ROS dependencies.

To run it, first pull the docker image (about 1GB):
  
    docker pull atilaromero/ros-kinetic

The following bash function makes easier to launch a container able to display graphics:
  
    dkr ()
    {
      xhost +                                             # allows containers to access host's display
      docker run --rm -it \
              -w "`pwd`" -v "`pwd`":"`pwd`":Z            `# share current dir` \
              -e DISPLAY                                 `# share display` \
              -v /tmp/.X11-unix/:/tmp/.X11-unix/         `# share display` \
              -v /etc/localtime:/etc/localtime:ro        `# share timezone` \
              "$@"                                        # function arguments (atilaromero/ros-kinetic for example)
    }
    export -f dkr

Using this bash function, starting a ros-kinetic container can be done this way:

    dkr atilaromero/ros-kinetic

Or, without the function:
    
      xhost +                                             # allows containers to access host's display
      docker run --rm -it \
              -w "`pwd`" -v "`pwd`":"`pwd`":Z            `# share current dir` \
              -e DISPLAY                                 `# share display` \
              -v /tmp/.X11-unix/:/tmp/.X11-unix/         `# share display` \
              -v /etc/localtime:/etc/localtime:ro        `# share timezone` \
              atilaromero/kinetic

Depending on acceleration card, it may be necessary to install inside the container the same version of graphic drivers running on the host and to add the options "--device /dev/dri" and "--privileged" to the "docker run" call.

This docker image is a generic environment and does not contains the ros-explorer module, which can be installed following the next section.

## Instalation

The following instructions can used both with or without docker.

To install ros-explorer:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/atilaromero/ros-explorer.git
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    echo $'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

## Usage

to start a turtlebot simulation:

    roslaunch turtlebot_stage turtlebot_in_stage.launch

To go to the ros-explorer folder:

    roscd ros-explorer
    
To move the robot using coodenates (without obstacle avoidance):

    rosrun ros-explorer move.py 1 0

### Exploration

The main functionality of ROS-explorer is a program that controls the robot and uses the laser scanner readings to build a map. It can be started with:
    
    rosrun ros-explorer explore.py
    
It uses a single ROS topic as input, "/scan". It then builds a map using harmonic potential fields and moves the robot to an unexplored area. 

It publishes the map in the topic "/mymap" and the potential field in the topic "/myfield".

When the program is interrupted with Ctrl-C, it saves the current map in a file named worldmap-DATE-TIME.png.

