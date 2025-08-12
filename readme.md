source /opt/ros/humble/setup.bash && colcon build
to build

ros-humble-rmw-cyclonedds-cpp or
rmw_fastrtps_cpp package
ros-humble-control-msgs
sudo apt install ros-humble-moveit
apt update && apt install ros-humble-joint-state-publisher-gui
ros-humble-xacro

apt update && apt install ros-humble-rmw-fastrtps-cpp ros-humble-control-msgs ros-humble-moveit ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ur


install everytime for docker

https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_manipulation.html

# -isaac environment setup-

## RedBall

path /World/RedBall

materials /World/Looks/OmniPBR

rigid body enabled, kinematic enabled, collision enabled

scale 0.1

## Camera

name : rsd455

camera pseudo depth

action graph here

    on playback tick - no mods
    isaac run one simulation frame - no mods
    isaac create render product - inputs:cameraprim -> set to depth camera
    ros2 context - no mods
    ros2 camera helper - frameid -> depth, topic name -> rsd455_depth, type-> depth, use system time-> check

camera color
    on playback tick - no mods
    isaac run one simulation frame - no mods
    isaac create render product - inputs:cameraprim -> set to color camera
    ros2 context - no mods
    ros2 camera helper - frameid -> rsd455, topic name -> rsd455_img, type-> rgb, use system time-> check

## Robot
