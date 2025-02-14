#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/jazzy/setup.bash
# source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/sayantani/Documents/Winter/project/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eno0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
