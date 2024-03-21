#!/bin/bash

echo "Setting ROS Master URI and ROS_IP..."

export ROS_MASTER_URI=http://10.255.32.70:11311
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

export ROS_IP=10.255.32.70
echo "ROS_IP: $ROS_IP"

# source ~/.bashrc

source ~/Projects/SoftMag/softmagenv/bin/activate

source /opt/ros/noetic/setup.bash

source ~/Projects/SoftMag/ros_workspace/devel/setup.bash

# export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
echo "PYTHONPATH: $PYTHONPATH"

# Launch rqt_graph and rqt_gui (in the background if desired)
rqt &

roslaunch launch_project project_launch.launch

