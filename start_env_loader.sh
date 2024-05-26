#!/bin/bash
echo "Setting ROS Master URI and ROS_IP..."
ROS_MASTER_URI=$(jq -r '.ros_master_uri' start_config.json)
export ROS_MASTER_URI
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

ROS_IP=$(jq -r '.local_ip' start_config.json)
export ROS_IP
echo "ROS_IP: $ROS_IP"

#source ~/.bashrc

echo "Activating virtual environment..."
source ~/Projects/SoftMag/softmagenv/bin/activate
echo "Virtual environment activated."

echo "Sourcing ROS Noetic setup..."
source /opt/ros/noetic/setup.bash
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"


echo "Sourcing ROS workspace setup..."
source ~/Projects/SoftMag/ros_workspace/devel/setup.bash
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"


