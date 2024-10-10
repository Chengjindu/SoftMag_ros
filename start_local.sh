#!/bin/bash

# Set up environment
source start_env_loader.sh

# Ensure ROS master is running
until rostopic list; do
  echo "Waiting for ROS master to start..."
  sleep 2
done

# Launch rqt_graph and rqt_gui (in the background if desired)
rqt &
# rqt_graph &

roslaunch launch_project project_launch.launch

