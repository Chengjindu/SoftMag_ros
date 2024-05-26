#!/bin/bash

# Set up environment
source start_env_loader.sh

# Launch rqt_graph and rqt_gui (in the background if desired)
rqt &

roslaunch launch_project project_launch.launch

