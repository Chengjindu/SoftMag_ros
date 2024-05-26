#!/bin/bash

# Function to kill the process started by SSH on the Raspberry Pi
kill_pi_process() {
  echo "Stopping the ROS system on the Raspberry Pi..."
  ssh chengjindu@$(jq -r '.pi_ip' start_config.json) 'screen -S ros_session -X quit'
}

# Function to kill local processes started by this script
kill_local_process() {
  echo "Stopping the local ROS system..."
  pkill -f start_local.sh
}

# Trap Ctrl+C (SIGINT) and call functions to kill processes
trap 'kill_local_process; kill_pi_process; exit' SIGINT

# Check if screen is installed on the Raspberry Pi
echo "Checking if screen is installed on the Raspberry Pi..."
ssh chengjindu@$(jq -r '.pi_ip' start_config.json) 'command -v screen'

# Starting the ROS system on the Raspberry Pi within a screen session
echo "Starting the ROS system on the Raspberry Pi..."
ssh chengjindu@$(jq -r '.pi_ip' start_config.json) 'screen -dmS ros_session bash -c "bash /home/chengjindu/SoftMag/Console/ros_workspace/start_console_pi.sh &> /home/chengjindu/ros_session.log; exec bash"'
if [ $? -ne 0 ]; then
  echo "Failed to start screen session on the Raspberry Pi"
  exit 1
fi

# Verify screen session on Raspberry Pi
ssh chengjindu@$(jq -r '.pi_ip' start_config.json) 'screen -list'

# Starting the local ROS system
echo "Starting the local ROS system..."
./start_local.sh &

# Wait for any background jobs to finish
wait

