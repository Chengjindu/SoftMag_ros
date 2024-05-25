#!/bin/bash

# Function to kill the process started by SSH on the Raspberry Pi
kill_pi_process() {
  echo "Stopping the ROS system on the Raspberry Pi..."
  # Send Ctrl+C to the tmux session on the Raspberry Pi, mimicking a graceful shutdown signal
  ssh chengjindu@10.255.32.38 'tmux send-keys -t ros_session C-c'
  sleep 2  # Give some time for processes to handle the signal
  # Kill the tmux session itself, cleaning up all associated processes
  ssh chengjindu@10.255.32.38 'tmux kill-session -t ros_session'
}

# Function to kill local processes started by this script
kill_local_process() {
  echo "Stopping the local ROS system..."
  # Assuming start_local.sh starts processes that can be terminated by terminating start_local.sh itself
  # If start_local.sh uses tmux or screen, similar commands to kill those sessions can be added here
  pkill -f start_local.sh
}

# Trap Ctrl+C (SIGINT) and call functions to kill processes
trap 'kill_local_process; kill_pi_process; exit' SIGINT

# Starting the ROS system on the Raspberry Pi within a tmux session
echo "Starting the ROS system on the Raspberry Pi..."
ssh chengjindu@10.255.32.38 'tmux new-session -d -s ros_session "bash /home/chengjindu/SoftMag/Console/ros_workspace/start_console_pi.sh"' &
sleep 2  # Give some time for tmux to start the session

# Verify tmux session on Raspberry Pi
ssh chengjindu@10.255.32.38 'tmux list-sessions'

# Starting the local ROS system
echo "Starting the local ROS system..."
./start_local.sh &

# Wait for any background jobs to finish
wait

