#!/bin/bash

# Function to get the local IP address dynamically
get_local_ip() {
  hostname -I | awk '{print $1}'  # Retrieves the first IP address
}

# Function to dynamically set ROS environment variables
set_ros_env() {
  export ROS_IP=$(get_local_ip)
  export ROS_MASTER_URI="http://$ROS_IP:11311"
  echo "ROS_IP: $ROS_IP"
  echo "ROS_MASTER_URI: $ROS_MASTER_URI"
}

# Function to update start_config.json dynamically
update_start_config() {
  echo "Updating start_config.json with dynamic ROS configuration..."
  jq --arg ros_master "$ROS_MASTER_URI" --arg local_ip "$ROS_IP" \
    '.ros_master_uri=$ros_master | .local_ip=$local_ip' $CONFIG_FILE > temp_start_config.json
  mv temp_start_config.json $CONFIG_FILE
}

# Function to create or update start_config_pi.json on the Raspberry Pi
create_config_pi() {
  echo "Creating or updating start_config_pi.json on the Raspberry Pi..."
  jq --arg ros_master "$ROS_MASTER_URI" --arg pi_ip "$ROS_IP" \
    '.ros_master_uri=$ros_master | .local_ip=$pi_ip' $CONFIG_FILE > temp_config_pi.json
  scp temp_config_pi.json "$PI_USER@$PI_IP:/home/$PI_USER/SoftMag/Console/ros_workspace/start_config_pi.json"
  rm temp_config_pi.json  # Clean up temporary file
}

# Function to kill the process started by SSH on the Raspberry Pi
kill_pi_process() {
  echo "Stopping the ROS system on the Raspberry Pi..."
  ssh "$PI_USER@$PI_IP" 'screen -S ros_session -X quit'
}

# Function to kill local processes started by this script
kill_local_process() {
  echo "Stopping the local ROS system..."
  pkill -f start_local.sh
}

# Trap Ctrl+C (SIGINT) and call functions to kill processes
trap 'kill_local_process; kill_pi_process; exit' SIGINT

# Main Script
CONFIG_FILE="start_config.json"

# Retrieve parameters from start_config.json
PI_USER=$(jq -r '.pi_user' $CONFIG_FILE)
PI_IP=$(jq -r '.pi_ip' $CONFIG_FILE)
DEFAULT_PI_MODE=$(jq -r '.default_pi_mode' $CONFIG_FILE)

# Dynamically set ROS environment
set_ros_env

# Update start_config.json locally
update_start_config

# Start ROS master on the local machine
echo "Starting the ROS master on the local machine..."
roscore &
ROSCORE_PID=$!
sleep 5  # Wait for ROS master to initialize

# Set default mode parameter
rosparam set /default_pi_mode "$DEFAULT_PI_MODE"
export DEFAULT_PI_MODE="$DEFAULT_PI_MODE"

# Check if screen is installed on the Raspberry Pi
echo "Checking if screen is installed on the Raspberry Pi..."
ssh "$PI_USER@$PI_IP" 'command -v screen'

# Update start_config_pi.json dynamically
create_config_pi

# Start ROS system on the Raspberry Pi in a screen session
echo "Starting the ROS system on the Raspberry Pi..."
ssh "$PI_USER@$PI_IP" 'screen -dmS ros_session bash -c "bash /home/'"$PI_USER"'/SoftMag/Console/ros_workspace/start_console_pi.sh &> /home/'"$PI_USER"'/ros_session.log; exec bash"'
if [ $? -ne 0 ]; then
  echo "Failed to start screen session on the Raspberry Pi"
  exit 1
fi

# Verify screen session on Raspberry Pi
ssh "$PI_USER@$PI_IP" 'screen -list'

# Start the local ROS system
echo "Starting the local ROS system..."
./start_local.sh &

# Wait for background jobs to finish
wait

