#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import signal
import sys

class NodeManager:
    def __init__(self):
        rospy.init_node('node_manager')
        self.current_mode = None
        self.current_process = None
        self.init_services()
        rospy.loginfo("NodeManager initialized and services started.")  # Debug log

        # Switch to automatic mode by default on initialization
        self.switch_motor_mode('automatic')

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def init_services(self):
        rospy.Service('/switch_to_automatic', Trigger, self.switch_to_automatic)
        rospy.Service('/switch_to_testing', Trigger, self.switch_to_testing)

    def switch_to_automatic(self, req):
        if self.current_mode != 'automatic':
            self.switch_motor_mode('automatic')
            if self.upload_firmware('/home/chengjin/Projects/SoftMag/ros_workspace/src/Arduino/Regulator/automatic_regulator/automatic_regulator.ino'):
                return TriggerResponse(success=True, message="Switched to automatic mode")
            else:
                return TriggerResponse(success=False, message="Failed to switch to automatic mode")
        else:
            return TriggerResponse(success=True, message="Already in automatic mode")

    def switch_to_testing(self, req):
        if self.current_mode != 'testing':
            self.switch_motor_mode('testing')
            if self.upload_firmware('/home/chengjin/Projects/SoftMag/ros_workspace/src/Arduino/Regulator/testing_regulator/testing_regulator.ino'):
                return TriggerResponse(success=True, message="Switched to testing mode")
            else:
                return TriggerResponse(success=False, message="Failed to switch to testing mode")
        else:
            return TriggerResponse(success=True, message="Already in testing mode")

    def upload_firmware(self, firmware_path):
        compile_result = subprocess.run(['arduino-cli', 'compile', '--fqbn', 'arduino:avr:mega', firmware_path])
        if compile_result.returncode != 0:
            rospy.logerr("Failed to compile firmware")
            return False

        upload_result = subprocess.run(['arduino-cli', 'upload', '--fqbn', 'arduino:avr:mega', '--port', '/dev/ttyACM0', firmware_path])
        if upload_result.returncode == 0:
            rospy.loginfo("Firmware uploaded successfully")
            return True
        else:
            rospy.logerr("Failed to upload firmware")
            return False

    def switch_motor_mode(self, mode):
        # Define the current nodes to be killed before switching modes
        nodes_to_kill = ['automatic_motor_control_node', 'testing_motor_control_node']

        # Kill the existing nodes locally
        for node in nodes_to_kill:
            kill_command = f"rosnode kill {node}"
            rospy.loginfo(f"Executing kill command: {kill_command}")
            kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
            if kill_result.returncode == 0:
                rospy.loginfo(f"Kill executed on node: {node}")
            else:
                rospy.logwarn(f"Failed to executed kill on node: {node}. It might not be running. Error: {kill_result.stderr}")

        if self.current_process:
            self.current_process.terminate()
            self.current_process.wait()
            self.current_process = None

        pi_user = "chengjindu"
        pi_address = "10.255.32.38"
        ros_env_loader_path = "/home/chengjindu/SoftMag/Console/ros_workspace/ros_env_loader.sh"

        if mode == 'automatic':
            command = f"ssh {pi_user}@{pi_address} 'source {ros_env_loader_path} && roslaunch launch_console_pi automatic_launch.launch'"
        elif mode == 'testing':
            command = f"ssh {pi_user}@{pi_address} 'source {ros_env_loader_path} && roslaunch launch_console_pi testing_launch.launch'"

        rospy.loginfo(f"Executing command: {command}")
        self.current_process = subprocess.Popen(command, shell=True)
        self.current_mode = mode

    def signal_handler(self, sig, frame):
        if self.current_process:
            self.current_process.terminate()
            self.current_process.wait()
        rospy.loginfo("SIGINT received. Shutting down gracefully...")
        rospy.signal_shutdown("SIGINT received")
        sys.exit(0)

if __name__ == "__main__":
    node_manager = NodeManager()
    rospy.spin()
