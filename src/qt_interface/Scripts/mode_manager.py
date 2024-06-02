#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import subprocess
import signal
import sys
import os
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import subprocess

def get_config_param(param):
    # Adjust the path to locate start_config.json
    config_file = os.path.join(os.path.dirname(__file__), '../../../start_config.json')
    result = subprocess.run(['jq', '-r', f'.{param}', config_file], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()
    else:
        raise Exception(f"Failed to get config param '{param}': {result.stderr}")

class ModeManager:
    def __init__(self):
        rospy.init_node('mode_manager')
        self.current_mode = None
        self.current_process = None
        self.sensor_process = None
        # self.current_sensor_node_name = None

        # Read configuration parameters
        self.pi_user = get_config_param('pi_user')
        self.pi_address = get_config_param('pi_ip')
        self.start_env_loader_local_path = "/home/chengjin/Projects/SoftMag/ros_workspace/start_env_loader.sh"
        self.start_env_loader_pi_path = "/home/chengjindu/SoftMag/Console/ros_workspace/start_env_loader_pi.sh"

        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.mode_pub = rospy.Publisher('mode_change', String, queue_size=10)  # Publisher for mode changes
        self.init_services()
        rospy.loginfo("ModeManager initialized and services started.")  # Debug log

        # Switch to automatic mode by default on initialization
        self.switch_motor_mode('Sensing')
        self.current_mode = 'Sensing'

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Set up a timer to periodically publish diagnostics
        self.diagnostics_timer = rospy.Timer(rospy.Duration(5), self.publish_periodic_diagnostics)

    def init_services(self):
        rospy.Service('/switch_to_sensing', Trigger, self.switch_to_sensing)
        rospy.Service('/switch_to_automatic', Trigger, self.switch_to_automatic)
        rospy.Service('/switch_to_testing', Trigger, self.switch_to_testing)

    def switch_to_automatic(self, req):
        if self.current_mode != 'Automatic':
            if self.current_mode == 'Sensing':
                self.switch_sensor_node('gripper')
                self.restart_data_processing_node()
            self.publish_mode_change("Automatic")

            self.switch_motor_mode('Automatic')
            self.publish_diagnostics(True, "Switched to Automatic mode")
            return TriggerResponse(success=True, message="Switched to Automatic mode")
            # The arduino-cli method is too slow and unreliable for real-time usage
            # if self.upload_firmware('/home/chengjin/Projects/SoftMag/ros_workspace/src/Arduino/Regulator/automatic_regulator/automatic_regulator.ino'):
            #     return TriggerResponse(success=True, message="Switched to automatic mode")
            # else:
            #     return TriggerResponse(success=False, message="Failed to switch to automatic mode")
        else:
            return TriggerResponse(success=True, message="Already in automatic mode")

    def switch_to_testing(self, req):
        if self.current_mode != 'Testing':
            if self.current_mode == 'Sensing':
                self.switch_sensor_node('gripper')
                self.restart_data_processing_node()
            self.publish_mode_change("Testing")

            self.switch_motor_mode('Testing')
            self.publish_diagnostics(True, "Switched to Testing mode")
            return TriggerResponse(success=True, message="Switched to Testing mode")
            # if self.upload_firmware('/home/chengjin/Projects/SoftMag/ros_workspace/src/Arduino/Regulator/testing_regulator/testing_regulator.ino'):
            #     return TriggerResponse(success=True, message="Switched to testing mode")
            # else:
            #     return TriggerResponse(success=False, message="Failed to switch to testing mode")
        else:
            return TriggerResponse(success=True, message="Already in testing mode")

    def switch_to_sensing(self, req):
        if self.current_mode != 'Sensing':
            self.switch_sensor_node('sensor')
            self.restart_data_processing_node()
            self.publish_mode_change("Sensing")

            self.switch_motor_mode('Sensing')
            self.publish_diagnostics(True, "Switched to Sensing mode")
            return TriggerResponse(success=True, message="Switched to Sensing mode")
        else:
            return TriggerResponse(success=True, message="Already in Sensing mode")

    def switch_motor_mode(self, mode):
        # Define the current nodes to be killed before switching modes
        global command
        nodes_to_kill = ['automatic_motor_control_node', 'testing_motor_control_node']

        # Kill the existing nodes locally
        for node in nodes_to_kill:
            kill_command = f"rosnode kill {node}"
            rospy.loginfo(f"Executing kill command: {kill_command}")
            kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
            if kill_result.returncode == 0:
                rospy.loginfo(f"{node} has been killed")
            else:
                rospy.logwarn(f"Failed to kill {node}. It might not be running. Error: {kill_result.stderr}")

        if self.current_process:
            self.current_process.terminate()
            self.current_process.wait()
            self.current_process = None

        if mode == 'Sensing':
            self.current_mode = mode
            return

        if mode == 'Automatic':
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi automatic_motor_launch.launch'"
        elif mode == 'Testing':
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi testing_motor_launch.launch'"

        rospy.loginfo(f"Executing command: {command}")
        self.current_process = subprocess.Popen(command, shell=True)
        self.current_mode = mode

    def switch_sensor_node(self, node_type):
        if self.sensor_process:
            kill_command = "rosnode kill sensors_i2c_reading_node"
            rospy.loginfo(f"Executing kill command: {kill_command}")
            while True:
                kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
                if kill_result.returncode == 0:
                    rospy.loginfo(f"sensors_i2c_reading_node has been killed")
                    break
                else:
                    rospy.logwarn(f"Failed to kill sensors_i2c_reading_node. Retrying...")

            self.sensor_process.terminate()
            self.sensor_process.wait()
            self.sensor_process = None

        if node_type == 'sensor':
            # self.current_sensor_node_name = "sensors_i2c_sensor_reading_node"
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi sensing_sensor_launch.launch'"
        elif node_type == 'gripper':
            # self.current_sensor_node_name = "sensors_i2c_gripper_reading_node"
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi sensing_gripper_launch.launch'"

        rospy.loginfo(f"Executing command: {command}")
        self.sensor_process = subprocess.Popen(command, shell=True)

    def restart_data_processing_node(self):
        # Kill the existing data processing node
        kill_command = "rosnode kill data_processing_node"
        rospy.loginfo(f"Executing kill command: {kill_command}")
        kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
        if kill_result.returncode == 0:
            rospy.loginfo(f"data_processing_node has been killed")
        else:
            rospy.logwarn(f"Failed to kill data_processing_node. It might not be running. Error: {kill_result.stderr}")

        command = f"bash -c 'source {self.start_env_loader_local_path} && roslaunch launch_project data_processing_launch.launch'"

        rospy.loginfo(f"Restarting data processing node...")
        self.current_process = subprocess.Popen(command, shell=True)
        rospy.loginfo("data_processing_node restarted successfully.")
        # try:
        #     restart_process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        #     stdout, stderr = restart_process.communicate()
        #     if restart_process.returncode == 0:
        #         rospy.loginfo("data_processing_node restarted successfully.")
        #     else:
        #         rospy.logerr(f"Failed to restart data_processing_node. Error: {stderr.decode('utf-8')}")
        # except Exception as e:
        #     rospy.logerr(f"Exception occurred while restarting data_processing_node: {str(e)}")

    def publish_mode_change(self, mode):
        mode_msg = String(data=mode)
        self.mode_pub.publish(mode_msg)
    def upload_firmware(self, firmware_path):
        compile_result = subprocess.run(['arduino-cli', 'compile', '--fqbn', 'arduino:avr:mega', firmware_path])
        if compile_result.returncode != 0:
            self.publish_diagnostics(False, "Failed to compile firmware")
            return False

        upload_result = subprocess.run(['arduino-cli', 'upload', '--fqbn', 'arduino:avr:mega', '--port', '/dev/ttyACM0', firmware_path])
        if upload_result.returncode == 0:
            self.publish_diagnostics(True, "Firmware uploaded successfully")
            return True
        else:
            rospy.logerr("Failed to upload firmware")
            return False

    def signal_handler(self, sig, frame):
        if self.current_process:
            self.current_process.terminate()
            self.current_process.wait()
        rospy.signal_shutdown("SIGINT received")
        self.publish_diagnostics(False, "SIGINT received, shutting down")
        sys.exit(0)

    def publish_periodic_diagnostics(self, event):
        self.publish_diagnostics(True, f"Current mode is: {self.current_mode}")

    def publish_diagnostics(self, status, message):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        status_msg = DiagnosticStatus()
        status_msg.name = "ModeManager"
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.ERROR
        status_msg.message = message
        status_msg.hardware_id = "ModeManager"
        status_msg.values = [KeyValue("Current Mode", self.current_mode)]
        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)

if __name__ == "__main__":
    mode_manager = ModeManager()
    rospy.spin()
