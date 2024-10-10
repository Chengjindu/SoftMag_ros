#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import subprocess
import threading
import signal
import sys
import os
import time


def get_config_param(param):
    # Adjust the path to locate start_config.json (# this is an useful comment)
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
        self.current_sensor_process = None
        self.current_data_process = None
        self.current_motor_process = None
        self.serial_process = None
        self.current_sensor_node_name = None
        self.current_motor_node_name = None
        self.serial_node_name = "serial_node"

        # Read configuration parameters
        self.pi_user = get_config_param('pi_user')
        self.pi_address = get_config_param('pi_ip')
        self.default_mode = get_config_param('default_pi_mode')
        self.start_env_loader_local_path = "/home/chengjin/Projects/SoftMag/ros_workspace/start_env_loader.sh"
        self.start_env_loader_pi_path = "/home/chengjindu/SoftMag/Console/ros_workspace/start_env_loader_pi.sh"

        # Initialize ROS publishers and services
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.mode_pub = rospy.Publisher('mode_change', String, queue_size=10)
        self.init_services()

        # Initialize default mode
        self.set_default_mode(self.default_mode)

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Periodic diagnostics publishing
        self.diagnostics_timer = rospy.Timer(rospy.Duration(5), self.publish_periodic_diagnostics)

    def set_default_mode(self, mode):   # Set the default mode of operation
        self.switch_motor_mode(mode)
        self.current_mode = mode
        if mode == 'sensor':
            self.switch_serial_node('stop')
            self.current_sensor_node_name = "sensor_reading_node"
            self.current_motor_node_name = None
        elif mode == 'gripper_automatic':
            self.switch_serial_node('start')
            self.current_sensor_node_name = "gripper_reading_node"
            self.current_motor_node_name = "automatic_motor_control_node"
        elif mode == 'gripper_testing':
            self.switch_serial_node('start')
            self.current_sensor_node_name = "gripper_reading_node"
            self.current_motor_node_name = "testing_motor_control_node"

    def init_services(self):    # Initialize ROS services for mode switching
        rospy.Service('/switch_to_sensor', Trigger, self.switch_to_sensor)
        rospy.Service('/switch_to_gripper_automatic', Trigger, self.switch_to_gripper_automatic)
        rospy.Service('/switch_to_gripper_testing', Trigger, self.switch_to_gripper_testing)

    def switch_to_sensor(self, req):    # Service response when switched to "sensor" mode
        if self.current_mode != 'sensor':
            self.switch_sensor_node('sensor')
            self.restart_sensors_data_processing_node()
            self.publish_mode_change("sensor")

            self.switch_motor_mode('sensor')
            self.switch_serial_node('stop')
            self.publish_diagnostics(True, "Switched to sensor mode")
            return TriggerResponse(success=True, message="Switched to sensor mode")
        else:
            return TriggerResponse(success=True, message="Already in sensor mode")

    def switch_to_gripper_automatic(self, req): # Service response when switched to "gripper_automatic" mode
        if self.current_mode != 'gripper_automatic':
            if self.current_mode == 'sensor':
                self.switch_sensor_node('gripper')
                self.restart_sensors_data_processing_node()
            self.publish_mode_change("gripper_automatic")

            self.switch_motor_mode('gripper_automatic')
            self.switch_serial_node('start')
            self.publish_diagnostics(True, "Switched to gripper_automatic mode")
            return TriggerResponse(success=True, message="Switched to gripper_automatic mode")

        else:
            return TriggerResponse(success=True, message="Already in gripper_automatic mode")

    def switch_to_gripper_testing(self, req):   # Service response when switched to "gripper_testing" mode
        if self.current_mode != 'gripper_testing':
            if self.current_mode == 'sensor':
                self.switch_sensor_node('gripper')
                self.restart_sensors_data_processing_node()
            self.publish_mode_change("gripper_testing")

            self.switch_motor_mode('gripper_testing')
            self.switch_serial_node('start')
            self.publish_diagnostics(True, "Switched to gripper_testing mode")
            return TriggerResponse(success=True, message="Switched to gripper_testing mode")

        else:
            return TriggerResponse(success=True, message="Already in gripper_testing mode")

    def switch_sensor_node(self, node_type):        # Operation logic for the sensor node switching
        if self.current_sensor_node_name is not None:
            kill_command = f"rosnode kill {self.current_sensor_node_name}"
            rospy.loginfo(f"Executing kill command: {kill_command}")
            retry_limit = 5  # Maximum number of retries
            retries = 0

            while retries < retry_limit:
                kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
                if kill_result.returncode == 0:
                    rospy.loginfo(f"{self.current_sensor_node_name} has been killed")
                    break
                else:
                    retries += 1
                    rospy.logwarn(f"Failed to kill {self.current_sensor_node_name}. Retrying...")
                    time.sleep(1)

        if self.current_sensor_process and not isinstance(self.current_sensor_process, str):
            self.current_sensor_process.terminate()
            self.current_sensor_process.wait()
            self.current_sensor_process = None

        if node_type == 'sensor':
            self.current_sensor_node_name = "sensor_reading_node"
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi sensor_launch.launch'"
        elif node_type == 'gripper':
            self.current_sensor_node_name = "gripper_reading_node"
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi gripper_launch.launch'"

        rospy.loginfo(f"Executing command: {command}")
        self.current_sensor_process = subprocess.Popen(command, shell=True)

    def switch_motor_mode(self, mode):  # Operation logic for the motor node switching
        if self.current_motor_node_name is not None:
            kill_command = f"rosnode kill {self.current_motor_node_name}"
            rospy.loginfo(f"Executing kill command: {kill_command}")
            kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
            if kill_result.returncode == 0:
                rospy.loginfo(f"{self.current_motor_node_name} has been killed")
            else:
                rospy.logwarn(f"Failed to kill {self.current_motor_node_name}. It might not be running. Error: {kill_result.stderr}")

        if self.current_motor_process and not isinstance(self.current_motor_process, str):
            self.current_motor_process.terminate()
            self.current_motor_process.wait()
            self.current_motor_process = None

        if mode == 'sensor':
            self.current_mode = mode
            return
        if mode == 'gripper_automatic':
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi automatic_motor_launch.launch'"
        elif mode == 'gripper_testing':
            command = f"ssh {self.pi_user}@{self.pi_address} 'source {self.start_env_loader_pi_path} && roslaunch launch_console_pi testing_motor_launch.launch'"

        rospy.loginfo(f"Executing command: {command}")
        self.current_motor_process = subprocess.Popen(command, shell=True)
        self.current_mode = mode

    def restart_sensors_data_processing_node(self):     # Restart sensors_data_processing_node is necessary when sensor addresses change
        kill_command = "rosnode kill sensors_data_processing_node"
        rospy.loginfo(f"Executing kill command: {kill_command}")
        kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
        if kill_result.returncode == 0:
            rospy.loginfo(f"sensors_data_processing_node has been killed")
        else:
            rospy.logwarn(f"Failed to kill sensors_data_processing_node. It might not be running. Error: {kill_result.stderr}")

        command = f"bash -c 'roslaunch launch_project sensors_data_processing_launch.launch'"

        self.current_data_process = subprocess.Popen(command, shell=True)
        rospy.loginfo("sensors_data_processing_node restarted successfully.")

    def publish_mode_change(self, mode):    # Publish the mode change event
        mode_msg = String(data=mode)
        self.mode_pub.publish(mode_msg)

    def switch_serial_node(self, action):   # Start or stop the serial node
        if action == 'start':
            if self.serial_process is None or self.serial_process.poll() is not None:
                command = f"rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600"
                rospy.loginfo(f"Starting serial node with command: {command}")
                self.serial_process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

                def monitor_process(proc):
                    try:
                        stdout, stderr = proc.communicate()
                        rospy.loginfo(f"Serial node output: {stdout.decode()}")
                        if stderr:
                            rospy.logwarn(f"Serial node error: {stderr.decode()}")
                    except Exception as e:
                        rospy.logwarn(f"Error reading serial node output: {e}")

                threading.Thread(target=monitor_process, args=(self.serial_process,)).start()

        elif action == 'stop':
            if self.serial_process:
                kill_command = "rosnode kill /serial_node"
                rospy.loginfo(f"Executing kill command: {kill_command}")
                kill_result = subprocess.run(kill_command, shell=True, capture_output=True, text=True)
                if kill_result.returncode == 0:
                    rospy.loginfo(f"/serial_node has been killed")
                else:
                    rospy.logwarn(f"Failed to kill /serial_node. It might not be running. Error: {kill_result.stderr}")

    def upload_firmware(self, firmware_path):   # Compile and upload firmware to the Arduino
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

    def signal_handler(self, sig, frame):    # Handle shutdown signals to gracefully terminate processes
        if self.current_sensor_process and not isinstance(self.current_sensor_process, str):
            self.current_sensor_process.terminate()
            self.current_sensor_process.wait()
        if self.current_data_process and not isinstance(self.current_data_process, str):
            self.current_data_process.terminate()
            self.current_data_process.wait()
        if self.current_motor_process and not isinstance(self.current_motor_process, str):
            self.current_motor_process.terminate()
            self.current_motor_process.wait()
        if self.serial_process:
            self.serial_process.terminate()
            self.serial_process.wait()
        rospy.signal_shutdown("SIGINT received")
        self.publish_diagnostics(False, "SIGINT received, shutting down")
        sys.exit(0)

    def publish_periodic_diagnostics(self, event):  # Periodically publish diagnostics
        self.publish_diagnostics(True, f"Current mode is: {self.current_mode}")

    def publish_diagnostics(self, status, message): # Publish diagnostic information
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
