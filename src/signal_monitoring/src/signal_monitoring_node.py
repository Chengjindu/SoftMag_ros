#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import subprocess
import os


def get_config_param(param):
    # Adjust the path to locate start_config.json
    config_file = os.path.join(os.path.dirname(__file__), '../../../start_config.json')
    result = subprocess.run(['jq', '-r', f'.{param}', config_file], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()
    else:
        raise Exception(f"Failed to get config param '{param}': {result.stderr}")

class SignalChangeDetector:
    contact_trigger = True
    def __init__(self, sensor_id, window_size=20):
        self.sensor_id = sensor_id
        self.window_size = window_size
        self.contact_detected = False

        self.default_mode = get_config_param('default_pi_mode')
        self.set_default_mode(self.default_mode)

        self.data_window = {'x': [], 'y': [], 'z': []}
        self.change_flag = {'x': False, 'y': False, 'z': False}
        self.processed_sensor_data_subscriber = rospy.Subscriber(f'processed_sensor_data_{self.sensor_id}', Float32MultiArray, self.processed_callback)

        self.release_finish_subscriber = rospy.Subscriber('release_finish', Bool, self.release_finish_callback)
        self.restart_subscriber = rospy.Subscriber('restart', Bool, self.restart_callback)
        self.stop_all_subscriber = rospy.Subscriber('stop_all', Bool, self.stop_all_callback)
        self.contact_trigger_publisher = rospy.Publisher('contact_trigger', Bool, queue_size=10)
        self.contact_detected_publisher = rospy.Publisher('contact_detect', Bool, queue_size=10)
        self.mode_subscriber = rospy.Subscriber('mode_change', String, self.mode_callback)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    def set_default_mode(self, mode):
        self.current_mode = mode
        self.update_mode_parameters(self.current_mode)

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            SignalChangeDetector.contact_trigger = False
            self.contact_trigger_publisher.publish(Bool(data=False))
            self.reset_data_window()
            self.reset_change_flags()
            self.publish_diagnostics(True, f"Mode changed to {new_mode}, initializing contact flag...")
            self.current_mode = new_mode
            self.update_mode_parameters(new_mode)

    def update_mode_parameters(self, mode):
        if mode == "sensor":
            self.mean_threshold = 0.25      # mean value within a data window
            self.diff_threshold = 0.7       # difference between the first and last value within a window
            self.axes_required = 2
            self.lower_threshold = 0.3      # second check for absolute value
            self.upper_threshold = 0.8
        else:  # "gripper_automatic" || "gripper_testing"
            self.mean_threshold = 0.12      # the mean value is the feature parameter
            self.diff_threshold = 0.35      # difference is the additional condition
            self.axes_required = 1
            self.lower_threshold = 0.05
            self.upper_threshold = 0.3

    def processed_callback(self, msg):
        filtered_data = list(msg.data)
        self.contact_detection(filtered_data)

    def release_finish_callback(self, data):
        if data.data:
            self.publish_diagnostics(True, "Release finished, initializing contact flag...")

    def restart_callback(self, data):
        if data.data:
            SignalChangeDetector.contact_trigger = False
            self.contact_trigger_publisher.publish(Bool(data=False))
            self.reset_data_window()
            self.reset_change_flags()
            self.publish_diagnostics(True, "Release finished, initializing contact flag...")

    def stop_all_callback(self, data):
        if data.data:
            self.reset_data_window()
            self.reset_change_flags()
            self.publish_diagnostics(True, "Stopping...")

    def contact_detection(self, filtered_data):
        for axis, value in zip(['x', 'y', 'z'], filtered_data):
            self.data_window[axis].append(value)
            if len(self.data_window[axis]) > self.window_size:
                self.data_window[axis].pop(0)
                change_detected, mean_data, difference = self.analyze_change(self.data_window[axis])
                if change_detected:
                    self.change_flag[axis] = True
                    self.publish_diagnostics(True, f"Significant change detected on {axis} axis for {self.sensor_id}. Mean: {mean_data}, Difference: {difference}")
            else:
                self.publish_diagnostics(True, "Detecting signal changes...")

        # if all(self.change_flag.values()): # If changes in all axis are detected
        if list(self.change_flag.values()).count(True) >= self.axes_required:  # If changes are detected in the required number of axes
            if not self.contact_detected:   # When no contact is detected, or at the first run
                if not SignalChangeDetector.contact_trigger:    # When contact_trigger is set to be false (first run or reinitialized)
                    SignalChangeDetector.contact_trigger =True
                    for _ in range(5):
                        self.contact_trigger_publisher.publish(Bool(data=True))
                        # rospy.sleep(0.1)
                self.contact_detected = True    # Set status as contact detected
                self.contact_detected_publisher.publish(Bool(data=True))
                self.publish_diagnostics(True, f"Contact has been detected on {self.sensor_id}.")
            else:   # When there is contact detected in the last run, and changes are detected in at least two axes for this run
                self.contact_detected = False
                self.contact_detected_publisher.publish(Bool(data=False))
                self.publish_diagnostics(True, f"Contact has been removed on {self.sensor_id}.")

        norm = np.linalg.norm(filtered_data)        # Calculate the Euclidean norm of the sensing data vector
        # Perform the additional significant check
        if self.contact_detected:   # If contact is detected last time
            if norm < self.lower_threshold:  # Lower threshold check
                self.contact_detected = False
                self.contact_detected_publisher.publish(Bool(data=False))
                self.publish_diagnostics(True, f"No contacts on {self.sensor_id}.")

                self.reset_data_window()  # Optionally reset flags and data windows to avoid repeated triggers
                self.reset_change_flags()
            else:
                self.contact_detected = True
                self.contact_detected_publisher.publish(Bool(data=True))
                self.publish_diagnostics(True, "Contact ongoing...")
        else:
            if norm > self.upper_threshold:  # Upper threshold check
                self.contact_detected = True
                self.contact_detected_publisher.publish(Bool(data=True))
                self.publish_diagnostics(True, f"Contact has been detected on {self.sensor_id}.")

                self.reset_data_window()  # Optionally reset flags and data windows to avoid repeated triggers
                self.reset_change_flags()
            else:
                self.contact_detected = False
                self.contact_detected_publisher.publish(Bool(data=False))
                self.publish_diagnostics(True, "No significant changes detected.")


    def analyze_change(self, data):
        change_flag = False

        mean_data = np.abs(np.mean(data))   # Calculate the mean value of the data array
        # Calculate the difference between the last and first element of the data array
        difference = abs(data[-1] - data[0])

        if mean_data > self.mean_threshold or difference > self.diff_threshold:
            change_flag = True

        return change_flag, mean_data, difference

    def reset_change_flags(self):
        self.change_flag = {'x': False, 'y': False, 'z': False}

    def reset_data_window(self):
        self.data_window = {'x': [], 'y': [], 'z': []}

    def publish_diagnostics(self, status, message):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        status_msg = DiagnosticStatus()
        status_msg.name = f"SignalChangeDetector: {self.sensor_id}"
        status_msg.level = DiagnosticStatus.OK if status else DiagnosticStatus.WARN
        status_msg.message = message
        status_msg.hardware_id = self.sensor_id
        status_msg.values = [
            KeyValue("Sensor ID", self.sensor_id),
            KeyValue("Window Size", str(self.window_size)),
            KeyValue("Contact Detected", str(self.contact_detected)),
            KeyValue("Contact Triggered", str(self.contact_trigger))
        ]
        diag_msg.status.append(status_msg)
        self.diag_pub.publish(diag_msg)


if __name__ == '__main__':
    rospy.init_node('signal_change_detection_node')
    sensor_ids = ['S1', 'S2']  # Update with actual sensor IDs
    detectors = [SignalChangeDetector(sensor_id) for sensor_id in sensor_ids]
    rospy.spin()


